#include "mq.h"
#include <math.h>
#include <string.h>

#ifndef MQ_MOVE_CRUISE_RPM
#define MQ_MOVE_CRUISE_RPM 2
#endif
#ifndef MQ_MOVE_NEAR_RPM
#define MQ_MOVE_NEAR_RPM 1
#endif
#ifndef MQ_TURN_RPM
#define MQ_TURN_RPM 1
#endif
#if MQ_MOVE_CRUISE_RPM < 0 || MQ_MOVE_CRUISE_RPM > INT16_MAX || \
    MQ_MOVE_NEAR_RPM < 0 || MQ_MOVE_NEAR_RPM > INT16_MAX || \
    MQ_TURN_RPM < 0 || MQ_TURN_RPM > INT16_MAX
#error "Wheel RPM magnitudes must be integers from 0 to INT16_MAX"
#endif

static bool after(uint32_t a,uint32_t b) { return (int32_t)(a-b)>0; }
static float absf(float a) { return a<0.f?-a:a; }
static float wrap(float a) { return atan2f(sinf(a),cosf(a)); }
static bool zero_sample(const MQOdom *o) {
  return o->left_tenths_rpm>=-2 && o->left_tenths_rpm<=2 &&
         o->right_tenths_rpm>=-2 && o->right_tenths_rpm<=2;
}
static MQDriveView view(MQ *q) { return q->drive.view(q->drive_user); }
bool mq_is_executing(const MQ *q) {
  return q->state==MQ_ARMING || q->state==MQ_MOVE || q->state==MQ_TURN ||
         q->state==MQ_BARRIER || q->state==MQ_STOPPING;
}
void mq_init(MQ *q,uint64_t boot_id,const MQDriveOps *ops,void *user) {
  memset(q,0,sizeof(*q));q->boot_id=boot_id;q->drive=*ops;q->drive_user=user;
  q->odom.epoch=1;q->state=MQ_IDLE;
  if(!boot_id){q->first_error=MQ_ERR_BOOT;q->critical_latched=true;}
}
void mq_fault(MQ *q,uint32_t error,uint32_t now) {
  if(error==MQ_ERR_UART || error==MQ_ERR_BOOT)q->critical_latched=true;
  if(!q->first_error){q->first_error=error;q->first_error_tick_ms=now;}
  if(q->state!=MQ_STOPPING){
    q->state=MQ_STOPPING;q->target_pending=false;
    q->physical_stop_confirmed=false;q->drive.stop(q->drive_user,now);
  }
}
void mq_sample(MQ *q,const MQSample *s) {
  MQOdom *o=&q->odom;
  if(!s->valid || s->left_tenths_rpm < -150 || s->left_tenths_rpm > 150 ||
     s->right_tenths_rpm < -150 || s->right_tenths_rpm > 150) {
    o->valid=false;o->discontinuities++;
    if(mq_is_executing(q))mq_fault(q,MQ_ERR_ODOM,s->tick_ms);
    return;
  }
  if(o->initialized && s->sequence==o->sample_sequence) return;
  float l=(float)s->left_tenths_rpm*(MQ_PI*MQ_DIAMETER_M/600.f);
  float r=-(float)s->right_tenths_rpm*(MQ_PI*MQ_DIAMETER_M/600.f);
  float v=(l+r)*.5f,w=(r-l)/MQ_SEPARATION_M;
  bool continuous=true;
  if(o->initialized) {
    uint32_t dt=s->tick_ms-o->measurement_tick_ms;
    continuous=s->sequence-o->sample_sequence==1u && dt>0 && dt<=MQ_SAMPLE_TIMEOUT_MS;
    if(continuous && o->valid) {
      float seconds=(float)dt*.001f;
      float ds=(o->previous_v+v)*.5f*seconds;
      float angle=(o->previous_w+w)*.5f*seconds;
      float half=angle*.5f,scale=absf(half)>.000001f?sinf(half)/half:1.f;
      o->x+=ds*scale*cosf(o->yaw+half);o->y+=ds*scale*sinf(o->yaw+half);
      o->travel+=ds;o->yaw_total+=angle;o->yaw=wrap(o->yaw+angle);
    } else if(!continuous) { o->valid=false;o->discontinuities++; }
  } else { o->valid=true;o->initialized=true; }
  o->previous_v=v;o->previous_w=w;o->v=v;o->w=w;
  o->sample_sequence=s->sequence;o->measurement_tick_ms=s->tick_ms;
  o->left_tenths_rpm=s->left_tenths_rpm;o->right_tenths_rpm=s->right_tenths_rpm;
  o->position_left=s->position_left;o->position_right=s->position_right;
  if(!o->valid && mq_is_executing(q))mq_fault(q,MQ_ERR_ODOM,s->tick_ms);
  if(q->state==MQ_BARRIER && q->barrier_applied && s->sequence!=q->barrier_sample &&
      after(s->tick_ms,q->target_requested_ms)) {
    q->barrier_sample=s->sequence;
    q->zero_count=zero_sample(o)?q->zero_count+1u:0;
  }
}
static uint32_t crc_byte(uint32_t c,uint8_t b) {
  c^=b;for(unsigned j=0;j<8;j++)c=(c>>1)^((0u-(c&1u))&0xedb88320u);return c;
}
uint32_t mq_segments_crc(const MQSegment *segments,uint16_t count) {
  uint32_t crc=0xffffffffu;
  for(uint16_t i=0;i<count;i++) {
    uint32_t words[2];words[0]=segments[i].kind;memcpy(&words[1],&segments[i].value,4);
    for(unsigned j=0;j<2;j++)for(unsigned k=0;k<4;k++)crc=crc_byte(crc,(uint8_t)(words[j]>>(k*8)));
  }
  return ~crc;
}
static bool segment_valid(MQSegment s) {
  return isfinite(s.value) && (s.kind==MQ_KIND_MOVE ? s.value>0.f && s.value<=1000.f :
    s.kind==MQ_KIND_TURN && absf(s.value)>0.f && absf(s.value)<=2.f*MQ_PI);
}
MQResult mq_begin(MQ *q,uint64_t task,uint32_t epoch,uint16_t count,
                  uint32_t crc,uint32_t duration_ms) {
  if(!task || !count || count>MQ_CAPACITY || !duration_ms || duration_ms>MQ_MAX_DURATION_MS)return MQ_INVALID;
  if(epoch!=q->odom.epoch)return MQ_IDENTITY;
  if(task<q->task_id)return MQ_ORDER; /* Monotonic boot-local replay watermark. */
  if(task==q->task_id) {
    return q->count==count && q->expected_crc==crc && q->duration_ms==duration_ms ? MQ_OK:MQ_ORDER;
  }
  if(mq_is_executing(q))return MQ_BUSY;
  MQDriveView d=view(q);
  if(d.state!=MQ_DRIVE_STOPPED || !d.stop_verified || !d.guard_valid || d.error || !q->odom.valid || q->critical_latched)return MQ_UNHEALTHY;
  q->task_id=task;q->count=count;q->expected_crc=crc;q->duration_ms=duration_ms;
  q->received=q->index=0;q->epoch_at_upload=epoch;q->state=MQ_LOADING;
  q->committed=q->has_started=q->arrival_reached=q->physical_stop_confirmed=false;
  q->first_error=q->first_drive_error=q->first_error_tick_ms=0;
  q->target_pending=false;q->zero_count=0;q->progress=q->yaw_deviation=q->lateral_deviation=0.f;
  return MQ_OK;
}
MQResult mq_append(MQ *q,uint64_t task,uint16_t index,MQSegment s) {
  if(task!=q->task_id)return MQ_IDENTITY;
  if(!segment_valid(s) || index>=q->count)return MQ_INVALID;
  if(index<q->received) {
    return q->segments[index].kind==s.kind && q->segments[index].value==s.value?MQ_OK:MQ_ORDER;
  }
  if(q->state!=MQ_LOADING)return MQ_BUSY;
  if(index!=q->received)return MQ_ORDER;
  q->segments[q->received++]=s;return MQ_OK;
}
MQResult mq_commit(MQ *q,uint64_t task) {
  if(task!=q->task_id || q->epoch_at_upload!=q->odom.epoch)return MQ_IDENTITY;
  if(q->committed)return MQ_OK;
  if(q->state!=MQ_LOADING || q->received!=q->count)return MQ_ORDER;
  if(mq_segments_crc(q->segments,q->count)!=q->expected_crc)return MQ_CRC;
  q->committed=true;q->state=MQ_READY;return MQ_OK;
}
MQResult mq_start(MQ *q,uint64_t task,uint32_t now) {
  if(task!=q->task_id || q->epoch_at_upload!=q->odom.epoch)return MQ_IDENTITY;
  /* Critical boot/UART faults also invalidate an old successful START ACK. */
  if(q->critical_latched)return MQ_UNHEALTHY;
  if(q->has_started)return MQ_OK; /* A repeated START never runs a finished task again. */
  if(q->state!=MQ_READY || !q->committed)return MQ_ORDER;
  MQDriveView d=view(q);
  if(d.state!=MQ_DRIVE_STOPPED || !d.stop_verified || !d.guard_valid || d.error || q->first_error || !q->boot_id)return MQ_UNHEALTHY;
  if(!q->odom.valid || !q->odom.initialized || now-q->odom.measurement_tick_ms>=MQ_SAMPLE_TIMEOUT_MS || !zero_sample(&q->odom))return MQ_STALE;
  q->has_started=true;q->started_ms=now;q->index=0;q->state=MQ_ARMING;
  q->drive.arm(q->drive_user,now);return MQ_OK;
}
void mq_stop(MQ *q,uint32_t now) {
  if(q->has_started && q->state!=MQ_COMPLETE && q->state!=MQ_FAILED)mq_fault(q,MQ_ERR_CANCEL,now);
  else {
    q->state=MQ_STOPPING;q->physical_stop_confirmed=false;
    q->drive.stop(q->drive_user,now);
  }
  q->committed=false; /* STOP invalidates every queued but unexecuted action. */
}
MQResult mq_reset_odom(MQ *q,uint32_t now) {
  if(q->critical_latched)return MQ_UNHEALTHY;
  MQDriveView d=view(q);
  if(mq_is_executing(q) || d.state!=MQ_DRIVE_STOPPED || !d.stop_verified || !d.guard_valid || d.error)return MQ_BUSY;
  if(!q->odom.initialized || now-q->odom.measurement_tick_ms>=MQ_SAMPLE_TIMEOUT_MS || !zero_sample(&q->odom))return MQ_STALE;
  MQOdom old=q->odom;memset(&q->odom,0,sizeof(q->odom));
  q->odom.epoch=old.epoch+1u;if(!q->odom.epoch)q->odom.epoch=1;
  q->odom.sample_sequence=old.sample_sequence;q->odom.measurement_tick_ms=old.measurement_tick_ms;
  q->odom.left_tenths_rpm=old.left_tenths_rpm;q->odom.right_tenths_rpm=old.right_tenths_rpm;
  q->odom.position_left=old.position_left;q->odom.position_right=old.position_right;
  q->odom.previous_v=old.v;q->odom.previous_w=old.w;
  q->odom.initialized=q->odom.valid=true;q->state=MQ_IDLE;
  q->committed=q->has_started=q->arrival_reached=false;
  q->count=q->received=q->index=0;q->first_error=q->first_drive_error=0;
  return MQ_OK;
}
static void begin_segment(MQ *q) {
  q->segment_travel=q->odom.travel;q->segment_yaw=q->odom.yaw_total;
  q->segment_x=q->odom.x;q->segment_y=q->odom.y;
  q->progress=q->yaw_deviation=q->lateral_deviation=0.f;
  q->desired_left=q->desired_right=0;q->target_generation=0;q->target_pending=false;
  q->state=q->segments[q->index].kind==MQ_KIND_MOVE?MQ_MOVE:MQ_TURN;
}
static void request_target(MQ *q,int16_t l,int16_t r,uint32_t now) {
  if(q->target_pending)return;
  if(q->target_generation && l==q->desired_left && r==q->desired_right && now-q->last_target_ms<100u)return;
  uint32_t generation=q->drive.target(q->drive_user,l,r,now);
  if(!generation){mq_fault(q,MQ_ERR_DRIVE,now);return;}
  q->desired_left=l;q->desired_right=r;q->target_generation=generation;
  q->target_requested_ms=q->last_target_ms=now;q->target_pending=true;
}
void mq_tick(MQ *q,uint32_t now) {
  MQDriveView d=view(q);
  if(d.error && !q->first_drive_error)q->first_drive_error=d.error;
  if(q->state==MQ_STOPPING) {
    if((d.error || d.state==MQ_DRIVE_FAILED) && !q->first_error) {
      q->first_error=MQ_ERR_DRIVE;q->first_error_tick_ms=now;
    }
    if(d.state==MQ_DRIVE_STOPPED || d.state==MQ_DRIVE_FAILED) {
      q->physical_stop_confirmed=d.stop_verified;
      if(!d.stop_verified && !q->first_error){q->first_error=MQ_ERR_DRIVE;q->first_error_tick_ms=now;}
      if(q->first_error || !d.stop_verified)q->state=MQ_FAILED;
      else q->state=q->arrival_reached?MQ_COMPLETE:q->has_started?MQ_FAILED:MQ_IDLE;
    }
    return;
  }
  if(!mq_is_executing(q))return;
  if(d.error || d.state==MQ_DRIVE_FAILED){mq_fault(q,MQ_ERR_DRIVE,now);return;}
  if(!q->odom.valid || now-q->odom.measurement_tick_ms>=MQ_SAMPLE_TIMEOUT_MS){mq_fault(q,MQ_ERR_ODOM,now);return;}
  if(now-q->started_ms>=q->duration_ms){mq_fault(q,MQ_ERR_DEADLINE,now);return;}
  if(q->target_pending) {
    if(d.applied_generation==q->target_generation) {
      if(now-q->target_requested_ms>=300u){mq_fault(q,MQ_ERR_APPLY,now);return;}
      q->target_pending=false;
      if(q->state==MQ_BARRIER && q->desired_left==0 && q->desired_right==0) {
        q->barrier_applied=true;q->target_requested_ms=d.applied_tick_ms;
        q->barrier_sample=q->odom.sample_sequence;q->zero_count=0;
      }
    } else if(now-q->target_requested_ms>=300u){mq_fault(q,MQ_ERR_APPLY,now);return;}
  }
  if(q->state==MQ_ARMING) {
    if(d.state==MQ_DRIVE_ENABLED){begin_segment(q);}
    else if(now-q->started_ms>=3000u)mq_fault(q,MQ_ERR_DRIVE,now);
    return;
  }
  if(d.state!=MQ_DRIVE_ENABLED || !d.guard_valid){mq_fault(q,MQ_ERR_DRIVE,now);return;}
  if(q->state==MQ_BARRIER) {
    if(now-q->barrier_started_ms>=2000u){mq_fault(q,MQ_ERR_HANDOFF,now);return;}
    if(q->barrier_applied && q->zero_count>=5u){q->index++;begin_segment(q);return;}
    if(!q->barrier_applied)request_target(q,0,0,now);
    return;
  }
  MQSegment s=q->segments[q->index];
  q->yaw_deviation=q->odom.yaw_total-q->segment_yaw;
  q->lateral_deviation=-(q->odom.x-q->segment_x)*sinf(q->segment_yaw)+(q->odom.y-q->segment_y)*cosf(q->segment_yaw);
  q->progress=s.kind==MQ_KIND_MOVE?q->odom.travel-q->segment_travel:
    (s.value>0.f?1.f:-1.f)*(q->odom.yaw_total-q->segment_yaw);
  float tolerance=s.kind==MQ_KIND_MOVE?(q->index+1u==q->count?.10f:.02f):MQ_PI/60.f;
  float remaining=absf(s.value)-q->progress;
  if(remaining<=tolerance) {
    if(q->index+1u==q->count) {
      q->arrival_reached=true;q->state=MQ_STOPPING;q->physical_stop_confirmed=false;
      q->target_pending=false;q->drive.stop(q->drive_user,now);return;
    }
    q->state=MQ_BARRIER;q->barrier_started_ms=now;q->barrier_applied=false;
    q->zero_count=0;
    /* The last nonzero command can still be in flight. Submit zero after its
       original acknowledgement, never relabel it as a zero command. */
    if(!q->target_pending){q->target_generation=0;request_target(q,0,0,now);}
    return;
  }
  if(s.kind==MQ_KIND_MOVE) {
    int16_t rpm=remaining<=.30f?MQ_MOVE_NEAR_RPM:MQ_MOVE_CRUISE_RPM;
    request_target(q,rpm,(int16_t)-rpm,now);
  } else {
    int16_t rpm=s.value>0.f?-MQ_TURN_RPM:MQ_TURN_RPM;
    request_target(q,rpm,rpm,now);
  }
}

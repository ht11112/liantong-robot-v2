/* Independent execution-boundary probes. Links the unmodified production mq.c. */
#include "mq.h"
#include "drive.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  MQDriveView v;
  uint32_t generation,requested_generation,requested_at;
  int16_t requested_left,requested_right;
  unsigned arms,targets,stops;
} Motor;
typedef struct {MQ q;Motor motor;uint32_t now,seq;} Fixture;
static void arm(void *p,uint32_t now){Motor *m=p;(void)now;m->arms++;m->v.state=MQ_DRIVE_ARMING;}
static uint32_t target(void *p,int16_t l,int16_t r,uint32_t now){
  Motor *m=p;m->targets++;m->requested_generation=++m->generation;
  m->requested_left=l;m->requested_right=r;m->requested_at=now;return m->requested_generation;
}
static void stop(void *p,uint32_t now){Motor *m=p;(void)now;m->stops++;m->v.state=MQ_DRIVE_STOPPING;m->v.stop_verified=false;}
static MQDriveView view(void *p){return ((Motor *)p)->v;}
static const MQDriveOps ops={arm,target,stop,view};
static void sample(Fixture *f,uint32_t tick,int16_t l,int16_t r){
  MQSample s={0};f->now=tick;s.sequence=++f->seq;s.tick_ms=tick;
  s.left_tenths_rpm=l;s.right_tenths_rpm=r;s.valid=true;mq_sample(&f->q,&s);
}
static void setup(Fixture *f,uint32_t now){
  memset(f,0,sizeof(*f));f->now=now;
  f->motor.v.state=MQ_DRIVE_STOPPED;f->motor.v.stop_verified=true;f->motor.v.guard_valid=true;
  mq_init(&f->q,UINT64_C(0x1234567890abcdef),&ops,&f->motor);sample(f,now,0,0);
}
static void ack(Fixture *f,uint32_t tick){
  f->motor.v.applied_generation=f->motor.requested_generation;
  f->motor.v.applied_left=f->motor.requested_left;f->motor.v.applied_right=f->motor.requested_right;
  f->motor.v.applied_tick_ms=tick;
}
static MQResult load(Fixture *f,uint64_t task,const MQSegment *s,uint16_t count){
  MQResult r=mq_begin(&f->q,task,f->q.odom.epoch,count,mq_segments_crc(s,count),MQ_LEASE_MS);
  if(r!=MQ_OK)return r;
  for(uint16_t i=0;i<count;i++){r=mq_append(&f->q,task,i,s[i]);if(r!=MQ_OK)return r;}
  return mq_commit(&f->q,task);
}
static void start(Fixture *f){
  assert(mq_start(&f->q,f->q.task_id,f->now)==MQ_OK);
  f->motor.v.state=MQ_DRIVE_ENABLED;f->motor.v.stop_verified=false;
  mq_tick(&f->q,f->now);mq_tick(&f->q,f->now);
}
static void step(Fixture *f,int16_t l,int16_t r){
  sample(f,f->now+50u,l,r);mq_tick(&f->q,f->now);
  if(f->q.target_pending){ack(f,f->now);mq_tick(&f->q,f->now);}
}
static void step_renewing(Fixture *f,int16_t l,int16_t r){
  step(f,l,r);
  if(mq_lease_remaining_ms(&f->q,f->now)==5000u)
    assert(mq_renew(&f->q,f->q.task_id,f->now)==MQ_OK);
}
static void finish_stop(Fixture *f){
  f->motor.v.state=MQ_DRIVE_STOPPED;f->motor.v.stop_verified=true;
  f->motor.v.applied_left=f->motor.v.applied_right=0;mq_tick(&f->q,f->now);
}
static void expect(const char *name){printf("REVIEW PASS %s\n",name);}

static void offline_and_tick_wrap(void){
  Fixture f;setup(&f,UINT32_MAX-1000u);MQSegment s={MQ_KIND_MOVE,.50f};
  assert(load(&f,1,&s,1)==MQ_OK);start(&f);ack(&f,f.now);mq_tick(&f.q,f.now);
  for(unsigned i=0;i<30;i++)step(&f,20,-20);
  assert(f.now<1000u && f.q.state==MQ_MOVE && f.motor.arms==1 && !f.motor.stops);
  assert(f.q.odom.valid && f.q.odom.travel>.02f && !f.q.first_error);
  /* No host API or heartbeat was called for 1500 ms, across the uint32 wrap. */
  expect("execution inside the initial lease crosses tick wrap");
}
static void deadline_and_late_ack(void){
  Fixture f;setup(&f,100u);MQSegment s={MQ_KIND_MOVE,10.f};
  assert(load(&f,1,&s,1)==MQ_OK);start(&f);
  for(unsigned i=0;i<5;i++){sample(&f,f.now+50u,0,0);mq_tick(&f.q,f.now);}
  sample(&f,f.now+50u,0,0);ack(&f,f.now);mq_tick(&f.q,f.now);
  assert(f.q.first_error==MQ_ERR_APPLY && f.q.state==MQ_STOPPING && !f.q.arrival_reached);
  setup(&f,UINT32_MAX-1000u);assert(load(&f,1,&s,1)==MQ_OK);start(&f);
  ack(&f,f.now);mq_tick(&f.q,f.now);
  /* Keep the measurement fresh without pretending to integrate the elapsed gap. */
  f.q.odom.measurement_tick_ms=f.q.started_ms+MQ_LEASE_MS;
  mq_tick(&f.q,f.q.odom.measurement_tick_ms);
  assert(f.q.first_error==MQ_ERR_DEADLINE && f.motor.stops==1);
  setup(&f,100u);assert(load(&f,1,&s,1)==MQ_OK);
  assert(mq_start(&f.q,1,f.now)==MQ_OK);
  f.now=3099u;f.q.odom.measurement_tick_ms=f.now;
  assert(mq_renew(&f.q,1,f.now)==MQ_OK);
  mq_tick(&f.q,3100u);
  assert(f.q.first_error==MQ_ERR_DRIVE && f.motor.stops==1);
  expect("late 300ms evidence, lease expiry and arming deadline remain hard bounds");
}
static void lease_renewal_and_expiration(void){
  Fixture f;setup(&f,UINT32_MAX-1000u);MQSegment s={MQ_KIND_MOVE,10.f};
  uint32_t crc=mq_segments_crc(&s,1);
  assert(mq_begin(&f.q,1,1,1,crc,1200000u)==MQ_INVALID);
  assert(load(&f,1,&s,1)==MQ_OK);
  assert(mq_renew(&f.q,1,f.now)==MQ_ORDER && mq_lease_remaining_ms(&f.q,f.now)==0);
  start(&f);ack(&f,f.now);mq_tick(&f.q,f.now);
  assert(mq_lease_remaining_ms(&f.q,f.now)==MQ_LEASE_MS);
  assert(mq_renew(&f.q,2,f.now)==MQ_IDENTITY);
  for(unsigned i=0;i<25000;i++)step_renewing(&f,0,0);
  assert(f.now-f.q.started_ms==1250000u && f.q.state==MQ_MOVE && f.motor.arms==1 && !f.motor.stops);
  for(unsigned i=0;i<599;i++)step(&f,0,0);
  assert(mq_lease_remaining_ms(&f.q,f.now)==50u);
  sample(&f,f.now+50u,0,0);
  assert(mq_renew(&f.q,1,f.now)==MQ_EXPIRED);
  assert(f.q.first_error==MQ_ERR_DEADLINE && f.q.state==MQ_STOPPING && f.motor.stops==1);
  assert(mq_renew(&f.q,1,f.now)==MQ_ORDER && mq_lease_remaining_ms(&f.q,f.now)==0);
  finish_stop(&f);
  assert(f.q.state==MQ_FAILED && mq_renew(&f.q,1,f.now)==MQ_ORDER && f.motor.arms==1);
  expect("renewals carry one task past 20 minutes across tick wrap; late renewal cannot revive it");
}
static void malformed_or_partial_queue(void){
  Fixture f;setup(&f,100u);MQSegment s[2]={{MQ_KIND_MOVE,.20f},{MQ_KIND_TURN,1.f}};
  assert(mq_begin(&f.q,1,f.q.odom.epoch,2,mq_segments_crc(s,2),MQ_LEASE_MS)==MQ_OK);
  assert(mq_append(&f.q,1,0,s[0])==MQ_OK && mq_commit(&f.q,1)==MQ_ORDER);
  assert(mq_start(&f.q,1,100u)!=MQ_OK && !f.motor.arms);
  assert(mq_append(&f.q,1,1,s[1])==MQ_OK);
  f.q.segments[1].value=2.f;assert(mq_commit(&f.q,1)==MQ_CRC);
  assert(mq_start(&f.q,1,100u)!=MQ_OK && !f.motor.arms && !f.motor.targets);
  expect("partial and CRC-mismatched queues never grant motion");
}
static void idempotence_and_old_task(void){
  Fixture f;setup(&f,100u);MQSegment s={MQ_KIND_MOVE,.101f};
  assert(load(&f,10,&s,1)==MQ_OK);start(&f);
  assert(mq_start(&f.q,10,f.now)==MQ_OK && f.motor.arms==1);
  for(unsigned i=0;i<4 && f.q.state!=MQ_STOPPING;i++)step(&f,10,-10);
  assert(f.q.arrival_reached);finish_stop(&f);
  assert(mq_start(&f.q,10,f.now)==MQ_OK && f.motor.arms==1);
  sample(&f,f.now+50u,0,0);assert(load(&f,11,&s,1)==MQ_OK);
  assert(mq_begin(&f.q,10,f.q.odom.epoch,1,mq_segments_crc(&s,1),MQ_LEASE_MS)!=MQ_OK);
  assert(mq_reset_odom(&f.q,f.now)==MQ_OK);
  assert(mq_begin(&f.q,10,f.q.odom.epoch,1,mq_segments_crc(&s,1),MQ_LEASE_MS)!=MQ_OK);
  assert(!f.q.committed && f.motor.arms==1);
  expect("same START idempotence and old task rejection survive new task and reset");
}
static void duplicate_and_missing_samples(void){
  Fixture f;setup(&f,100u);MQSample duplicate={0};duplicate.sequence=f.seq;
  duplicate.tick_ms=500u;duplicate.left_tenths_rpm=100;duplicate.right_tenths_rpm=-100;duplicate.valid=true;
  mq_sample(&f.q,&duplicate);assert(f.q.odom.measurement_tick_ms==100u && f.q.odom.travel==0.f);
  MQSegment s={MQ_KIND_MOVE,1.f};assert(load(&f,1,&s,1)==MQ_OK);start(&f);
  sample(&f,150u,10,-10);ack(&f,f.now);mq_tick(&f.q,f.now);
  float travel=f.q.odom.travel;f.seq++;sample(&f,200u,10,-10);
  assert(!f.q.odom.valid && f.q.odom.travel==travel && f.q.first_error==MQ_ERR_ODOM);
  sample(&f,250u,0,0);assert(!f.q.odom.valid && f.q.odom.travel==travel);
  setup(&f,100u);sample(&f,351u,10,-10);
  assert(!f.q.odom.valid && f.q.odom.travel==0.f);
  expect("duplicate does not renew source; missing MCU sample is permanently discontinuous");
}
static void move_yaw_is_observation_only(void){
  Fixture f;setup(&f,100u);MQSegment s={MQ_KIND_MOVE,.5f};
  assert(load(&f,1,&s,1)==MQ_OK);start(&f);ack(&f,f.now);mq_tick(&f.q,f.now);
  for(unsigned i=0;i<1200 && f.q.state!=MQ_STOPPING;i++)step_renewing(&f,20,-10);
  assert(f.q.arrival_reached && !f.q.first_error);
  assert(fabsf(f.q.yaw_deviation)>8.f*MQ_PI/180.f && fabsf(f.q.lateral_deviation)>.02f);
  assert(f.motor.requested_left==-f.motor.requested_right && f.motor.requested_left>0);
  expect("MOVE completes by measured centre travel despite yaw over eight degrees");
}
static void negative_turn_wrap(void){
  Fixture f;setup(&f,100u);MQSegment s={MQ_KIND_TURN,-1.5f*MQ_PI};
  assert(load(&f,1,&s,1)==MQ_OK);start(&f);ack(&f,f.now);mq_tick(&f.q,f.now);
  for(unsigned i=0;i<1500 && f.q.state!=MQ_STOPPING;i++)step_renewing(&f,10,10);
  assert(f.q.arrival_reached && !f.q.first_error && f.q.odom.yaw_total < -MQ_PI);
  assert(f.q.odom.yaw>0.f && f.q.progress>MQ_PI && f.motor.requested_left==f.motor.requested_right);
  expect("negative TURN uses unwrapped angle across pi boundary");
}
static void direct_stop_and_parking_sample(void){
  Fixture f;setup(&f,100u);MQSegment s={MQ_KIND_MOVE,.101f};
  assert(load(&f,1,&s,1)==MQ_OK);start(&f);
  /* Deliberately do not acknowledge the first nonzero target. */
  for(unsigned i=0;i<4 && f.q.state!=MQ_STOPPING;i++){
    sample(&f,f.now+50u,10,-10);mq_tick(&f.q,f.now);
  }
  assert(f.q.arrival_reached && f.q.state==MQ_STOPPING && f.motor.stops==1 && f.motor.targets==1);
  float travel=f.q.odom.travel;sample(&f,f.now+50u,10,-10);
  assert(f.q.odom.travel>travel && f.q.odom.valid);
  finish_stop(&f);assert(f.q.state==MQ_COMPLETE && f.q.physical_stop_confirmed);
  expect("arrival STOP preempts old nonzero ACK; parking feedback still integrates");
}
static void parking_fault_cannot_upgrade(void){
  Fixture f;setup(&f,100u);MQSegment s={MQ_KIND_MOVE,.101f};
  assert(load(&f,1,&s,1)==MQ_OK);start(&f);
  for(unsigned i=0;i<4 && f.q.state!=MQ_STOPPING;i++)step(&f,10,-10);
  assert(f.q.arrival_reached);
  f.motor.v.state=MQ_DRIVE_FAILED;f.motor.v.error=81u;f.motor.v.stop_verified=true;mq_tick(&f.q,f.now);
  assert(f.q.state==MQ_FAILED && f.q.arrival_reached && f.q.physical_stop_confirmed);
  assert(f.q.first_error==MQ_ERR_DRIVE && f.q.first_drive_error==81u);
  expect("new parking driver fault cannot convert an arrival into session success");
}
static void handoff_requires_new_zero_application(void){
  Fixture f;setup(&f,100u);MQSegment s[2]={{MQ_KIND_MOVE,.021f},{MQ_KIND_TURN,1.f}};
  assert(load(&f,1,s,2)==MQ_OK);start(&f);
  for(unsigned i=0;i<4 && f.q.state!=MQ_BARRIER;i++){
    sample(&f,f.now+50u,10,-10);mq_tick(&f.q,f.now);
  }
  assert(f.q.state==MQ_BARRIER && f.q.target_pending && !f.q.barrier_applied);
  assert(f.motor.targets==1 && f.motor.requested_left>0);
  ack(&f,f.now);mq_tick(&f.q,f.now); /* This ACK belongs to the old NONZERO target. */
  assert(!f.q.barrier_applied && f.q.target_pending && f.motor.targets==2);
  assert(f.motor.requested_left==0 && f.motor.requested_right==0);
  f.now+=5u;ack(&f,f.now);mq_tick(&f.q,f.now);assert(f.q.barrier_applied);
  sample(&f,f.now,0,0);mq_tick(&f.q,f.now);assert(f.q.zero_count==0); /* Equal tick is not post-application. */
  MQSample duplicate={0};duplicate.sequence=f.seq;duplicate.tick_ms=f.now+50u;duplicate.valid=true;
  mq_sample(&f.q,&duplicate);assert(f.q.zero_count==0);
  sample(&f,f.now+50u,0,0);mq_tick(&f.q,f.now);
  sample(&f,f.now+50u,0,0);mq_tick(&f.q,f.now);assert(f.q.zero_count==2);
  sample(&f,f.now+50u,3,0);mq_tick(&f.q,f.now);assert(f.q.zero_count==0);
  for(unsigned i=0;i<4;i++){sample(&f,f.now+50u,0,0);mq_tick(&f.q,f.now);}
  assert(f.q.state==MQ_BARRIER && f.q.zero_count==4 && f.q.index==0 && f.motor.targets==2);
  sample(&f,f.now+50u,0,0);mq_tick(&f.q,f.now);
  assert(f.q.state==MQ_TURN && f.q.index==1 && !f.q.first_error);
  expect("handoff distinguishes old nonzero ACK and requires five distinct post-zero sources");
}
static bool fifo_never_ready(void *p){(void)p;return false;}
static bool impossible_send(void *p,const uint8_t data[8]){(void)p;(void)data;assert(!"TX must not bypass unavailable FIFO");return false;}
static uint32_t bus_without_error(void *p){(void)p;return 0;}
static void unavailable_can_fifo_is_bounded(void){
  Drive d;const DriveBusOps bus={impossible_send,fifo_never_ready,bus_without_error};
  drive_init(&d,&bus,NULL,100u);
  for(uint32_t now=100u;now<5100u;now++)drive_poll(&d,now);
  assert(d.view.state==MQ_DRIVE_FAILED && d.tx_suppressed && !d.view.stop_verified && d.view.error);
  expect("permanently unavailable CAN FIFO cannot leave STOP pending forever");
}
static void dock_execution(void){
  Fixture f;MQSegment s={MQ_KIND_DOCK_MOVE,-.05f};setup(&f,100u);
  assert(load(&f,1,&s,1)==MQ_OK);start(&f);
  assert(f.q.state==MQ_MOVE && f.motor.requested_left==-1 && f.motor.requested_right==1);
  ack(&f,f.now);mq_tick(&f.q,f.now);
  for(unsigned i=0;i<20;i++)step(&f,10,-10);
  assert(f.q.progress<0.f && !f.q.arrival_reached);
  for(unsigned i=0;i<1200 && f.q.state==MQ_MOVE;i++)step_renewing(&f,-10,10);
  assert(f.q.state==MQ_STOPPING && f.q.progress>=.045f && f.q.progress<.046f);
  assert(f.q.arrival_reached && !f.q.physical_stop_confirmed);finish_stop(&f);
  assert(f.q.state==MQ_COMPLETE && f.q.physical_stop_confirmed);
  for(int sign=-1;sign<=1;sign+=2){
    setup(&f,100u);s=(MQSegment){MQ_KIND_DOCK_TURN,sign*MQ_PI/36.f};
    assert(load(&f,1,&s,1)==MQ_OK);start(&f);
    assert(f.q.state==MQ_TURN && f.motor.requested_left==-sign && f.motor.requested_right==-sign);
    ack(&f,f.now);mq_tick(&f.q,f.now);
    for(unsigned i=0;i<1200 && f.q.state==MQ_TURN;i++)step_renewing(&f,-sign*10,-sign*10);
    assert(f.q.state==MQ_STOPPING && f.q.progress>=4.f*MQ_PI/180.f && f.q.progress<4.1f*MQ_PI/180.f);
    finish_stop(&f);assert(f.q.state==MQ_COMPLETE && f.q.physical_stop_confirmed);
  }
  setup(&f,100u);s=(MQSegment){MQ_KIND_DOCK_MOVE,-.05f};assert(load(&f,1,&s,1)==MQ_OK);start(&f);
  mq_stop(&f.q,f.now);finish_stop(&f);
  assert(f.q.state==MQ_FAILED && f.q.first_error==MQ_ERR_CANCEL && !f.q.arrival_reached);
  expect("dock reverse and both turns use signed measurements, fine thresholds and real STOP path");
}
static void dock_upload_bounds(void){
  Fixture f;MQSegment bad[]={{MQ_KIND_DOCK_MOVE,0.f},{MQ_KIND_DOCK_MOVE,-.005f},{MQ_KIND_DOCK_MOVE,.01f},
    {MQ_KIND_DOCK_MOVE,-.051f},{MQ_KIND_DOCK_TURN,0.f},{MQ_KIND_DOCK_TURN,MQ_PI/36.f+.001f},
    {MQ_KIND_DOCK_TURN,-MQ_PI/36.f-.001f},{MQ_KIND_DOCK_TURN,MQ_PI/180.f},
    {MQ_KIND_DOCK_TURN,-MQ_PI/180.f}};
  for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++){
    setup(&f,100u);assert(load(&f,1,&bad[i],1)==MQ_INVALID);
    assert(mq_start(&f.q,1,f.now)==MQ_ORDER && !f.motor.arms);
  }
  MQSegment pair[]={{MQ_KIND_MOVE,.2f},{MQ_KIND_DOCK_MOVE,-.05f}};
  setup(&f,100u);assert(load(&f,1,pair,2)==MQ_INVALID);
  assert(mq_start(&f.q,1,f.now)==MQ_ORDER && !f.motor.arms);
  expect("dock upload rejects invalid ranges and mixed or multiple segments before START");
}
int main(void){
  offline_and_tick_wrap();deadline_and_late_ack();lease_renewal_and_expiration();malformed_or_partial_queue();
  idempotence_and_old_task();duplicate_and_missing_samples();move_yaw_is_observation_only();
  negative_turn_wrap();direct_stop_and_parking_sample();parking_fault_cannot_upgrade();
  handoff_requires_new_zero_application();
  unavailable_can_fifo_is_bounded();
  dock_upload_bounds();dock_execution();
  puts("REVIEW COMPLETE 12 production-core/driver scenarios");return 0;
}

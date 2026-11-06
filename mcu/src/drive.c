/* One nonblocking CANopen SDO owner. No host lease, heap or test entry points. */
#include "drive.h"
#include <string.h>

enum { FLOW_STOP=1,FLOW_ARM=2 };
enum { OWNER_FLOW=1,OWNER_SPEED,OWNER_AUX,OWNER_GUARD,OWNER_TARGET,OWNER_REFRESH };
enum { READ=0,WRITE=1 };
enum { C_LEFT=1,C_RIGHT=2,C_SPEED=4,C_STATUS=8,C_FAULT=16,C_CURRENT=32 };

typedef struct { uint16_t index;uint8_t sub,bytes,check;uint32_t value; } ArmStep;
enum { EQ=0,BUS_RANGE,SPEED_ZERO,STATUS_ZERO,HALL_RANGE,LE500,POSITIVE,LIMIT_WRITE,LIMIT_VERIFY,STATUS1,STATUS3,STATUS7 };
static const ArmStep arm_steps[]={
  {0x2035,0,2,BUS_RANGE,0},{0x603f,0,4,EQ,0},{0x606c,3,4,SPEED_ZERO,0},
  {0x6041,0,4,STATUS_ZERO,0},{0x6060,0,1,EQ,3},{0x6061,0,1,EQ,3},
  {0x200f,0,2,EQ,1},{0x2026,5,2,EQ,1},
  {0x200c,1,2,EQ,15},{0x200e,1,2,EQ,4096},{0x60ff,1,4,EQ,0},
  {0x2034,1,2,HALL_RANGE,0},{0x6083,1,4,LE500,0},{0x6084,1,4,LE500,0},
  {0x200c,2,2,EQ,15},{0x200e,2,2,EQ,4096},{0x60ff,2,4,EQ,0},
  {0x2034,2,2,HALL_RANGE,0},{0x6083,2,4,LE500,0},{0x6084,2,4,LE500,0},
  {0x2014,1,2,POSITIVE,0},{0x2014,2,2,POSITIVE,1},
  {0x2015,1,2,POSITIVE,2},{0x2015,2,2,POSITIVE,3},{0x2008,0,2,POSITIVE,4},
  {0x2014,1,2,LIMIT_WRITE,0},{0x2014,1,2,LIMIT_VERIFY,0},
  {0x2014,2,2,LIMIT_WRITE,1},{0x2014,2,2,LIMIT_VERIFY,1},
  {0x2015,1,2,LIMIT_WRITE,2},{0x2015,1,2,LIMIT_VERIFY,2},
  {0x2015,2,2,LIMIT_WRITE,3},{0x2015,2,2,LIMIT_VERIFY,3},
  {0x2008,0,2,LIMIT_VERIFY,4},
  {0x2000,0,2,EQ,500},
  {0x60ff,3,4,LIMIT_WRITE,5},{0x60ff,3,4,EQ,0},
  {0x6040,0,2,LIMIT_WRITE,6},{0x6041,0,4,STATUS1,0},
  {0x6040,0,2,LIMIT_WRITE,7},{0x6041,0,4,STATUS3,0},
  {0x6040,0,2,LIMIT_WRITE,8},{0x6041,0,4,STATUS7,0},
  {0x603f,0,4,EQ,0},{0x2000,0,2,EQ,500},
  {0x6064,1,4,LE500,0},{0x6064,2,4,LE500,0},
  {0x6077,3,4,LE500,0},{0x6041,0,4,STATUS7,0},{0x606c,3,4,SPEED_ZERO,0}
};

static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static uint32_t pack(int16_t l,int16_t r){return (uint16_t)l|((uint32_t)(uint16_t)r<<16);}
static uint32_t magnitude(int16_t v){return v<0?(uint32_t)-(int32_t)v:(uint32_t)v;}
static bool elapsed(uint32_t now,uint32_t then,uint32_t ms){return (uint32_t)(now-then)>=ms;}
static bool statuses(uint32_t v,uint32_t n){return (v&15u)==n&&((v>>16)&15u)==n;}

static void begin_stop(Drive *d,bool explicit_stop);
static void fault(Drive *d,uint32_t error){
  if(!d->view.error){d->view.error=error;d->first_error_tick=d->now;}
  if(d->flow!=FLOW_STOP&&!d->tx_suppressed&&d->view.state!=MQ_DRIVE_FAILED)begin_stop(d,false);
}

static bool start(Drive *d,uint16_t index,uint8_t sub,uint8_t bytes,bool write,uint32_t value,uint8_t owner,uint32_t generation){
  if(d->transaction.pending||d->transaction.done||d->tx_suppressed||!d->bus.tx_ready(d->bus_user))return false;
  uint8_t b[8]={0x40,(uint8_t)index,(uint8_t)(index>>8),sub,0,0,0,0};
  if(write){b[0]=(uint8_t)(0x23u|((4u-bytes)<<2));for(unsigned i=0;i<4;i++)b[4+i]=(uint8_t)(value>>(8*i));}
  if(owner==OWNER_FLOW)generation=((uint32_t)d->flow<<16)|d->flow_step;
  DriveTransaction t={index,sub,bytes,write?WRITE:READ,owner,value,d->now,generation,0,true,false,false};
  d->transaction=t;
  if(!d->bus.send(d->bus_user,b)){d->transaction.pending=false;d->transaction.done=true;return false;}
  return true;
}

void drive_receive_can(Drive *d,uint32_t id,uint32_t dlc,const uint8_t b[8],uint32_t now){
  d->last_rx_ms=now;
  DriveTransaction *t=&d->transaction;
  if(id!=0x581u||dlc!=8u)return;
  if(!t->pending){d->late_replies++;return;}
  if((uint16_t)(b[1]|((uint16_t)b[2]<<8))!=t->index||b[3]!=t->sub){d->late_replies++;return;}
  t->completed_ms=now;
  if(elapsed(now,t->started_ms,50u)){d->late_replies++;d->sdo_ambiguous=true;t->pending=false;t->done=true;t->ok=false;return;}
  if(b[0]==0x80u){t->pending=false;t->done=true;t->ok=false;return;}
  bool valid=t->kind==WRITE?b[0]==0x60u:((b[0]&0xe3u)==0x43u&&(4u-((b[0]>>2)&3u))==t->bytes);
  if(!valid){d->malformed_replies++;t->pending=false;t->done=true;t->ok=false;return;}
  if(t->kind==READ){t->value=u32(b+4);if(t->bytes<4)t->value&=(1u<<(8u*t->bytes))-1u;}
  t->pending=false;t->done=true;t->ok=true;
}

static void publish_speed(Drive *d,uint32_t value,uint32_t tick){
  d->speed_raw=value;d->speed_tick=tick;d->cache_mask|=C_SPEED;
  bool fresh=(d->cache_mask&(C_LEFT|C_RIGHT|C_STATUS|C_FAULT))==(C_LEFT|C_RIGHT|C_STATUS|C_FAULT)
    &&!elapsed(d->now,d->left_tick,250)&&!elapsed(d->now,d->right_tick,250)
    &&!elapsed(d->now,d->status_tick,250)&&!elapsed(d->now,d->fault_tick,250)&&!elapsed(d->now,tick,250)&&!d->fault_raw&&!d->sdo_ambiguous;
  if(d->sample_ready)d->sample_overruns++;
  d->sample=(MQSample){++d->sample_sequence,tick,(int16_t)value,(int16_t)(value>>16),d->position_left,d->position_right,fresh};
  d->sample_ready=true;
  if(magnitude((int16_t)value)>150u||magnitude((int16_t)(value>>16))>150u)fault(d,DRIVE_ERR_LIMIT);
  if(d->flow==FLOW_STOP&&d->flow_step==2){
    if(!elapsed(d->now,tick,250)&&magnitude((int16_t)value)<=2u&&magnitude((int16_t)(value>>16))<=2u)d->zero_count++;
    else d->zero_count=0;
    if(d->zero_count>=3)d->stationary_ok=true;
  }
}

static void cache(Drive *d,const DriveTransaction *t){
  if(t->kind!=READ)return;
  if(t->index==0x606c&&t->sub==3)publish_speed(d,t->value,t->completed_ms);
  else if(t->index==0x6064&&t->sub==1){d->position_left=t->value;d->left_tick=t->completed_ms;d->cache_mask|=C_LEFT;}
  else if(t->index==0x6064&&t->sub==2){d->position_right=t->value;d->right_tick=t->completed_ms;d->cache_mask|=C_RIGHT;}
  else if(t->index==0x6041){d->status_raw=t->value;d->status_tick=t->completed_ms;d->cache_mask|=C_STATUS;}
  else if(t->index==0x603f){d->fault_raw=t->value;d->fault_tick=t->completed_ms;d->cache_mask|=C_FAULT;}
  else if(t->index==0x6077){d->current_raw=t->value;d->current_tick=t->completed_ms;d->cache_mask|=C_CURRENT;}
}

static uint32_t arm_value(Drive *d,const ArmStep *s){
  if(s->value<5)return d->limit_values[s->value];
  return s->value==5?0u:s->value==6?6u:s->value==7?7u:15u;
}
static bool arm_check(Drive *d,const ArmStep *s,uint32_t v){
  switch(s->check){
  case EQ:return v==s->value;
  case BUS_RANGE:return v>=2000u&&v<=3250u;
  case SPEED_ZERO:return magnitude((int16_t)v)<=2u&&magnitude((int16_t)(v>>16))<=2u;
  case STATUS_ZERO:return statuses(v,0);
  case HALL_RANGE:return v>=1u&&v<=6u;
  case LE500:return s->index==0x6064||s->index==0x6077||v<=500u;
  case POSITIVE:
    if(!v)return false;
    /* The configured maximum speed is verified as read. Current limits retain their cap. */
    d->limit_values[s->value]=s->value<4&&v>30u?30u:v;
    return true;
  case LIMIT_WRITE:return true;
  case LIMIT_VERIFY:return v==d->limit_values[s->value];
  case STATUS1:return statuses(v,1);
  case STATUS3:return statuses(v,3);
  case STATUS7:return statuses(v,7);
  default:return false;
  }
}

static void finish_stop(Drive *d){
  d->view.stop_verified=!d->sdo_ambiguous&&d->zero_ok&&d->shutdown_ok&&d->stationary_ok&&d->disable_ok&&d->status_ok&&d->fault_ok;
  if(!d->view.stop_verified&&!d->view.error){d->view.error=DRIVE_ERR_STOP;d->first_error_tick=d->now;}
  d->tx_suppressed=!d->view.stop_verified;
  d->view.state=d->view.error||!d->view.guard_valid?MQ_DRIVE_FAILED:MQ_DRIVE_STOPPED;
  d->view.applied_left=d->view.applied_right=0;
  d->flow=0;d->boot_finished=true;
}

static void complete_flow(Drive *d,const DriveTransaction *t){
  if(d->flow==FLOW_ARM){
    if(!t->ok||!arm_check(d,&arm_steps[d->flow_step],t->value)){fault(d,DRIVE_ERR_CAN);return;}
    d->flow_step++;
    if(d->flow_step==sizeof(arm_steps)/sizeof(arm_steps[0])){
      d->flow=0;d->view.state=MQ_DRIVE_ENABLED;d->view.stop_verified=false;
      d->desired_left=d->desired_right=0;d->last_target_ms=d->now;
      d->view.applied_left=d->view.applied_right=0;d->view.applied_tick_ms=t->completed_ms;
    }
    return;
  }
  if(d->flow!=FLOW_STOP)return;
  switch(d->flow_step){
  case 0:d->zero_ok=t->ok&&!d->sdo_ambiguous;d->flow_step=1;break;
  case 1:d->shutdown_ok=t->ok&&!d->sdo_ambiguous;d->stop_wait_started_ms=t->completed_ms;d->zero_count=0;d->flow_step=2;break;
  case 3:d->disable_ok=t->ok;d->flow_step=4;break;
  case 4:d->status_ok=t->ok&&statuses(t->value,0);d->flow_step=5;break;
  case 5:d->fault_ok=t->ok&&!t->value;d->flow_step=6;break;
  case 6:
    d->view.stop_verified=!d->sdo_ambiguous&&d->zero_ok&&d->shutdown_ok&&d->stationary_ok&&d->disable_ok&&d->status_ok&&d->fault_ok;
    if(!d->view.stop_verified){d->flow_step=10;break;}
    if(!t->ok){d->view.guard_valid=false;fault(d,DRIVE_ERR_GUARD);finish_stop(d);}
    else if(t->value==500u){d->view.guard_valid=true;finish_stop(d);}
    else d->flow_step=7;
    break;
  case 7:if(!t->ok){d->view.guard_valid=false;fault(d,DRIVE_ERR_GUARD);finish_stop(d);}else d->flow_step=8;break;
  case 8:d->view.guard_valid=t->ok&&t->value==500u;if(!d->view.guard_valid)fault(d,DRIVE_ERR_GUARD);finish_stop(d);break;
  case 10:finish_stop(d);break;
  default:break;
  }
}

static void completed(Drive *d){
  DriveTransaction t=d->transaction;d->transaction.done=false;
  if(t.ok)cache(d,&t);
  if(t.owner==OWNER_FLOW){
    if(t.generation==(((uint32_t)d->flow<<16)|d->flow_step))complete_flow(d,&t);
    return;
  }
  if(!t.ok){fault(d,DRIVE_ERR_CAN);return;}
  if(t.owner==OWNER_TARGET||t.owner==OWNER_REFRESH){
    if(d->flow==FLOW_STOP||d->view.state!=MQ_DRIVE_ENABLED)return;
    d->last_target_ms=t.completed_ms;
    d->view.applied_left=(int16_t)t.value;d->view.applied_right=(int16_t)(t.value>>16);
    if(t.owner==OWNER_TARGET){d->view.applied_generation=t.generation;d->view.applied_tick_ms=t.completed_ms;d->target_pending=false;}
  }else if(t.owner==OWNER_GUARD){
    d->guard_known=true;d->view.guard_valid=t.value==500u;if(!d->view.guard_valid)fault(d,DRIVE_ERR_GUARD);
  }
}

static void begin_stop(Drive *d,bool explicit_stop){
  d->flow=FLOW_STOP;d->flow_step=0;d->flow_started_ms=d->now;
  d->view.state=MQ_DRIVE_STOPPING;d->view.stop_verified=false;
  d->desired_left=d->desired_right=0;d->target_pending=false;
  d->zero_ok=d->shutdown_ok=d->disable_ok=d->stationary_ok=d->status_ok=d->fault_ok=false;
  d->zero_count=0;d->stop_explicit=explicit_stop;
  if(explicit_stop)d->tx_suppressed=false;
  /* An in-flight transaction terminates on its own original <=50ms deadline.
   * No concurrent SDO or reused ACK authority is introduced by STOP. */
}

static void request_arm(void *user,uint32_t now){
  Drive *d=user;d->now=now;
  if(d->view.state!=MQ_DRIVE_STOPPED||d->view.error||!d->view.stop_verified||!d->view.guard_valid||d->uart_failed)return;
  d->flow=FLOW_ARM;d->flow_step=0;d->flow_started_ms=now;d->view.state=MQ_DRIVE_ARMING;
}
static uint32_t request_target(void *user,int16_t left,int16_t right,uint32_t now){
  Drive *d=user;d->now=now;
  if(d->view.state!=MQ_DRIVE_ENABLED||d->flow||d->view.error||d->target_pending)return 0;
  if(++d->generation==0)++d->generation;
  d->desired_generation=d->generation;d->desired_left=left;d->desired_right=right;d->target_pending=true;
  return d->generation;
}
static void request_stop(void *user,uint32_t now){Drive *d=user;d->now=now;if(d->flow!=FLOW_STOP)begin_stop(d,true);}
static MQDriveView get_view(void *user){return ((Drive *)user)->view;}
const MQDriveOps *drive_ops(void){static const MQDriveOps ops={request_arm,request_target,request_stop,get_view};return &ops;}

void drive_init(Drive *d,const DriveBusOps *ops,void *user,uint32_t now){
  memset(d,0,sizeof(*d));d->bus=*ops;d->bus_user=user;d->now=now;
  d->last_speed_start=now-50u;d->last_aux_start=now-1u;d->last_guard_start=now;
  d->view.state=MQ_DRIVE_STARTUP;begin_stop(d,true);
}
void drive_uart_fault(Drive *d,uint32_t now){
  if(d->uart_failed)return;
  d->now=now;d->uart_failed=true;fault(d,DRIVE_ERR_UART);
}
bool drive_pop_sample(Drive *d,MQSample *s){if(!d->sample_ready)return false;*s=d->sample;d->sample_ready=false;return true;}

static bool stop_step(Drive *d){
  switch(d->flow_step){
  case 0:return start(d,0x60ff,3,4,true,0,OWNER_FLOW,0);
  case 1:return start(d,0x6040,0,2,true,6,OWNER_FLOW,0);
  case 2:
    if(d->stationary_ok||elapsed(d->now,d->stop_wait_started_ms,600)){d->flow_step=3;return stop_step(d);}return false;
  case 3:return start(d,0x6040,0,2,true,0,OWNER_FLOW,0);
  case 4:return start(d,0x6041,0,4,false,0,OWNER_FLOW,0);
  case 5:return start(d,0x603f,0,4,false,0,OWNER_FLOW,0);
  case 6:return start(d,0x2000,0,2,false,0,OWNER_FLOW,0);
  case 7:return start(d,0x2000,0,2,true,500,OWNER_FLOW,0);
  case 8:return start(d,0x2000,0,2,false,0,OWNER_FLOW,0);
  case 10:return start(d,0x6040,0,2,true,2,OWNER_FLOW,0);
  default:return false;
  }
}

static bool auxiliary(Drive *d,uint32_t interval){
  static const uint16_t indexes[]={0x6064,0x6064,0x6041,0x603f,0x6077};
  static const uint8_t subs[]={1,2,0,0,3};
  if(!elapsed(d->now,d->last_aux_start,interval))return false;
  unsigned i=d->aux_index;
  if(!start(d,indexes[i],subs[i],4,false,0,OWNER_AUX,0))return false;
  d->last_aux_start=d->now;d->aux_index=(uint8_t)((i+1u)%5u);return true;
}

void drive_poll(Drive *d,uint32_t now){
  d->now=now;
  uint32_t errors=d->bus.bus_errors(d->bus_user);
  if(errors){if(!d->first_can_error)d->first_can_error=errors;fault(d,DRIVE_ERR_CAN);}
  if(d->transaction.pending&&elapsed(now,d->transaction.started_ms,50)){
    /* CANopen SDO has no transaction id. A reply after a timeout can match a
     * later request for the same object; no timed silence proves otherwise.
     * Keep physical STOP attempts, but never upgrade ACK/odom/stop evidence. */
    d->sdo_ambiguous=true;
    d->transaction.pending=false;d->transaction.done=true;d->transaction.ok=false;d->transaction.completed_ms=now;
  }
  if(d->transaction.done)completed(d);
  if(d->flow==FLOW_STOP&&elapsed(now,d->flow_started_ms,2000)){
    /* A full/unavailable CAN TX FIFO is also bounded; no pending SDO exists
     * in that case to supply a 50ms deadline. Leave the driver watchdog alone. */
    if(d->transaction.pending)d->sdo_ambiguous=true;
    d->transaction.pending=d->transaction.done=false;
    if(!d->view.error){d->view.error=DRIVE_ERR_STOP;d->first_error_tick=now;}
    d->view.stop_verified=false;d->view.state=MQ_DRIVE_FAILED;
    d->tx_suppressed=true;d->flow=0;return;
  }
  if(d->flow==FLOW_ARM&&elapsed(now,d->flow_started_ms,3000))fault(d,DRIVE_ERR_ARM_TIMEOUT);
  if(d->view.state==MQ_DRIVE_ENABLED){
    if((d->cache_mask&63u)!=63u||elapsed(now,d->speed_tick,250)||elapsed(now,d->left_tick,250)||elapsed(now,d->right_tick,250)
        ||elapsed(now,d->status_tick,250)||elapsed(now,d->fault_tick,250)||elapsed(now,d->current_tick,250))fault(d,DRIVE_ERR_SAMPLE);
    else if(d->fault_raw||!statuses(d->status_raw,7)||magnitude((int16_t)d->current_raw)>40u||magnitude((int16_t)(d->current_raw>>16))>40u)fault(d,DRIVE_ERR_CAN);
  }
  if(d->transaction.pending||d->transaction.done||d->tx_suppressed)return;
  /* STOP's first physical commands precede background queries. */
  if(d->flow==FLOW_STOP&&d->flow_step==2&&(d->stationary_ok||elapsed(now,d->stop_wait_started_ms,600)))d->flow_step=3;
  if(d->flow==FLOW_STOP&&d->flow_step!=2){stop_step(d);return;}
  if(elapsed(now,d->last_speed_start,50)){
    if(start(d,0x606c,3,4,false,0,OWNER_SPEED,0))d->last_speed_start=now;
    return;
  }
  if(d->target_pending&&d->view.state==MQ_DRIVE_ENABLED){
    start(d,0x60ff,3,4,true,pack(d->desired_left,d->desired_right),OWNER_TARGET,d->desired_generation);return;
  }
  if(d->view.state==MQ_DRIVE_ENABLED&&elapsed(now,d->last_target_ms,100)){
    start(d,0x60ff,3,4,true,pack(d->desired_left,d->desired_right),OWNER_REFRESH,d->view.applied_generation);return;
  }
  if(d->flow==FLOW_ARM){
    /* Preflight cannot starve the actual encoder/health sources it relies on.
     * A 30ms auxiliary cadence leaves time for bounded ARM progress. */
    if(auxiliary(d,30))return;
    const ArmStep *s=&arm_steps[d->flow_step];
    start(d,s->index,s->sub,s->bytes,s->check==LIMIT_WRITE,s->check==LIMIT_WRITE?arm_value(d,s):0,OWNER_FLOW,0);return;
  }
  if(d->flow==FLOW_STOP&&stop_step(d))return;
  if(elapsed(now,d->last_guard_start,200)&&d->view.state!=MQ_DRIVE_STOPPING){
    if(start(d,0x2000,0,2,false,0,OWNER_GUARD,0))d->last_guard_start=now;
    return;
  }
  /* Each auxiliary field gets a fresh SDO at least every 5*20ms nominally.
   * Slow/failed CAN is reported; its old source is never rejuvenated. */
  auxiliary(d,10);
}

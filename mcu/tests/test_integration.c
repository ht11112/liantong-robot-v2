/* Independent closed loop: production Drive + MQ; all CAN traffic stays in RAM. */
#include "drive.h"
#include "mq.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  Drive d;MQ q;uint32_t now,reply_at,status,guard,fault,current;
  uint32_t target,position_left,position_right,limits[5],decel_until;
  uint8_t pending[8];bool waiting,drop_target_ack;
  int16_t actual_left,actual_right;
  unsigned sends,motion_writes,turn_writes,move_writes,zero_writes,stop_words;
  uint32_t samples,last_sample,max_gap;
} ClosedLoop;
static ClosedLoop p;
static uint32_t read32(const uint8_t *v){return (uint32_t)v[0]|((uint32_t)v[1]<<8)|((uint32_t)v[2]<<16)|((uint32_t)v[3]<<24);}
static uint32_t packed(int16_t l,int16_t r){return (uint16_t)l|((uint32_t)(uint16_t)r<<16);}
static bool ready(void *u){(void)u;return true;}
static uint32_t errors(void *u){(void)u;return 0;}
static bool send(void *u,const uint8_t request[8]){
  ClosedLoop *a=u;assert(!a->waiting);memcpy(a->pending,request,8);
  a->waiting=true;a->reply_at=a->now+1u;a->sends++;return true;
}
static unsigned limit_slot(uint16_t index,uint8_t sub){
  if(index==0x2014)return sub-1u;
  if(index==0x2015)return sub+1u;
  return 4;
}
static uint32_t object_value(ClosedLoop *a,uint16_t index,uint8_t sub,unsigned *width){
  *width=4;
  switch(index){
    case 0x2035:*width=2;return 2400;
    case 0x603f:return a->fault;
    case 0x606c:return packed(a->actual_left,a->actual_right);
    case 0x6041:return a->status|(a->status<<16);
    case 0x6060:case 0x6061:*width=1;return 3;
    case 0x200f:case 0x2026:*width=2;return 1;
    case 0x200c:*width=2;return 15;
    case 0x200e:*width=2;return 4096;
    case 0x60ff:return sub==3?a->target:sub==1?(uint32_t)(int32_t)(int16_t)a->target:(uint32_t)(int32_t)(int16_t)(a->target>>16);
    case 0x2034:*width=2;return 1;
    case 0x6083:case 0x6084:return 100;
    case 0x2014:case 0x2015:case 0x2008:*width=2;return a->limits[limit_slot(index,sub)];
    case 0x2000:*width=2;return a->guard;
    case 0x6064:return sub==1?a->position_left:a->position_right;
    case 0x6077:return a->current;
    default:assert(!"unrecognised object in production CAN request");return 0;
  }
}
static void apply_write(ClosedLoop *a,uint16_t index,uint8_t sub,uint32_t value){
  if(index==0x60ff){
    uint32_t previous=a->target;
    a->target=value;
    int16_t l=(int16_t)value,r=(int16_t)(value>>16);
    assert(l>=-5&&l<=5&&r>=-5&&r<=5);
    if(value){
      assert(a->status==7u);a->motion_writes++;
      if(l==r)a->turn_writes++;else {assert(l>0&&r==-l);a->move_writes++;}
      a->actual_left=(int16_t)(l*10);a->actual_right=(int16_t)(r*10);
    }else {a->zero_writes++;if(previous)a->decel_until=a->now+120u;}
  }else if(index==0x6040){
    a->stop_words++;a->status=value==6u?1u:value==7u?3u:value==15u?7u:0u;
    if(value==6u||value==0u||value==2u)a->decel_until=a->now+120u;
  }else if(index==0x2000){assert(value==500u);a->guard=value;}
  else {unsigned i=limit_slot(index,sub);assert(value<=a->limits[i]);a->limits[i]=value;}
}
static void reply(ClosedLoop *a){
  if(!a->waiting||(int32_t)(a->now-a->reply_at)<0)return;
  uint8_t q[8];memcpy(q,a->pending,8);a->waiting=false;
  uint16_t index=(uint16_t)(q[1]|((uint16_t)q[2]<<8));uint8_t sub=q[3];
  uint8_t b[8]={0,q[1],q[2],sub,0,0,0,0};
  if(q[0]!=0x40){
    uint32_t value=read32(q+4);apply_write(a,index,sub,value);b[0]=0x60;
    if(a->drop_target_ack&&index==0x60ff&&sub==3&&value){a->drop_target_ack=false;return;}
  }else{
    unsigned width;uint32_t value=object_value(a,index,sub,&width);
    b[0]=(uint8_t)(0x43u|((4u-width)<<2));for(unsigned i=0;i<4;i++)b[4+i]=(uint8_t)(value>>(i*8));
  }
  drive_receive_can(&a->d,0x581u,8,b,a->now);
}
static void tick(void){
  p.now++;
  if(p.target==0u&&(int32_t)(p.now-p.decel_until)>=0)p.actual_left=p.actual_right=0;
  /* These raw counters are opaque metadata, deliberately never converted to metres. */
  p.position_left+=(uint32_t)(int32_t)(p.actual_left>0?1:p.actual_left<0?-1:0);
  p.position_right+=(uint32_t)(int32_t)(p.actual_right>0?1:p.actual_right<0?-1:0);
  reply(&p);drive_poll(&p.d,p.now);MQSample s;
  while(drive_pop_sample(&p.d,&s)){
    p.samples++;if(p.last_sample){uint32_t gap=s.tick_ms-p.last_sample;if(gap>p.max_gap)p.max_gap=gap;}
    p.last_sample=s.tick_ms;mq_sample(&p.q,&s);
  }
  mq_tick(&p.q,p.now);
}
static void boot(void){
  memset(&p,0,sizeof(p));p.guard=500u;for(unsigned i=0;i<5;i++)p.limits[i]=i==4?10u:30u;
  const DriveBusOps bus={send,ready,errors};drive_init(&p.d,&bus,&p,0);
  mq_init(&p.q,UINT64_C(0xaca5102026110401),drive_ops(),&p.d);
  for(unsigned i=0;i<2000u && (p.d.view.state!=MQ_DRIVE_STOPPED||!p.q.odom.initialized||!p.q.odom.valid);i++)tick();
  assert(p.d.view.state==MQ_DRIVE_STOPPED&&p.d.view.stop_verified&&p.q.odom.valid);
}
static void upload_start(const MQSegment *s,uint16_t count){
  assert(mq_begin(&p.q,1u,p.q.odom.epoch,count,mq_segments_crc(s,count),MQ_MAX_DURATION_MS)==MQ_OK);
  for(uint16_t i=0;i<count;i++)assert(mq_append(&p.q,1u,i,s[i])==MQ_OK);
  assert(mq_commit(&p.q,1u)==MQ_OK);assert(!p.motion_writes);
  assert(mq_start(&p.q,1u,p.now)==MQ_OK);
}
static void to_moving(void){
  for(unsigned i=0;i<4000u && (p.q.state!=MQ_MOVE||p.q.progress<.02f);i++)tick();
  assert(p.q.state==MQ_MOVE && p.q.progress>=.02f && !p.q.first_error);
}
static void closed_queue(float angle){
  boot();MQSegment s[3]={{MQ_KIND_MOVE,.5f},{MQ_KIND_TURN,angle},{MQ_KIND_MOVE,.5f}};upload_start(s,3);
  /* No host API, RX byte, renewal, or new path is sent after this START. */
  for(unsigned i=0;i<200000u&&p.q.state!=MQ_COMPLETE&&p.q.state!=MQ_FAILED;i++)tick();
  if(p.q.state!=MQ_COMPLETE)printf("INTEGRATION DIAG tick=%u q=%u err=%u drive=%u derr=%u flow=%u step=%u sample=%u odo_valid=%u progress=%f target_gen=%u applied_gen=%u\n",(unsigned)p.now,(unsigned)p.q.state,(unsigned)p.q.first_error,(unsigned)p.d.view.state,(unsigned)p.d.view.error,(unsigned)p.d.flow,(unsigned)p.d.flow_step,(unsigned)p.q.odom.sample_sequence,(unsigned)p.q.odom.valid,p.q.progress,(unsigned)p.q.target_generation,(unsigned)p.d.view.applied_generation);
  assert(p.q.state==MQ_COMPLETE && p.q.arrival_reached && p.q.physical_stop_confirmed);
  assert(!p.q.first_error&&!p.d.view.error&&p.d.view.stop_verified&&p.q.odom.valid);
  assert(p.move_writes>50u&&p.turn_writes>50u&&p.samples>1000u&&p.max_gap<=55u);
  assert(p.q.odom.travel>.85f&&p.q.odom.travel<1.1f);
  assert(angle>0.f?p.q.odom.y>.3f:p.q.odom.y<-.3f);
  assert(p.q.odom.yaw_total*angle>0.f && p.now-p.q.started_ms>500u);
  printf("PASS integration real Drive+MQ MOVE .5 / TURN %s90 / MOVE .5, offline %u ms, %u sources, max gap %u ms\n",angle>0.f?"+":"-",(unsigned)(p.now-p.q.started_ms),(unsigned)p.samples,(unsigned)p.max_gap);
}
static void cancellation(void){
  boot();MQSegment s={MQ_KIND_MOVE,.5f};upload_start(&s,1);to_moving();mq_stop(&p.q,p.now);
  for(unsigned i=0;i<2500u&&p.q.state!=MQ_FAILED;i++)tick();
  assert(p.q.state==MQ_FAILED&&!p.q.arrival_reached&&p.q.first_error==MQ_ERR_CANCEL);
  assert(p.q.physical_stop_confirmed&&p.d.view.stop_verified&&!p.d.view.error);
  puts("PASS integration cancellation remains failed task while real STOP proof succeeds");
}
static void drive_fault(void){
  boot();MQSegment s={MQ_KIND_MOVE,.5f};upload_start(&s,1);to_moving();p.fault=0x100u;
  for(unsigned i=0;i<3000u&&p.q.state!=MQ_FAILED;i++)tick();
  assert(p.q.state==MQ_FAILED&&!p.q.arrival_reached&&p.q.first_error);
  assert(p.q.first_drive_error&&p.d.view.error&&!p.q.physical_stop_confirmed&&p.d.tx_suppressed);
  puts("PASS integration real CAN drive fault terminates queue and cannot upgrade parking proof");
}
static void dropped_target_ack(void){
  boot();MQSegment s={MQ_KIND_MOVE,.5f};upload_start(&s,1);to_moving();p.drop_target_ack=true;
  for(unsigned i=0;i<3000u&&p.q.state!=MQ_FAILED;i++)tick();
  assert(p.q.state==MQ_FAILED&&!p.q.arrival_reached&&p.q.first_error);
  assert(p.d.sdo_ambiguous&&p.d.view.error&&p.d.tx_suppressed&&!p.q.physical_stop_confirmed);
  puts("PASS integration target may physically apply after lost ACK but queue and STOP evidence stay failed");
}
int main(void){
  closed_queue(MQ_PI*.5f);closed_queue(-MQ_PI*.5f);cancellation();drive_fault();dropped_target_ack();
  puts("PASS all independent production Drive+MQ closed-loop tests");return 0;
}

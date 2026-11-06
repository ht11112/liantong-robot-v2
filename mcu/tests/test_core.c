#ifndef MQ_HOST_TEST
#error "Test harness is never an MCU production source"
#endif
#include "mq.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {MQDriveView d;unsigned arms,stops,targets;uint32_t generation;} Fake;
static void arm(void *p,uint32_t n){(void)n;Fake*f=p;f->arms++;f->d.state=MQ_DRIVE_ENABLED;f->d.stop_verified=false;}
static uint32_t target(void *p,int16_t l,int16_t r,uint32_t n){
  Fake*f=p;f->targets++;f->d.applied_generation=++f->generation;
  f->d.applied_tick_ms=n;f->d.applied_left=l;f->d.applied_right=r;return f->generation;
}
static void stop(void*p,uint32_t n){(void)n;Fake*f=p;f->stops++;f->d.state=MQ_DRIVE_STOPPED;f->d.stop_verified=true;f->d.applied_left=f->d.applied_right=0;}
static MQDriveView view(void*p){return ((Fake*)p)->d;}
static const MQDriveOps ops={arm,target,stop,view};
static void init(MQ*q,Fake*f){memset(f,0,sizeof(*f));f->d.state=MQ_DRIVE_STOPPED;f->d.stop_verified=f->d.guard_valid=true;mq_init(q,0x12345678,&ops,f);}
static void sample(MQ*q,uint32_t seq,uint32_t n,int16_t l,int16_t r){MQSample s={.sequence=seq,.tick_ms=n,.left_tenths_rpm=l,.right_tenths_rpm=r,.valid=true};mq_sample(q,&s);}
static void load(MQ*q,MQSegment*s,uint16_t count,uint64_t task){
  assert(mq_begin(q,task,q->odom.epoch,count,mq_segments_crc(s,count),1200000)==MQ_OK);
  for(uint16_t i=0;i<count;i++)assert(mq_append(q,task,i,s[i])==MQ_OK);
  assert(mq_commit(q,task)==MQ_OK);
}
static void exact_odom(void){
  MQ q;Fake f;init(&q,&f);sample(&q,0,0,20,-20);
  for(uint32_t i=1;i<=200;i++)sample(&q,i,i*50,20,-20);
  assert(fabsf(q.odom.x-10.f*2.f*MQ_PI*.2f/60.f)<.00001f);
  assert(fabsf(q.odom.y)<.00001f && fabsf(q.odom.yaw)<.00001f);
  MQ z;init(&z,&f);sample(&z,0,0,1,0);sample(&z,1,50,1,0);
  assert(z.odom.travel>0.f); /* No hidden 0.2RPM odometry deadband. */
  puts("PASS exact straight integration and sub-threshold real RPM");
}
static void wheel_feedback_validity(void){
  MQ q;Fake f;init(&q,&f);sample(&q,1,0,0,0);
  MQSegment move={MQ_KIND_MOVE,1.f};
  load(&q,&move,1,1);assert(mq_start(&q,1,0)==MQ_OK);
  mq_tick(&q,1);
  sample(&q,2,50,151,-175);
  mq_tick(&q,50);
  assert(q.odom.valid&&q.odom.sample_sequence==2&&q.odom.x>0.f);
  assert(q.odom.left_tenths_rpm==151&&q.odom.right_tenths_rpm==-175);
  assert(q.state==MQ_MOVE&&q.first_error==MQ_ERR_NONE&&!f.stops);
  MQSample invalid={.sequence=3,.tick_ms=100,.left_tenths_rpm=151,.right_tenths_rpm=-175};
  mq_sample(&q,&invalid);
  assert(!q.odom.valid&&q.first_error==MQ_ERR_ODOM&&q.state==MQ_STOPPING&&f.stops==1);
  puts("PASS valid feedback above 15 RPM integrates, invalid feedback stops the task");
}
static void multi_segment(void){
  MQ q;Fake f;init(&q,&f);sample(&q,1,0,0,0);
  MQSegment s[]={{MQ_KIND_MOVE,.40f},{MQ_KIND_TURN,MQ_PI/2.f},{MQ_KIND_MOVE,.40f}};
  load(&q,s,3,1);assert(mq_start(&q,1,0)==MQ_OK);
  uint32_t n=0,seq=1;
  while(q.state!=MQ_COMPLETE && q.state!=MQ_FAILED && n<120000) {
    n+=10;
    if(n%50==0)sample(&q,++seq,n,(int16_t)(f.d.applied_left*10),(int16_t)(f.d.applied_right*10));
    mq_tick(&q,n);
  }
  assert(q.state==MQ_COMPLETE && q.arrival_reached && q.physical_stop_confirmed);
  assert(f.arms==1 && f.stops==1 && q.index==2);
  assert(fabsf(q.odom.x-.38f)<.03f && q.odom.y>.28f && q.odom.y<.33f);
  unsigned arms=f.arms;assert(mq_start(&q,1,n)==MQ_OK);assert(f.arms==arms);
  printf("PASS closed-loop MOVE TURN MOVE, simulated elapsed=%u ms x=%.6f y=%.6f\n",(unsigned)n,(double)q.odom.x,(double)q.odom.y);
}
static void crc_contract(void){
  MQSegment s[]={{1,.5f},{2,-1.5f}};
  /* Python zlib.crc32(struct.pack('<IfIf',1,.5,2,-1.5)) cross-language vector. */
  assert(mq_segments_crc(s,2)==0x615aabe9u);
  MQ q;Fake f;init(&q,&f);sample(&q,1,0,0,0);
  assert(mq_begin(&q,1,1,2,0,1200000)==MQ_OK);
  assert(mq_append(&q,1,1,s[1])==MQ_ORDER);
  assert(mq_append(&q,1,0,s[0])==MQ_OK);
  assert(mq_commit(&q,1)==MQ_ORDER);
  assert(mq_start(&q,1,0)==MQ_ORDER);
  assert(mq_append(&q,1,1,s[1])==MQ_OK);
  assert(mq_commit(&q,1)==MQ_CRC);
  assert(!f.arms && !f.targets);puts("PASS partial, ordered and CRC-bound upload");
}
static void uart_fault_stays_latched(void){
  MQ q;Fake f;init(&q,&f);sample(&q,1,0,0,0);
  mq_fault(&q,MQ_ERR_UART,0);mq_tick(&q,1);
  assert(q.physical_stop_confirmed && q.state==MQ_FAILED);
  MQSegment s={1,.5f};
  assert(mq_begin(&q,1,1,1,mq_segments_crc(&s,1),1200000)==MQ_UNHEALTHY);
  assert(mq_reset_odom(&q,1)==MQ_UNHEALTHY);
  init(&q,&f);sample(&q,1,0,0,0);
  mq_fault(&q,MQ_ERR_CANCEL,0);mq_fault(&q,MQ_ERR_UART,0);mq_tick(&q,1);
  assert(q.first_error==MQ_ERR_CANCEL && q.critical_latched);
  assert(mq_begin(&q,1,1,1,mq_segments_crc(&s,1),1200000)==MQ_UNHEALTHY);
  assert(mq_reset_odom(&q,1)==MQ_UNHEALTHY);
  puts("PASS critical UART fault survives physical STOP, new task and reset");
}
int main(void){exact_odom();wheel_feedback_validity();multi_segment();crc_contract();uart_fault_stays_latched();return 0;}

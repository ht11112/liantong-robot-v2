/* Actual production drive.c with a memory-only CANopen peer. No HAL/device API. */
#include "drive.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  Drive d;uint32_t now,status,guard,target,bus_error,delay,decel_until;
  uint32_t speed,current,positions[2],pole_override,max_speed,max_speed_reads,max_speed_writes;
  uint32_t sends,target_writes,zero_writes,fallback_writes,speed_reads,max_speed_gap,last_speed_sample;
  uint8_t request[8];bool waiting,drop_all,drop_next,force_speed,send_fail,fifo_blocked;
  uint32_t sent_at;MQSample samples[2000];unsigned sample_count;
} Peer;
static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static bool ready(void *u){return !((Peer *)u)->fifo_blocked;}
static uint32_t errors(void *u){return ((Peer *)u)->bus_error;}
static bool send(void *u,const uint8_t b[8]){
  Peer *p=u;p->sends++;if(p->send_fail)return false;
  assert(!p->waiting||(uint32_t)(p->now-p->sent_at)>=50u);memcpy(p->request,b,8);p->waiting=true;p->sent_at=p->now;
  if(p->drop_next){p->drop_next=false;p->waiting=false;return true;}
  return true;
}
static uint32_t read_value(Peer *p,uint16_t index,uint8_t sub,unsigned *bytes){
  *bytes=4;
  switch(index){
  case 0x2035:*bytes=2;return 2400;
  case 0x603f:return 0;
  case 0x606c:
    p->speed_reads++;
    if(p->force_speed)return p->speed;
    if(p->status!=7u){if((int32_t)(p->now-p->decel_until)>=0)p->speed=0;return p->speed;}
    p->speed=(uint16_t)((int16_t)p->target*10)|((uint32_t)(uint16_t)((int16_t)(p->target>>16)*10)<<16);return p->speed;
  case 0x6041:return p->status|(p->status<<16);
  case 0x6060:case 0x6061:*bytes=1;return 3;
  case 0x200f:case 0x2026:*bytes=2;return 1;
  case 0x200c:*bytes=2;return p->pole_override?p->pole_override:15;
  case 0x200e:*bytes=2;return 4096;
  case 0x60ff:return sub==3?p->target:sub==1?(uint32_t)(int32_t)(int16_t)p->target:(uint32_t)(int32_t)(int16_t)(p->target>>16);
  case 0x2034:*bytes=2;return 1;
  case 0x6083:case 0x6084:return 100;
  case 0x2014:case 0x2015:*bytes=2;return 30;
  case 0x2008:*bytes=2;p->max_speed_reads++;return p->max_speed;
  case 0x2000:*bytes=2;return p->guard;
  case 0x6064:return p->positions[sub-1];
  case 0x6077:return p->current;
  default:assert(!"Unexpected CAN object");return 0;
  }
}
static void respond(Peer *p){
  if(!p->waiting||p->drop_all||(uint32_t)(p->now-p->sent_at)<p->delay)return;
  uint8_t *q=p->request;uint16_t index=(uint16_t)(q[1]|((uint16_t)q[2]<<8));uint8_t sub=q[3];
  uint8_t b[8]={0,q[1],q[2],sub,0,0,0,0};
  if(q[0]!=0x40){
    uint32_t value=u32(q+4);b[0]=0x60;
    if(index==0x60ff){p->target=value;p->target_writes++;if(!value)p->zero_writes++;}
    if(index==0x6040){
      if(value==2u)p->fallback_writes++;
      if(value==6u){p->status=1;p->decel_until=p->now+150;}
      else p->status=value==7u?3u:value==15u?7u:0u;
    }
    if(index==0x2000)p->guard=value;
    if(index==0x2008){p->max_speed=value;p->max_speed_writes++;}
  }else{
    unsigned bytes;uint32_t value=read_value(p,index,sub,&bytes);b[0]=(uint8_t)(0x43u|((4u-bytes)<<2));
    for(unsigned i=0;i<4;i++)b[4+i]=(uint8_t)(value>>(i*8));
  }
  p->waiting=false;drive_receive_can(&p->d,0x581,8,b,p->now);
}
static void tick(Peer *p){
  p->now++;respond(p);drive_poll(&p->d,p->now);MQSample s;
  while(drive_pop_sample(&p->d,&s)){
    assert(p->sample_count<2000);p->samples[p->sample_count++]=s;
    if(p->last_speed_sample){uint32_t gap=s.tick_ms-p->last_speed_sample;if(gap>p->max_speed_gap)p->max_speed_gap=gap;}
    p->last_speed_sample=s.tick_ms;
  }
}
static void run(Peer *p,unsigned ms){for(unsigned i=0;i<ms;i++)tick(p);}
static void init(Peer *p){
  memset(p,0,sizeof(*p));p->delay=1;p->guard=500;p->max_speed=87;p->now=1;
  static const DriveBusOps ops={send,ready,errors};drive_init(&p->d,&ops,p,p->now);
}
static void boot(Peer *p){init(p);run(p,350);assert(p->d.view.state==MQ_DRIVE_STOPPED);assert(p->d.view.stop_verified&&p->d.view.guard_valid);}
static void armed(Peer *p){boot(p);drive_ops()->arm(&p->d,p->now);run(p,220);assert(p->d.view.state==MQ_DRIVE_ENABLED);assert(!p->d.view.error);}
static void test_boot_arm_refresh(void){
  Peer p;armed(&p);uint32_t generation=drive_ops()->target(&p.d,2,-2,p.now);assert(generation);
  run(&p,12);assert(p.d.view.applied_generation==generation);assert(p.d.view.applied_left==2&&p.d.view.applied_right==-2);
  uint32_t count=p.target_writes;run(&p,320);assert(p.target_writes>=count+3);assert(!p.d.view.error);assert(p.max_speed_gap<=55);
  puts("PASS drive boot, ARM object/readback, target generation, local refresh");
}
static void test_stop_sampling_and_inflight(void){
  Peer p;armed(&p);drive_ops()->target(&p.d,2,-2,p.now);run(&p,70);
  uint32_t before=p.sample_count,z=p.zero_writes;p.delay=4;
  drive_ops()->target(&p.d,1,-1,p.now);tick(&p);drive_ops()->stop(&p.d,p.now);run(&p,450);
  assert(p.d.view.stop_verified&&p.d.view.state==MQ_DRIVE_STOPPED);assert(p.zero_writes>z);assert(p.sample_count>before+5);assert(p.max_speed_gap<=65);
  puts("PASS drive STOP keeps fresh speed samples and cannot consume old target ACK as flow ACK");
}
static void test_speed_feedback_and_failed_stop(void){
  Peer p;armed(&p);p.force_speed=true;p.speed=151u|((uint32_t)(uint16_t)-173<<16);
  unsigned sample_count=p.sample_count;
  uint32_t generation=drive_ops()->target(&p.d,20,-20,p.now);assert(generation);
  run(&p,150);
  assert(p.d.view.state==MQ_DRIVE_ENABLED&&!p.d.view.error&&p.d.view.applied_generation==generation);
  assert(p.sample_count>sample_count);
  const MQSample *sample=&p.samples[p.sample_count-1];
  assert(sample->valid&&sample->left_tenths_rpm==151&&sample->right_tenths_rpm==-173);
  drive_ops()->stop(&p.d,p.now);run(&p,800);
  assert(p.d.zero_ok&&p.d.shutdown_ok&&p.d.disable_ok&&!p.d.stationary_ok);
  assert(p.d.view.state==MQ_DRIVE_FAILED&&p.d.view.error==DRIVE_ERR_STOP);
  assert(!p.d.view.stop_verified&&p.d.tx_suppressed&&p.target==0&&p.fallback_writes==1&&p.guard==500);
  uint32_t sends=p.sends;run(&p,501);assert(p.sends==sends);
  puts("PASS drive accepts feedback above 15 RPM and explicit STOP still needs zero feedback or falls back to watchdog silence");
}
static void test_uart_fault_latch(void){
  Peer p;boot(&p);drive_uart_fault(&p.d,p.now);run(&p,300);assert(p.d.view.error==DRIVE_ERR_UART);assert(p.d.view.stop_verified);
  drive_ops()->stop(&p.d,p.now);run(&p,300);drive_ops()->arm(&p.d,p.now);run(&p,100);
  assert(p.d.view.error==DRIVE_ERR_UART&&p.d.view.state==MQ_DRIVE_FAILED);
  puts("PASS drive UART first cause survives STOP and new ARM");
}
static void test_current_bus_guard(void){
  Peer p;armed(&p);p.current=40;run(&p,100);assert(!p.d.view.error);p.current=41;run(&p,100);assert(p.d.view.error==DRIVE_ERR_CAN);
  armed(&p);p.bus_error=3;tick(&p);assert(p.d.first_can_error==3&&p.d.view.error==DRIVE_ERR_CAN);
  armed(&p);p.guard=499;run(&p,220);assert(p.d.view.error==DRIVE_ERR_GUARD);run(&p,350);assert(p.guard==500&&p.d.view.state==MQ_DRIVE_FAILED);
  puts("PASS drive current boundary, bus error and watchdog mismatch retain first cause");
}
static void test_bad_config_timeout_and_late_reply(void){
  Peer p;boot(&p);p.pole_override=16;drive_ops()->arm(&p.d,p.now);run(&p,100);assert(p.d.view.error==DRIVE_ERR_CAN);
  boot(&p);while(!p.d.transaction.pending)tick(&p);
  uint8_t bad[8]={0x43,0x99,0x99,0,0,0,0,0};drive_receive_can(&p.d,0x581,8,bad,p.now);assert(p.d.transaction.pending&&p.d.late_replies);
  p.drop_all=true;run(&p,1400);assert(p.d.view.error==DRIVE_ERR_CAN&&p.d.tx_suppressed&&!p.d.view.stop_verified);
  uint32_t sends=p.sends;run(&p,500);assert(p.sends==sends);
  puts("PASS drive wrong-index reply, CAN timeout, invalid preflight and bounded failed-STOP silence");
}
static void test_boot_guard_and_target_encoding(void){
  Peer p;init(&p);p.guard=0;run(&p,350);assert(p.guard==500&&p.d.view.guard_valid&&p.d.view.stop_verified);
  armed(&p);assert(p.max_speed_reads==2&&!p.max_speed_writes&&p.max_speed==87);
  /* Feedback is independent of the requested value or the driver's SDO ACK. */
  p.force_speed=true;p.speed=0;
  uint32_t generation=drive_ops()->target(&p.d,6,-17,p.now);assert(generation);
  run(&p,60);assert(p.target==0xffef0006u&&p.d.view.applied_generation==generation&&!p.d.view.error);
  generation=drive_ops()->target(&p.d,INT16_MAX,INT16_MIN,p.now);assert(generation);
  run(&p,60);assert(p.target==0x80007fffu&&p.d.view.applied_generation==generation&&!p.d.view.error);
  puts("PASS drive watchdog, read-only max speed and exact signed I16 target packing");
}
static void test_original_completion_clock(void){
  Peer p;armed(&p);
  while(!p.d.transaction.pending||p.d.transaction.index!=0x606c)tick(&p);
  p.now++;respond(&p);assert(p.d.transaction.done);uint32_t received=p.now;
  p.now+=20;drive_poll(&p.d,p.now);MQSample sample;assert(drive_pop_sample(&p.d,&sample));assert(sample.tick_ms==received);
  uint32_t generation=drive_ops()->target(&p.d,2,-2,p.now);assert(generation);
  while(!p.d.transaction.pending||p.d.transaction.index!=0x60ff||p.d.transaction.generation!=generation)tick(&p);
  p.now++;respond(&p);received=p.now;p.now+=20;drive_poll(&p.d,p.now);
  assert(p.d.view.applied_generation==generation&&p.d.view.applied_tick_ms==received);
  puts("PASS drive uses original CAN completion tick after delayed main-loop processing");
}
static void test_arm_keeps_aux_sources(void){
  Peer p;boot(&p);p.delay=10;drive_ops()->arm(&p.d,p.now);run(&p,1800);
  assert(p.d.view.state==MQ_DRIVE_ENABLED&&!p.d.view.error);
  assert((uint32_t)(p.now-p.d.left_tick)<250u&&(uint32_t)(p.now-p.d.right_tick)<250u);
  assert((uint32_t)(p.now-p.d.status_tick)<250u&&(uint32_t)(p.now-p.d.fault_tick)<250u);
  puts("PASS drive slow valid SDO preflight still refreshes encoder and health sources");
}
static void test_ambiguous_late_target_ack(void){
  Peer p;armed(&p);p.drop_next=true;uint32_t generation=drive_ops()->target(&p.d,2,-2,p.now);assert(generation);
  while(!p.d.sdo_ambiguous)tick(&p);
  while(!p.d.transaction.pending||p.d.transaction.index!=0x60ff)tick(&p);
  uint8_t late[8]={0x60,0xff,0x60,3,0,0,0,0};
  /* Drop the new zero's real reply and deliver only the indistinguishable old ACK. */
  p.waiting=false;drive_receive_can(&p.d,0x581,8,late,p.now);drive_poll(&p.d,p.now);
  assert(!p.d.zero_ok);run(&p,1000);
  assert(p.d.view.error&&p.d.sdo_ambiguous&&p.d.tx_suppressed&&!p.d.view.stop_verified);
  drive_ops()->stop(&p.d,p.now);run(&p,1000);
  assert(p.d.sdo_ambiguous&&!p.d.view.stop_verified&&p.d.view.state==MQ_DRIVE_FAILED);
  puts("PASS drive same-object late ACK cannot upgrade STOP after any SDO timeout, including explicit recovery");
}
static void test_unavailable_tx_is_bounded(void){
  Peer p;init(&p);p.fifo_blocked=true;run(&p,2100);
  assert(p.d.view.state==MQ_DRIVE_FAILED&&p.d.view.error==DRIVE_ERR_STOP&&p.d.tx_suppressed&&!p.d.view.stop_verified);
  assert(p.sends==0);run(&p,500);assert(p.sends==0);
  puts("PASS drive unavailable CAN TX cannot leave boot STOP pending indefinitely");
}
int main(void){
  test_boot_arm_refresh();test_stop_sampling_and_inflight();test_speed_feedback_and_failed_stop();test_uart_fault_latch();test_current_bus_guard();test_bad_config_timeout_and_late_reply();test_boot_guard_and_target_encoding();
  test_original_completion_clock();test_arm_keeps_aux_sources();test_ambiguous_late_target_ack();test_unavailable_tx_is_bounded();
  puts("PASS all production drive C tests (memory-only CAN peer)");return 0;
}

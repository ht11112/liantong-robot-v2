#ifndef MQ_HOST_TEST
#error "Host fault tests must never be compiled into firmware"
#endif
#include "mq.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Queue-only simulation. No UART, HAL, CAN, or physical stop proof exists. */
typedef struct { MQDriveView view;unsigned arms,stops,views; } Motor;
typedef struct { MQ queue;Motor motor; } Fixture;
static void arm(void *p,uint32_t now){Motor *m=p;(void)now;m->arms++;m->view.state=MQ_DRIVE_ARMING;}
static uint32_t target(void *p,int16_t l,int16_t r,uint32_t now){(void)p;(void)l;(void)r;(void)now;return 1;}
static void stop(void *p,uint32_t now){Motor *m=p;(void)now;m->stops++;m->view.state=MQ_DRIVE_STOPPING;m->view.stop_verified=false;}
static MQDriveView view(void *p){Motor *m=p;m->views++;return m->view;}
static const MQDriveOps drive_ops={arm,target,stop,view};
static void setup(Fixture *f){
  memset(f,0,sizeof(*f));f->motor.view.state=MQ_DRIVE_STOPPED;
  f->motor.view.stop_verified=f->motor.view.guard_valid=true;
  mq_init(&f->queue,UINT64_C(0x1234567890abcdef),&drive_ops,&f->motor);
  MQSample sample={.sequence=1,.tick_ms=100,.valid=true};mq_sample(&f->queue,&sample);
  MQSegment segment={MQ_KIND_MOVE,.5f};
  assert(mq_begin(&f->queue,42,1,1,mq_segments_crc(&segment,1),MQ_LEASE_MS)==MQ_OK);
  assert(mq_append(&f->queue,42,0,segment)==MQ_OK);
  assert(mq_commit(&f->queue,42)==MQ_OK);
}
static void healthy_repeat_stays_inert(void){
  Fixture f;setup(&f);
  assert(mq_start(&f.queue,42,100)==MQ_OK);
  assert(mq_start(&f.queue,42,101)==MQ_OK);
  assert(f.motor.arms==1 && f.queue.has_started && f.queue.state==MQ_ARMING);
  puts("PASS MQ healthy repeated START remains idempotent");
}
static void latched_ready_cannot_start(void){
  Fixture f;setup(&f);
  /* Exercise the independent boot latch even before main publishes its cause. */
  f.queue.critical_latched=true;
  unsigned views=f.motor.views;
  assert(mq_start(&f.queue,42,100)==MQ_UNHEALTHY);
  assert(!f.motor.arms && !f.queue.has_started && f.queue.state==MQ_READY);
  assert(f.motor.views==views && !f.queue.first_error);
  puts("PASS MQ critical latch rejects first START before drive inspection");
}
static void critical_fault_rejects_first_and_repeated_start(void){
  for(unsigned started=0;started<2;started++){
    for(unsigned boot_fault=0;boot_fault<2;boot_fault++){
      Fixture f;setup(&f);
      if(started)assert(mq_start(&f.queue,42,100)==MQ_OK);
      uint32_t cause=boot_fault?MQ_ERR_BOOT:MQ_ERR_UART;
      mq_fault(&f.queue,cause,101);
      unsigned arms=f.motor.arms,stops=f.motor.stops,views=f.motor.views;
      assert(mq_start(&f.queue,42,102)==MQ_UNHEALTHY);
      assert(f.queue.critical_latched && f.queue.first_error==cause && f.queue.first_error_tick_ms==101);
      assert(f.motor.arms==arms && f.motor.stops==stops && f.motor.views==views);
      assert(f.queue.state==MQ_STOPPING && f.queue.has_started==(bool)started);
    }
  }
  puts("PASS MQ UART and boot faults reject both first and cached START without new effects");
}
static void stop_and_reset_cannot_unlock(void){
  Fixture f;setup(&f);assert(mq_start(&f.queue,42,100)==MQ_OK);
  mq_fault(&f.queue,MQ_ERR_UART,101);mq_stop(&f.queue,102);
  assert(!f.queue.committed && f.queue.critical_latched);
  /* Synthetic stopped view only: this is not evidence of physical parking. */
  f.motor.view.state=MQ_DRIVE_STOPPED;f.motor.view.stop_verified=true;mq_tick(&f.queue,103);
  assert(f.queue.state==MQ_FAILED && f.queue.physical_stop_confirmed);
  assert(mq_reset_odom(&f.queue,104)==MQ_UNHEALTHY);
  assert(mq_start(&f.queue,42,104)==MQ_UNHEALTHY && f.motor.arms==1);
  assert(f.queue.odom.epoch==1 && f.queue.first_error==MQ_ERR_UART && f.queue.first_error_tick_ms==101);
  puts("PASS MQ synthetic STOP completion and RESET do not clear critical START latch");
}
static void identity_and_first_cause_are_preserved(void){
  Fixture f;setup(&f);assert(mq_start(&f.queue,42,100)==MQ_OK);
  mq_fault(&f.queue,MQ_ERR_DRIVE,101);mq_fault(&f.queue,MQ_ERR_UART,102);
  assert(mq_start(&f.queue,41,103)==MQ_IDENTITY);
  f.queue.epoch_at_upload=2;assert(mq_start(&f.queue,42,103)==MQ_IDENTITY);
  f.queue.epoch_at_upload=1;assert(mq_start(&f.queue,42,103)==MQ_UNHEALTHY);
  assert(f.queue.first_error==MQ_ERR_DRIVE && f.queue.first_error_tick_ms==101);
  assert(f.queue.critical_latched && f.motor.arms==1);
  puts("PASS MQ wrong task/epoch still fail identity and original first cause is preserved");
}
static void noncritical_repeat_does_not_rearm(void){
  Fixture f;setup(&f);assert(mq_start(&f.queue,42,100)==MQ_OK);
  mq_fault(&f.queue,MQ_ERR_CANCEL,101);
  assert(!f.queue.critical_latched && mq_start(&f.queue,42,102)==MQ_OK);
  assert(f.motor.arms==1 && f.queue.state==MQ_STOPPING && f.queue.first_error==MQ_ERR_CANCEL);
  puts("PASS MQ noncritical repeated START retains inert historical response");
}
static void recovery_window(void){
  Fixture f;setup(&f);MQSample s={.sequence=2,.tick_ms=150,.valid=false};
  mq_sample(&f.queue,&s);assert(!f.queue.odom.valid&&f.queue.odom.recovery_allowed);
  for(unsigned i=0;i<3;i++){s.sequence++;s.tick_ms+=50;s.valid=true;mq_sample(&f.queue,&s);}
  assert(f.queue.odom.valid&&f.queue.odom.travel==0.f);
  s.sequence++;s.tick_ms+=50;s.left_tenths_rpm=10;s.right_tenths_rpm=-10;mq_sample(&f.queue,&s);
  assert(f.queue.odom.valid&&f.queue.odom.travel>0.f);
  setup(&f);assert(mq_start(&f.queue,42,100)==MQ_OK);
  s=(MQSample){.sequence=2,.tick_ms=150,.valid=false};mq_sample(&f.queue,&s);
  assert(f.queue.state==MQ_STOPPING&&!f.queue.odom.recovery_allowed);
  f.motor.view.state=MQ_DRIVE_STOPPED;f.motor.view.stop_verified=true;mq_tick(&f.queue,151);
  for(unsigned i=0;i<4;i++){s.sequence++;s.tick_ms+=50;s.valid=true;mq_sample(&f.queue,&s);}
  assert(!f.queue.odom.valid);
  setup(&f);s=(MQSample){.sequence=2,.tick_ms=150,.valid=false};mq_sample(&f.queue,&s);
  s.sequence++;s.tick_ms=401;s.valid=true;mq_sample(&f.queue,&s);
  assert(!f.queue.odom.valid&&!f.queue.odom.recovery_allowed&&f.queue.odom.travel==0.f);
  setup(&f);s=(MQSample){.sequence=2,.tick_ms=150,.valid=false};mq_sample(&f.queue,&s);
  s.sequence=4;s.tick_ms=200;s.valid=true;mq_sample(&f.queue,&s);
  assert(!f.queue.odom.recovery_allowed&&!f.queue.odom.valid);
  setup(&f);s=(MQSample){.sequence=2,.tick_ms=150,.valid=false};mq_sample(&f.queue,&s);
  s.sequence++;s.tick_ms=200;s.valid=true;s.left_tenths_rpm=3;mq_sample(&f.queue,&s);
  assert(!f.queue.odom.recovery_allowed&&!f.queue.odom.valid);
  puts("PASS MQ stopped recovery excludes gaps and moving loss requires reset");
}
int main(void){
  healthy_repeat_stays_inert();latched_ready_cannot_start();
  critical_fault_rejects_first_and_repeated_start();stop_and_reset_cannot_unlock();
  identity_and_first_cause_are_preserved();noncritical_repeat_does_not_rearm();
  recovery_window();
  puts("MQ FAULT COMPLETE 6 queue-only scenarios");return 0;
}

#include "mq.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  MQDriveView view;
  uint32_t generation,requested_ms,applied_delay;
  int16_t left,right;
  unsigned targets,stops;
  uint32_t times[512];
  int16_t differences[512];
} Motor;
typedef struct { MQ q;Motor motor;uint32_t now,sequence; } Fixture;
static void arm(void *p,uint32_t now){(void)now;((Motor *)p)->view.state=MQ_DRIVE_ENABLED;}
static uint32_t target(void *p,int16_t left,int16_t right,uint32_t now){
  Motor *m=p;assert(m->targets<512);m->times[m->targets]=now;
  m->differences[m->targets++]=(int16_t)(-right-left);
  m->left=left;m->right=right;m->requested_ms=now;return ++m->generation;
}
static void stop(void *p,uint32_t now){
  (void)now;Motor *m=p;m->stops++;m->view.state=MQ_DRIVE_STOPPED;
  m->view.stop_verified=true;m->view.applied_left=m->view.applied_right=0;
}
static MQDriveView view(void *p){return ((Motor *)p)->view;}
static const MQDriveOps ops={arm,target,stop,view};
static void source(Fixture *f,uint32_t now){
  MQSample s={.sequence=++f->sequence,.tick_ms=now,.valid=true,
    .left_tenths_rpm=(int16_t)(f->motor.view.applied_left*10),
    .right_tenths_rpm=(int16_t)(f->motor.view.applied_right*10)};
  mq_sample(&f->q,&s);
}
static void apply(Fixture *f,uint32_t now){
  Motor *m=&f->motor;
  if(m->generation!=m->view.applied_generation && now-m->requested_ms>=m->applied_delay){
    m->view.applied_generation=m->generation;m->view.applied_tick_ms=now;
    m->view.applied_left=m->left;m->view.applied_right=m->right;
  }
}
static void setup(Fixture *f,uint32_t now,const MQSegment *segments,uint16_t count){
  memset(f,0,sizeof(*f));f->now=now;
  f->motor.view.state=MQ_DRIVE_STOPPED;f->motor.view.stop_verified=f->motor.view.guard_valid=true;
  mq_init(&f->q,123,&ops,&f->motor);source(f,now);
  assert(mq_begin(&f->q,1,1,count,mq_segments_crc(segments,count),MQ_LEASE_MS)==MQ_OK);
  for(uint16_t i=0;i<count;i++)assert(mq_append(&f->q,1,i,segments[i])==MQ_OK);
  assert(mq_commit(&f->q,1)==MQ_OK && mq_start(&f->q,1,now)==MQ_OK);
  mq_tick(&f->q,now);
}
static void step(Fixture *f,uint32_t delta){
  f->now+=delta;apply(f,f->now);source(f,f->now);mq_tick(&f->q,f->now);
}
static void steer(Fixture *f,int16_t bias){
  assert(mq_steer(&f->q,1,f->q.index,bias,f->q.odom.measurement_tick_ms,f->now)==MQ_OK);
}
static double averaged_bias(int16_t bias,uint32_t tick_step,uint32_t delay,uint32_t command_period){
  Fixture f;MQSegment s={MQ_KIND_MOVE_GUIDED,1000.f};setup(&f,0,&s,1);
  f.motor.applied_delay=delay;double area=0.;steer(&f,bias);mq_tick(&f.q,0);
  for(uint32_t t=tick_step;t<=20000;t+=tick_step){
    area+=(double)(-f.motor.view.applied_right-f.motor.view.applied_left)*tick_step;
    f.now=t;apply(&f,t);
    if(t%50==0)source(&f,t);
    if(t%command_period==0)steer(&f,bias);
    mq_tick(&f.q,t);
    assert(f.q.state==MQ_MOVE && !f.motor.stops);
    assert(f.motor.left>=0 && f.motor.right<=0 && f.motor.left<=2 && f.motor.right>=-2);
  }
  unsigned transitions=0;int sum=0;
  for(unsigned i=0;i<f.motor.targets;i++){
    sum+=f.motor.differences[i];
    if(i){assert(f.motor.times[i]-f.motor.times[i-1]>=100u);transitions++;}
  }
  assert(transitions>100 && fabs((double)sum/f.motor.targets-bias*.001)<.015);
  assert(fabs(area/20000.-bias*.001)<.02);
  return area/20000.;
}
static void fractional_slots(void){
  for(int sign=-1;sign<=1;sign+=2){
    double a=averaged_bias((int16_t)(sign*250),1,0,100);
    double b=averaged_bias((int16_t)(sign*250),10,0,10);
    assert(fabs(a-b)<.001);
    double c=averaged_bias((int16_t)(sign*250),1,150,100);
    double d=averaged_bias((int16_t)(sign*250),10,150,10);
    assert(fabs(c-d)<.001);
  }
  puts("PASS +/-0.25 RPM physical mean, 1/10ms ticks, 10/100ms commands and delayed application");
}
static void pending_and_signs(void){
  Fixture f;MQSegment s={MQ_KIND_MOVE_GUIDED,10.f};setup(&f,0,&s,1);
  steer(&f,250);mq_tick(&f.q,0);assert(f.q.steer_residual==250 && f.motor.targets==1);
  f.motor.applied_delay=150;
  for(unsigned i=0;i<14;i++){
    step(&f,10);steer(&f,500);
    assert(f.q.steer_residual==250 && f.motor.targets==1);
  }
  step(&f,10);assert(f.q.steer_residual==750 && f.motor.targets==2);
  steer(&f,-250);assert(!f.q.steer_residual);
  steer(&f,0);assert(!f.q.steer_residual);
  steer(&f,2000);
  for(unsigned i=0;i<15;i++)step(&f,10);
  assert(f.motor.left==0 && f.motor.right==-2 && !f.q.steer_residual);
  puts("PASS pending target freezes residual; sign/zero reset; saturated trim never reverses or accelerates");
}
static void expiry_and_identity(void){
  Fixture f;MQSegment s={MQ_KIND_MOVE_GUIDED,10.f};setup(&f,0,&s,1);
  for(unsigned i=0;i<9;i++){step(&f,50);assert(f.motor.left==0 && f.motor.right==0);}
  assert(mq_steer(&f.q,2,0,250,450,450)==MQ_IDENTITY);
  assert(mq_steer(&f.q,1,1,250,450,450)==MQ_ORDER);
  assert(mq_steer(&f.q,1,0,2001,450,450)==MQ_INVALID);
  assert(mq_steer(&f.q,1,0,250,451,450)==MQ_STALE);
  source(&f,500);
  assert(mq_steer(&f.q,1,0,250,500,500)==MQ_EXPIRED);
  assert(f.q.first_error==MQ_ERR_STEER && f.motor.stops==1);
  assert(mq_steer(&f.q,1,0,250,500,500)==MQ_ORDER);

  setup(&f,0,&s,1);steer(&f,250);mq_tick(&f.q,0);
  for(unsigned i=0;i<9;i++){
    step(&f,50);
    assert(mq_steer(&f.q,1,0,250,0,f.now)==MQ_OK);
    assert(f.q.steer_basis_ms==0);
  }
  step(&f,50);assert(f.q.first_error==MQ_ERR_STEER && f.motor.stops==1);
  assert(!mq_steer_active(&f.q,500));
  assert(mq_steer(&f.q,1,0,250,500,500)==MQ_ORDER);
  puts("PASS zero until initial correction; fixed basis expires without late revival; identity/range/future rejected");
}
static void wrapping_and_stop(void){
  Fixture f;MQSegment s={MQ_KIND_MOVE_GUIDED,10.f};setup(&f,UINT32_MAX-200u,&s,1);
  steer(&f,-250);mq_tick(&f.q,f.now);
  for(unsigned i=0;i<12;i++){step(&f,50);steer(&f,-250);}
  assert(f.now<1000 && !f.motor.stops && f.q.state==MQ_MOVE);
  assert(mq_steer(&f.q,1,0,-250,f.now-100u,f.now)==MQ_STALE);
  mq_stop(&f.q,f.now);unsigned targets=f.motor.targets;
  assert(mq_steer(&f.q,1,0,-250,f.now,f.now)==MQ_ORDER);
  mq_tick(&f.q,f.now);assert(f.motor.targets==targets && f.q.state==MQ_FAILED && f.q.physical_stop_confirmed);
  puts("PASS measurement tick wrap, backwards basis rejection and STOP prevents late control");
}
static void next_segment_resets(void){
  Fixture f;MQSegment s[]={{MQ_KIND_MOVE_GUIDED,.03f},{MQ_KIND_MOVE_GUIDED,10.f}};
  setup(&f,0,s,2);steer(&f,250);mq_tick(&f.q,0);assert(f.q.steer_residual==250);
  for(unsigned i=0;i<100 && f.q.index==0;i++){
    step(&f,50);
    if(f.q.state==MQ_MOVE && f.q.index==0)steer(&f,250);
  }
  assert(f.q.index==1 && f.q.state==MQ_MOVE && !f.q.steer_valid && !f.q.steer_residual);
  assert(mq_steer(&f.q,1,0,250,f.now,f.now)==MQ_ORDER);
  step(&f,50);assert(f.motor.left==0 && f.motor.right==0);
  steer(&f,-250);
  for(unsigned i=0;i<2;i++)step(&f,50);
  assert(f.motor.left>0 && f.motor.right<=0);
  puts("PASS measured distance and five-sample barrier preserved; each guided segment waits for its own control");
}
int main(void){fractional_slots();pending_and_signs();expiry_and_identity();wrapping_and_stop();next_segment_resets();return 0;}

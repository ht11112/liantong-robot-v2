#ifndef MQ_HOST_TEST
#error "Host test must never be compiled into firmware"
#endif
#include "wire.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {MQDriveView drive;unsigned arms,stops,tx_count;bool busy;uint8_t tx[110];uint32_t clock;} Fake;
static void arm(void*p,uint32_t n){(void)n;Fake*f=p;f->arms++;f->drive.state=MQ_DRIVE_ENABLED;}
static uint32_t target(void*p,int16_t l,int16_t r,uint32_t n){(void)p;(void)l;(void)r;(void)n;return 1;}
static void stop(void*p,uint32_t n){(void)n;Fake*f=p;f->stops++;f->drive.state=MQ_DRIVE_STOPPED;f->drive.stop_verified=true;}
static MQDriveView view(void*p){return ((Fake*)p)->drive;}
static bool ready(void*p){return !((Fake*)p)->busy;}
static bool tx(void*p,const uint8_t*b,uint16_t n,uint32_t tick){(void)tick;Fake*f=p;assert(n==110);memcpy(f->tx,b,n);f->tx_count++;return true;}
static uint32_t advancing_clock(void*p){Fake*f=p;return f->clock++;}
static const MQDriveOps dops={arm,target,stop,view};
static const WireOps wops={ready,tx,NULL};
static void p16(uint8_t*p,uint16_t n){p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);}
static void p32(uint8_t*p,uint32_t n){p16(p,(uint16_t)n);p16(p+2,(uint16_t)(n>>16));}
static void p64(uint8_t*p,uint64_t n){p32(p,(uint32_t)n);p32(p+4,(uint32_t)(n>>32));}
static void pf(uint8_t*p,float v){uint32_t n;memcpy(&n,&v,4);p32(p,n);}
static unsigned command(uint8_t*b,uint8_t op,uint64_t boot,uint32_t epoch,uint32_t req,uint32_t grant,uint64_t task){
  unsigned n=(op==WIRE_BEGIN||op==WIRE_APPEND)?46:36;
  memset(b,0,64);b[0]=0xa5;b[1]=0x5a;b[2]=10;b[3]=op;p16(b+4,(uint16_t)(n-8));
  p64(b+6,boot);p32(b+14,epoch);p32(b+18,req);p32(b+22,grant);p64(b+26,task);return n;
}
static void seal(uint8_t*b,unsigned n){p16(b+n-2,wire_crc16(b+2,n-4));}
static void send(Wire*w,uint8_t*b,unsigned n,uint32_t at){seal(b,n);for(unsigned i=0;i<n;i++)wire_rx_byte(w,b[i],at);wire_poll(w,at);}
static void init(MQ*q,Wire*w,Fake*f){
  memset(f,0,sizeof(*f));f->drive.state=MQ_DRIVE_STOPPED;f->drive.stop_verified=f->drive.guard_valid=true;
  mq_init(q,0x1122334455667788ull,&dops,f);MQSample s={.sequence=1,.tick_ms=0,.valid=true};mq_sample(q,&s);
  wire_init(w,q,&wops,f,0);
}
static void upload_and_replay(void){
  MQ q;Wire w;Fake f;uint8_t b[64];unsigned n;MQSegment s={1,.5f};init(&q,&w,&f);
  wire_poll(&w,0);assert(f.tx_count==1&&w.pub_sequence==1);
  n=command(b,WIRE_QUERY,q.boot_id,1,1,1,0);send(&w,b,n,14);assert(w.ack_result==MQ_OK);
  n=command(b,WIRE_BEGIN,q.boot_id,1,2,1,1);p16(b+34,1);p32(b+36,mq_segments_crc(&s,1));p32(b+40,1200000);
  send(&w,b,n,20);assert(w.ack_result==MQ_BUSY&&q.state==MQ_IDLE);
  wire_poll(&w,50);p32(b+22,2);send(&w,b,n,64);assert(q.state==MQ_LOADING&&w.ack_result==MQ_OK);
  n=command(b,WIRE_APPEND,q.boot_id,1,3,2,1);p16(b+34,0);p32(b+36,1);pf(b+40,.5f);send(&w,b,n,65);
  assert(w.ack_result==MQ_BUSY&&q.received==0);
  wire_poll(&w,100);p32(b+22,3);send(&w,b,n,114);assert(q.received==1&&w.ack_result==MQ_OK);
  wire_poll(&w,150);n=command(b,WIRE_COMMIT,q.boot_id,1,4,4,1);send(&w,b,n,164);assert(q.committed&&q.state==MQ_READY);
  wire_poll(&w,200);n=command(b,WIRE_START,q.boot_id,1,5,5,1);send(&w,b,n,214);assert(f.arms==1&&q.has_started);
  /* Same request in a later/stale slot ACKs but never dispatches START twice. */
  send(&w,b,n,240);assert(f.arms==1&&w.ack_result==MQ_OK);
  wire_poll(&w,250);p32(b+22,6);send(&w,b,n,264);assert(f.arms==1&&w.ack_result==MQ_OK);
  /* Changed semantic bytes under same ID are rejected, not treated as duplicate. */
  b[3]=WIRE_RESET;send(&w,b,n,270);assert(w.ack_result==MQ_IDENTITY&&q.odom.epoch==1);
  /* STOP works outside a grant and across old epochs. CRC and boot still apply. */
  n=command(b,WIRE_STOP,q.boot_id,999,6,0,0);send(&w,b,n,300);assert(f.stops==1&&w.ack_result==MQ_OK);
  send(&w,b,n,310);assert(f.stops==1&&w.ack_result==MQ_OK);
  n=command(b,WIRE_QUERY,q.boot_id,1,1,6,0);send(&w,b,n,320);assert(w.ack_result==MQ_ORDER&&f.stops==1);
  wire_poll(&w,350);assert(w.tx[42]==1&&w.tx[104]==6); /* Reject ACK never lowers accepted watermark. */
  puts("PASS real C BEGIN APPEND COMMIT START, one request/grant, semantic replay and out-of-slot STOP");
}
static void invalid_frames(void){
  MQ q;Wire w;Fake f;uint8_t b[64];unsigned n;init(&q,&w,&f);wire_poll(&w,0);
  n=command(b,WIRE_QUERY,q.boot_id,1,1,1,0);send(&w,b,n,13);assert(w.ack_result==MQ_EXPIRED&&!w.has_request);
  send(&w,b,n,38);assert(w.ack_result==MQ_EXPIRED&&!w.has_request);
  wire_poll(&w,50);p32(b+22,2);send(&w,b,n,64);assert(w.ack_result==MQ_OK&&w.has_request);
  n=command(b,WIRE_STOP,q.boot_id+1,1,2,0,0);send(&w,b,n,65);assert(w.ack_result==MQ_IDENTITY&&f.stops==0);
  n=command(b,WIRE_RESET,q.boot_id,9,2,2,0);send(&w,b,n,66);assert(w.ack_result==MQ_IDENTITY&&q.odom.epoch==1);
  n=command(b,WIRE_STOP,q.boot_id,1,2,0,0);seal(b,n);b[n-1]^=1;
  for(unsigned i=0;i<n;i++){wire_rx_byte(&w,b[i],67);}wire_poll(&w,67);assert(f.stops==0);
  wire_rx_byte(&w,0xa5,68);wire_rx_byte(&w,0x5a,68);wire_rx_byte(&w,10,68);wire_rx_byte(&w,1,68);
  wire_rx_byte(&w,255,68);wire_rx_byte(&w,255,68);wire_poll(&w,68);assert(w.used==0);
  n=command(b,WIRE_QUERY,q.boot_id,1,2,2,0);send(&w,b,n,70);assert(w.ack_result==MQ_BUSY&&f.stops==0);
  puts("PASS C CRC, length, wrong boot/epoch, closed grant and retry on next grant");
}
static void timing_and_fault(void){
  MQ q;Wire w;Fake f;init(&q,&w,&f);wire_poll(&w,3);assert(f.tx_count==0);
  wire_poll(&w,50);assert(f.tx_count==1);uint32_t source=q.odom.measurement_tick_ms,seq=q.odom.sample_sequence;
  wire_poll(&w,100);assert(f.tx_count==2&&source==q.odom.measurement_tick_ms&&seq==q.odom.sample_sequence);
  /* No catch-up, and no mutation of old measured time when telemetry repeats. */
  wire_poll(&w,249);assert(f.tx_count==2);wire_poll(&w,250);assert(f.tx_count==3);
  f.busy=true;uint8_t copy[110];memcpy(copy,w.tx,110);wire_poll(&w,300);assert(f.tx_count==3&&!memcmp(copy,w.tx,110));
  f.busy=false;wire_rx_byte(&w,0xa5,350);wire_poll(&w,350);wire_poll(&w,352);assert(f.tx_count==3);
  wire_poll(&w,400);assert(f.tx_count==4&&w.used==0);
  assert(!(w.tx[48]&WIRE_ODOM_VALID)); /* Idle stale samples are also marked stale. */
  wire_rx_byte(&w,0,449);wire_poll(&w,450);assert(f.tx_count==4);wire_poll(&w,451);assert(f.tx_count==5);
  assert(w.tx[100]==13&&w.tx[102]==37); /* grant follows fixed frame, not delayed TX */
  for(unsigned i=0;i<129;i++){wire_rx_byte(&w,0,460);}wire_poll(&w,460);assert(q.first_error==MQ_ERR_UART&&f.stops==1);
  wire_poll(&w,461);assert(f.stops==1);
  puts("PASS C 20Hz slot pacing, busy/partial/quiet guards, old source preserved, UART fault once");
}
static void changing_transmit_clock(void){
  MQ q;Wire w;Fake f;init(&q,&w,&f);w.io.now_ms=advancing_clock;
  wire_poll(&w,0);assert(f.tx_count==0); /* Preparing a frame crossed an integer tick. */
  wire_poll(&w,0);assert(f.tx_count==0); /* Fresh phase 2 is already too late. */
  puts("PASS C fresh pre-transmit clock rejects a frame prepared across the slot boundary");
}
static void critical_transport_stays_locked(void){
  for(unsigned mode=0;mode<2;mode++){
    MQ q;Wire w;Fake f;uint8_t b[64];unsigned n;MQSegment s={1,.5f};init(&q,&w,&f);wire_poll(&w,0);
    if(mode){wire_transport_error(&w);}
    else {for(unsigned i=0;i<129;i++){wire_rx_byte(&w,0,10);}}
    wire_poll(&w,10);assert(q.first_error==MQ_ERR_UART&&f.stops==1);
    mq_tick(&q,11);assert(q.state==MQ_FAILED&&q.physical_stop_confirmed);
    /* Healthy stopped drive and still-fresh CAN data must not unlock the wire. */
    assert(q.odom.valid&&f.drive.guard_valid&&f.drive.stop_verified&&!f.drive.error);
    wire_poll(&w,50);assert(w.tx[48]&WIRE_RX_FAULT);
    n=command(b,WIRE_BEGIN,q.boot_id,1,1,2,1);p16(b+34,1);p32(b+36,mq_segments_crc(&s,1));p32(b+40,1200000);
    send(&w,b,n,64);assert(w.ack_result==MQ_UNHEALTHY&&q.task_id==0&&q.count==0&&q.first_error==MQ_ERR_UART);
    n=command(b,WIRE_START,q.boot_id,1,1,2,1);send(&w,b,n,65);assert(w.ack_result==MQ_UNHEALTHY&&f.arms==0);
    n=command(b,WIRE_RESET,q.boot_id,1,1,2,0);send(&w,b,n,66);assert(w.ack_result==MQ_UNHEALTHY&&q.odom.epoch==1);
    n=command(b,WIRE_APPEND,q.boot_id,1,1,2,1);p32(b+36,1);pf(b+40,.5f);send(&w,b,n,67);assert(w.ack_result==MQ_UNHEALTHY);
    n=command(b,WIRE_COMMIT,q.boot_id,1,1,2,1);send(&w,b,n,68);assert(w.ack_result==MQ_UNHEALTHY);
    n=command(b,WIRE_QUERY,q.boot_id,1,1,2,0);send(&w,b,n,72);assert(w.ack_result==MQ_OK&&q.first_error==MQ_ERR_UART);
    n=command(b,WIRE_STOP,q.boot_id,999,2,0,0);send(&w,b,n,80);assert(w.ack_result==MQ_OK&&f.stops==2);
    mq_tick(&q,81);wire_poll(&w,100);
    n=command(b,WIRE_BEGIN,q.boot_id,1,3,3,1);p16(b+34,1);p32(b+36,mq_segments_crc(&s,1));p32(b+40,1200000);
    send(&w,b,n,114);assert(w.ack_result==MQ_UNHEALTHY&&q.first_error==MQ_ERR_UART&&f.arms==0);
  }
  puts("PASS C UART and ring-overflow boot latch survives physical STOP, new task, START and RESET");
}
static void hex(const uint8_t*b,unsigned n){for(unsigned i=0;i<n;i++)printf("%02x",b[i]);}
static void golden(void){
  MQ q;Wire w;Fake f;uint8_t t[110],b[64],a[64];MQSegment seg[]={{1,.5f},{2,-1.5f}};init(&q,&w,&f);
  q.odom.epoch=7;q.odom.sample_sequence=42;q.odom.measurement_tick_ms=9950;q.task_id=0x0102030405060708ull;
  q.state=MQ_TURN;q.index=1;q.count=2;q.progress=.25f;q.odom.x=1.25f;q.odom.y=-.5f;q.odom.yaw=-1.f;
  q.odom.v=.02f;q.odom.w=-.05f;q.odom.position_left=0xfffffffe;q.odom.position_right=1234;
  q.odom.left_tenths_rpm=7;q.odom.right_tenths_rpm=-9;q.first_error=3;q.first_drive_error=0x12345678;
  q.first_error_tick_ms=9000;q.committed=q.has_started=true;
  wire_telemetry(&q,t,9,10000,5,MQ_ORDER,true,false,14,38,32);
  unsigned nb=command(b,WIRE_BEGIN,q.boot_id,7,33,9,q.task_id+1);p16(b+34,2);p32(b+36,mq_segments_crc(seg,2));p32(b+40,1200000);seal(b,nb);
  unsigned na=command(a,WIRE_APPEND,q.boot_id,7,34,10,q.task_id+1);p16(a+34,1);p32(a+36,2);pf(a+40,-1.5f);seal(a,na);
  printf("GOLDEN{\"telemetry\":\"");hex(t,110);printf("\",\"begin\":\"");hex(b,nb);
  printf("\",\"append\":\"");hex(a,na);printf("\",\"queue_crc32\":%u}\n",(unsigned)mq_segments_crc(seg,2));
}
int main(void){
  assert(wire_crc16((const uint8_t*)"123456789",9)==0x29b1);
  upload_and_replay();invalid_frames();timing_and_fault();changing_transmit_clock();critical_transport_stays_locked();golden();return 0;
}

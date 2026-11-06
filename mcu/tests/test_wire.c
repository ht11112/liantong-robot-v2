#ifndef MQ_HOST_TEST
#error "Host test must never be compiled into firmware"
#endif
#include "wire.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  MQDriveView drive;
  unsigned arms,stops,tx_attempts,tx_count;
  bool busy,tx_rejected;
  uint8_t tx[110];
  uint32_t clock;
} Fake;
static void arm(void*p,uint32_t n){(void)n;Fake*f=p;f->arms++;f->drive.state=MQ_DRIVE_ENABLED;}
static uint32_t target(void*p,int16_t l,int16_t r,uint32_t n){(void)p;(void)l;(void)r;(void)n;return 1;}
static void stop(void*p,uint32_t n){(void)n;Fake*f=p;f->stops++;f->drive.state=MQ_DRIVE_STOPPED;f->drive.stop_verified=true;}
static MQDriveView view(void*p){return ((Fake*)p)->drive;}
static bool ready(void*p){return !((Fake*)p)->busy;}
static bool tx(void*p,const uint8_t*b,uint16_t n,uint32_t tick){
  (void)tick;Fake*f=p;assert(n==110);f->tx_attempts++;
  if(f->tx_rejected)return false;
  memcpy(f->tx,b,n);f->tx_count++;return true;
}
static uint32_t advancing_clock(void*p){Fake*f=p;return f->clock++;}
static const MQDriveOps dops={arm,target,stop,view};
static const WireOps wops={ready,tx,NULL,NULL,NULL,NULL};
static void p16(uint8_t*p,uint16_t n){p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);}
static void p32(uint8_t*p,uint32_t n){p16(p,(uint16_t)n);p16(p+2,(uint16_t)(n>>16));}
static void p64(uint8_t*p,uint64_t n){p32(p,(uint32_t)n);p32(p+4,(uint32_t)(n>>32));}
static uint16_t r16(const uint8_t*p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static uint32_t r32(const uint8_t*p){return (uint32_t)r16(p)|((uint32_t)r16(p+2)<<16);}
static void pf(uint8_t*p,float v){uint32_t n;memcpy(&n,&v,4);p32(p,n);}
static unsigned command(uint8_t*b,uint8_t op,uint64_t boot,uint32_t epoch,uint32_t req,uint64_t task){
  unsigned n=(op==WIRE_BEGIN||op==WIRE_APPEND)?46:36;
  memset(b,0,64);b[0]=0xa5;b[1]=0x5a;b[2]=WIRE_VERSION;b[3]=op;p16(b+4,(uint16_t)(n-8));
  p64(b+6,boot);p32(b+14,epoch);p32(b+18,req);p64(b+26,task);return n;
}
static void seal(uint8_t*b,unsigned n){p16(b+n-2,wire_crc16(b+2,n-4));}
static void receive(Wire*w,const uint8_t*b,unsigned n,uint32_t at){
  for(unsigned i=0;i<n;i++)wire_rx_byte(w,b[i],at);
  wire_poll(w,at);
}
static void send(Wire*w,uint8_t*b,unsigned n,uint32_t at){
  seal(b,n);receive(w,b,n,at);wire_poll(w,at+2);
}
static void init(MQ*q,Wire*w,Fake*f){
  memset(f,0,sizeof(*f));f->drive.state=MQ_DRIVE_STOPPED;f->drive.stop_verified=f->drive.guard_valid=true;
  mq_init(q,0x1122334455667788ull,&dops,f);MQSample s={.sequence=1,.tick_ms=0,.valid=true};mq_sample(q,&s);
  wire_init(w,q,&wops,f,0);
}
static void ack(const Fake*f,uint32_t request,MQResult result,uint32_t watermark){
  const uint8_t*b=f->tx;
  assert(b[0]==0xa5&&b[1]==0x5a&&b[2]==WIRE_VERSION&&b[3]==WIRE_TELEMETRY);
  assert(r16(b+4)==102&&r32(b+42)==request&&b[46]==result&&(r16(b+48)&WIRE_ACK_VALID));
  assert(r32(b+100)<=MQ_LEASE_MS&&r32(b+104)==watermark&&r16(b+108)==wire_crc16(b+2,106));
}
static void discovery_and_legacy(void){
  MQ q;Wire w;Fake f;uint8_t b[64],old[64];init(&q,&w,&f);
  wire_poll(&w,0);wire_poll(&w,50);wire_poll(&w,100);assert(!f.tx_count);
  unsigned n=command(b,WIRE_QUERY,0,0,0,0);
  p32(b+14,1);send(&w,b,n,110);assert(!f.tx_count);
  p32(b+14,0);p32(b+22,1);send(&w,b,n,120);assert(!f.tx_count);
  p32(b+22,0);send(&w,b,n,130);ack(&f,0,MQ_OK,0);
  assert(r32(f.tx+6)==0x55667788u&&r32(f.tx+10)==0x11223344u&&r32(f.tx+14)==1);
  assert(f.tx_count==1&&r32(f.tx+18)==1&&r32(f.tx+22)==132&&!w.has_request);
  wire_poll(&w,250);assert(f.tx_count==1);
  send(&w,b,n,300);ack(&f,0,MQ_OK,0);
  assert(f.tx_count==2&&r32(f.tx+18)==2&&r32(f.tx+22)==302);
  assert(r32(f.tx+26)==1&&r32(f.tx+30)==0&&!(r16(f.tx+48)&WIRE_ODOM_VALID));
  n=command(b,WIRE_STOP,q.boot_id,1,1,0);b[2]=10;send(&w,b,n,310);
  assert(f.tx_count==2&&!f.stops&&!w.has_request);
  b[2]=11;send(&w,b,n,315);
  assert(f.tx_count==2&&!f.stops&&!w.has_request);
  b[2]=WIRE_VERSION;seal(b,n);
  memset(old,0,sizeof(old));old[0]=0xa5;old[1]=0x5a;old[2]=10;old[3]=WIRE_APPEND;
  p16(old+4,sizeof(old)-8);memcpy(old+8,b,n);send(&w,old,sizeof(old),320);
  assert(f.tx_count==2&&!f.stops&&!w.has_request);
  old[2]=11;send(&w,old,sizeof(old),330);
  assert(f.tx_count==2&&!f.stops&&!w.has_request);
  puts("PASS no unsolicited frames, exact discovery reply, reply-time snapshot and full legacy-frame rejection");
}
static void upload_and_replay(void){
  MQ q;Wire w;Fake f;uint8_t b[64];unsigned n;MQSegment s={1,.5f};init(&q,&w,&f);
  n=command(b,WIRE_BEGIN,q.boot_id,1,1,1);p16(b+34,1);p32(b+36,mq_segments_crc(&s,1));p32(b+40,MQ_LEASE_MS);
  send(&w,b,n,20);ack(&f,1,MQ_OK,1);assert(q.state==MQ_LOADING);
  n=command(b,WIRE_APPEND,q.boot_id,1,2,1);p16(b+34,0);p32(b+36,1);pf(b+40,.5f);
  send(&w,b,n,40);ack(&f,2,MQ_OK,2);assert(q.received==1);
  n=command(b,WIRE_COMMIT,q.boot_id,1,3,1);send(&w,b,n,60);
  ack(&f,3,MQ_OK,3);assert(q.committed&&q.state==MQ_READY);
  n=command(b,WIRE_START,q.boot_id,1,4,1);send(&w,b,n,80);
  ack(&f,4,MQ_OK,4);assert(f.arms==1&&q.has_started&&r32(f.tx+100)==MQ_LEASE_MS-2u);
  send(&w,b,n,100);ack(&f,4,MQ_OK,4);assert(f.arms==1&&r32(f.tx+18)==5);
  b[3]=WIRE_RESET;send(&w,b,n,120);ack(&f,4,MQ_IDENTITY,4);assert(q.odom.epoch==1);
  n=command(b,WIRE_STOP,q.boot_id,999,5,0);send(&w,b,n,140);
  ack(&f,5,MQ_OK,5);assert(f.stops==1&&r32(f.tx+100)==0);
  send(&w,b,n,160);ack(&f,5,MQ_OK,5);assert(f.stops==1);
  n=command(b,WIRE_QUERY,q.boot_id,1,1,0);send(&w,b,n,180);ack(&f,1,MQ_ORDER,5);
  n=command(b,WIRE_QUERY,0,0,0,0);send(&w,b,n,200);ack(&f,0,MQ_OK,5);
  assert(f.tx_count==10);
  puts("PASS BEGIN APPEND COMMIT START, byte-exact replay, unchanged STOP epoch bypass and discovery watermark");
}
static void renewal_and_replay(void){
  MQ q;Wire w;Fake f;uint8_t b[64];MQSegment s={MQ_KIND_MOVE,1.f};init(&q,&w,&f);
  assert(mq_begin(&q,1,1,1,mq_segments_crc(&s,1),MQ_LEASE_MS)==MQ_OK);
  assert(mq_append(&q,1,0,s)==MQ_OK&&mq_commit(&q,1)==MQ_OK);
  unsigned n=command(b,WIRE_RENEW,q.boot_id,1,1,1);send(&w,b,n,10);
  ack(&f,1,MQ_ORDER,1);assert(!f.arms&&r32(f.tx+100)==0);
  assert(mq_start(&q,1,20)==MQ_OK);
  n=command(b,WIRE_RENEW,q.boot_id+1,1,2,1);send(&w,b,n,30);
  ack(&f,2,MQ_IDENTITY,1);
  n=command(b,WIRE_RENEW,q.boot_id,2,2,1);send(&w,b,n,40);
  ack(&f,2,MQ_IDENTITY,1);
  n=command(b,WIRE_RENEW,q.boot_id,1,2,2);send(&w,b,n,50);
  ack(&f,2,MQ_IDENTITY,2);assert(q.lease_started_ms==20);
  n=command(b,WIRE_RENEW,q.boot_id,1,3,1);send(&w,b,n,60);
  ack(&f,3,MQ_OK,3);assert(q.lease_started_ms==60&&r32(f.tx+100)==MQ_LEASE_MS-2u);
  send(&w,b,n,80);ack(&f,3,MQ_OK,3);
  assert(q.lease_started_ms==60&&r32(f.tx+100)==MQ_LEASE_MS-22u);
  f.drive.guard_valid=false;
  n=command(b,WIRE_RENEW,q.boot_id,1,4,1);send(&w,b,n,90);
  ack(&f,4,MQ_UNHEALTHY,4);assert(q.lease_started_ms==60);
  n=command(b,WIRE_STOP,q.boot_id,1,5,0);send(&w,b,n,100);
  ack(&f,5,MQ_OK,5);assert(f.stops==1);
  n=command(b,WIRE_RENEW,q.boot_id,1,6,1);send(&w,b,n,110);
  ack(&f,6,MQ_ORDER,6);assert(q.lease_started_ms==60&&r32(f.tx+100)==0&&f.arms==1);
  puts("PASS RENEW binds boot, epoch and task; replay does not extend lease; STOP ends renewal");
}
static void invalid_frames(void){
  MQ q;Wire w;Fake f;uint8_t b[64];init(&q,&w,&f);
  unsigned n=command(b,WIRE_QUERY,q.boot_id,1,1,0);p32(b+22,1);send(&w,b,n,10);
  ack(&f,1,MQ_INVALID,0);assert(!w.has_request);
  n=command(b,WIRE_STOP,q.boot_id+1,1,1,0);send(&w,b,n,20);
  ack(&f,1,MQ_IDENTITY,0);assert(!f.stops);
  n=command(b,WIRE_RESET,q.boot_id,9,1,0);send(&w,b,n,30);
  ack(&f,1,MQ_IDENTITY,0);assert(q.odom.epoch==1);
  n=command(b,WIRE_START,q.boot_id,1,1,0);send(&w,b,n,40);
  assert(r32(f.tx+104)==1&&f.tx[46]!=MQ_OK&&w.has_request&&!f.arms);
  n=command(b,WIRE_STOP,q.boot_id,1,2,0);seal(b,n);b[n-1]^=1;receive(&w,b,n,50);wire_poll(&w,52);
  assert(f.tx_count==4&&!f.stops);
  b[0]=0xa5;b[1]=0x5a;b[2]=WIRE_VERSION;b[3]=WIRE_STOP;p16(b+4,255);
  receive(&w,b,6,60);assert(!w.used&&f.tx_count==4);
  puts("PASS CRC/length checks, reserved field, identity, epoch and dispatch-result watermark");
}
static void pending_stop_and_expiry(void){
  MQ q;Wire w;Fake f;uint8_t b[64];init(&q,&w,&f);
  unsigned n=command(b,WIRE_QUERY,q.boot_id,1,1,0);seal(b,n);receive(&w,b,n,10);
  assert(w.reply_pending&&!f.tx_count&&w.last_request==1);
  f.busy=true;n=command(b,WIRE_START,q.boot_id,1,2,1);seal(b,n);receive(&w,b,n,11);
  assert(w.ack_request==1&&!f.arms&&w.last_request==1);
  n=command(b,WIRE_STOP,q.boot_id+1,1,2,0);seal(b,n);receive(&w,b,n,12);
  assert(w.ack_request==1&&!f.stops);
  n=command(b,WIRE_STOP,q.boot_id,999,2,0);seal(b,n);receive(&w,b,n,13);
  assert(w.reply_pending&&w.ack_request==2&&f.stops==1);
  f.busy=false;wire_poll(&w,14);assert(!f.tx_count);
  f.tx_rejected=true;wire_poll(&w,15);assert(f.tx_attempts==1&&w.reply_pending&&!w.pub_sequence);
  f.busy=true;wire_poll(&w,63);assert(!w.reply_pending&&!f.tx_count);
  f.busy=false;f.tx_rejected=false;wire_poll(&w,64);assert(!f.tx_count);
  send(&w,b,n,70);ack(&f,2,MQ_OK,2);assert(f.tx_count==1&&w.pub_sequence==1&&f.stops==1);
  p32(b+14,998);seal(b,n);receive(&w,b,n,80);assert(w.ack_result==MQ_IDENTITY);
  p32(b+14,999);seal(b,n);receive(&w,b,n,81);assert(w.ack_result==MQ_OK);
  wire_poll(&w,83);ack(&f,2,MQ_OK,2);assert(f.tx_count==2&&f.stops==1);
  wire_poll(&w,125);assert(f.tx_count==2);
  puts("PASS pending reply bound, STOP priority, quiet TX, failed launch and byte-identical replay");
}
static void changing_transmit_clock(void){
  MQ q;Wire w;Fake f;uint8_t b[64];init(&q,&w,&f);
  unsigned n=command(b,WIRE_QUERY,0,0,0,0);seal(b,n);receive(&w,b,n,10);
  f.clock=12;w.io.now_ms=advancing_clock;wire_poll(&w,12);
  assert(w.reply_pending&&!f.tx_count&&!f.tx_attempts);
  w.io.now_ms=NULL;wire_poll(&w,14);ack(&f,0,MQ_OK,0);
  assert(f.tx_count==1&&r32(f.tx+22)==14);
  puts("PASS reply rebuilt on clock change and only counts committed transmit");
}
static void critical_transport_stays_locked(void){
  for(unsigned mode=0;mode<2;mode++){
    MQ q;Wire w;Fake f;uint8_t b[64];MQSegment s={1,.5f};init(&q,&w,&f);
    if(mode)wire_transport_error(&w);
    else for(unsigned i=0;i<129;i++)wire_rx_byte(&w,0,10);
    wire_poll(&w,10);assert(q.first_error==MQ_ERR_UART&&f.stops==1&&!f.tx_count);
    mq_tick(&q,11);assert(q.state==MQ_FAILED&&q.physical_stop_confirmed);
    assert(q.odom.valid&&f.drive.guard_valid&&f.drive.stop_verified&&!f.drive.error);
    unsigned n=command(b,WIRE_BEGIN,q.boot_id,1,1,1);p16(b+34,1);p32(b+36,mq_segments_crc(&s,1));p32(b+40,MQ_LEASE_MS);
    send(&w,b,n,20);ack(&f,1,MQ_UNHEALTHY,0);assert(q.task_id==0&&q.count==0);
    n=command(b,WIRE_START,q.boot_id,1,1,1);send(&w,b,n,30);ack(&f,1,MQ_UNHEALTHY,0);assert(!f.arms);
    n=command(b,WIRE_RESET,q.boot_id,1,1,0);send(&w,b,n,40);ack(&f,1,MQ_UNHEALTHY,0);assert(q.odom.epoch==1);
    n=command(b,WIRE_APPEND,q.boot_id,1,1,1);p16(b+34,0);p32(b+36,1);pf(b+40,.5f);
    send(&w,b,n,50);ack(&f,1,MQ_UNHEALTHY,0);
    n=command(b,WIRE_COMMIT,q.boot_id,1,1,1);send(&w,b,n,60);ack(&f,1,MQ_UNHEALTHY,0);
    n=command(b,WIRE_QUERY,q.boot_id,1,1,0);send(&w,b,n,70);ack(&f,1,MQ_OK,1);
    assert((r16(f.tx+48)&WIRE_RX_FAULT)&&r16(f.tx+90)==MQ_ERR_UART);
    n=command(b,WIRE_STOP,q.boot_id,999,2,0);send(&w,b,n,80);ack(&f,2,MQ_OK,2);assert(f.stops==2);
    mq_tick(&q,83);wire_poll(&w,100);assert(f.tx_count==7);
  }
  puts("PASS UART and RX-overflow latch stops the drive, no unsolicited fault TX, QUERY and STOP retained");
}
static void hex(const uint8_t*b,unsigned n){for(unsigned i=0;i<n;i++)printf("%02x",b[i]);}
static void golden(void){
  MQ q;Wire w;Fake f;uint8_t t[110],b[64],a[64],query[64];MQSegment seg[]={{1,.5f},{2,-1.5f}};init(&q,&w,&f);
  q.odom.epoch=7;q.odom.sample_sequence=42;q.odom.measurement_tick_ms=9950;q.task_id=0x0102030405060708ull;
  q.state=MQ_TURN;q.index=1;q.count=2;q.progress=.25f;q.odom.x=1.25f;q.odom.y=-.5f;q.odom.yaw=-1.f;
  q.odom.v=.02f;q.odom.w=-.05f;q.odom.position_left=0xfffffffe;q.odom.position_right=1234;
  q.odom.left_tenths_rpm=7;q.odom.right_tenths_rpm=-9;q.first_error=3;q.first_drive_error=0x12345678;
  q.first_error_tick_ms=9000;q.committed=q.has_started=true;q.lease_started_ms=9500;
  wire_telemetry(&q,t,9,10000,5,MQ_ORDER,true,false,32);
  unsigned nq=command(query,WIRE_QUERY,0,0,0,0);seal(query,nq);
  unsigned nb=command(b,WIRE_BEGIN,q.boot_id,7,33,q.task_id+1);p16(b+34,2);p32(b+36,mq_segments_crc(seg,2));p32(b+40,MQ_LEASE_MS);seal(b,nb);
  unsigned na=command(a,WIRE_APPEND,q.boot_id,7,34,q.task_id+1);p16(a+34,1);p32(a+36,2);pf(a+40,-1.5f);seal(a,na);
  printf("GOLDEN{\"telemetry\":\"");hex(t,110);printf("\",\"query\":\"");hex(query,nq);printf("\",\"begin\":\"");hex(b,nb);
  printf("\",\"append\":\"");hex(a,na);printf("\",\"queue_crc32\":%u}\n",(unsigned)mq_segments_crc(seg,2));
}
int main(void){
  assert(wire_crc16((const uint8_t*)"123456789",9)==0x29b1);
  discovery_and_legacy();upload_and_replay();renewal_and_replay();invalid_frames();pending_stop_and_expiry();
  changing_transmit_clock();critical_transport_stays_locked();golden();return 0;
}

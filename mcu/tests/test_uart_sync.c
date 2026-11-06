/* Deterministic software schedules, not a UART/DE electrical model. */
#ifndef MQ_HOST_TEST
#error "Host test only"
#endif
#include "wire.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  MQ q;Wire w;MQDriveView drive;uint8_t tx[WIRE_TELEMETRY_SIZE];
  unsigned tx_count,arms,stops,enters,exits,pending_calls;
  uint32_t clock,mask;unsigned fault_on_enter;bool raw_pending,raw_error,busy;
  bool rx_at_enter,fault_at_enter,fault_at_ready,error_at_view,receive_after_exit,expect_view_unmasked;
} Sim;
static void arm(void *u,uint32_t n){(void)n;Sim*s=u;s->arms++;s->drive.state=MQ_DRIVE_ARMING;}
static uint32_t target(void*u,int16_t l,int16_t r,uint32_t n){(void)u;(void)l;(void)r;(void)n;assert(!"no targets in this simulation");return 0;}
static void stop(void*u,uint32_t n){(void)n;Sim*s=u;s->stops++;s->drive.state=MQ_DRIVE_STOPPING;}
static MQDriveView view(void*u){Sim*s=u;if(s->expect_view_unmasked)assert(s->mask==0);if(s->error_at_view){s->error_at_view=false;s->raw_error=true;}return s->drive;}
static bool ready(void*u){Sim*s=u;if(s->fault_at_ready){s->fault_at_ready=false;wire_transport_error(&s->w);}return !s->busy;}
static bool tx(void*u,const uint8_t*b,uint16_t n,uint32_t tick){
  Sim*s=u;assert(s->mask==1&&n==110&&tick==s->clock);s->busy=true;
  memcpy(s->tx,b,n);s->tx_count++;return true;
}
static uint32_t now(void*u){return ((Sim*)u)->clock;}
static uint32_t enter(void*u){
  Sim*s=u;uint32_t saved=s->mask;
  /* These represent ISR work immediately before the mask becomes active. */
  if(s->rx_at_enter){s->rx_at_enter=false;wire_rx_byte(&s->w,0xa5,s->clock);}
  if(s->fault_at_enter||s->fault_on_enter==s->enters+1u){s->fault_at_enter=false;wire_transport_error(&s->w);}
  s->mask=1;s->enters++;return saved;
}
static void leave(void*u,uint32_t saved){
  Sim*s=u;assert(s->mask==1);s->mask=saved;s->exits++;
  /* A pending IRQ is delivered only after restoring an unmasked state. */
  if(!saved&&s->receive_after_exit){s->receive_after_exit=false;wire_rx_byte(&s->w,0xa5,s->clock);}
}
static bool pending(void*u){
  Sim*s=u;assert(s->mask==1);s->pending_calls++;
  if(s->raw_error)wire_transport_error(&s->w);
  return s->raw_pending||s->raw_error;
}
static const MQDriveOps dops={arm,target,stop,view};
static const WireOps ops={ready,tx,now,enter,leave,pending};
static void init(Sim*s){
  memset(s,0,sizeof(*s));s->clock=50;s->drive.state=MQ_DRIVE_STOPPED;
  s->drive.stop_verified=s->drive.guard_valid=true;
  mq_init(&s->q,0x1122334455667788ull,&dops,s);
  MQSample sample={.sequence=1,.tick_ms=0,.valid=true};mq_sample(&s->q,&sample);
  wire_init(&s->w,&s->q,&ops,s,0);
}
static void p16(uint8_t*p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void p32(uint8_t*p,uint32_t v){p16(p,(uint16_t)v);p16(p+2,(uint16_t)(v>>16));}
static void p64(uint8_t*p,uint64_t v){p32(p,(uint32_t)v);p32(p+4,(uint32_t)(v>>32));}
static void command(Sim*s,uint8_t*b,uint8_t op,uint32_t request){
  memset(b,0,36);b[0]=0xa5;b[1]=0x5a;b[2]=WIRE_VERSION;b[3]=op;p16(b+4,28);
  p64(b+6,s->q.boot_id);p32(b+14,op==WIRE_STOP?999:1);p32(b+18,request);
  p64(b+26,s->q.task_id);
  p16(b+34,wire_crc16(b+2,32));
}
static void receive(Sim*s,const uint8_t*b,unsigned n){for(unsigned i=0;i<n;i++)wire_rx_byte(&s->w,b[i],s->clock);}
static void poll(Sim*s,uint32_t at){s->clock=at;wire_poll(&s->w,at);assert(s->enters==s->exits);}
static void crc_valid(const uint8_t*b){assert(wire_crc16(b+2,106)==(uint16_t)(b[108]|((uint16_t)b[109]<<8)));}
static void discovery(Sim*s){
  uint8_t b[36]={0xa5,0x5a,WIRE_VERSION,WIRE_QUERY,28};p16(b+34,wire_crc16(b+2,32));
  receive(s,b,sizeof(b));poll(s,s->clock);assert(s->w.reply_pending&&!s->tx_count);
}
static void rx_commit_schedules(void){
  Sim s;init(&s);discovery(&s);s.rx_at_enter=true;poll(&s,52);
  assert(!s.tx_count&&s.w.rx_write-s.w.rx_read==1&&s.w.reply_pending);
  init(&s);discovery(&s);s.raw_pending=true;poll(&s,52);assert(!s.tx_count&&!s.w.transport_fault);
  s.raw_pending=false;poll(&s,53);assert(s.tx_count==1);crc_valid(s.tx);
  init(&s);discovery(&s);s.mask=1;poll(&s,52);assert(s.tx_count==1&&s.mask==1);
  init(&s);discovery(&s);s.receive_after_exit=true;poll(&s,52);
  assert(s.tx_count==1&&s.w.rx_write-s.w.rx_read==1&&s.busy); /* retained after claim */
  puts("PASS sync RX before claim/raw pending blocks TX, saved mask restores, post-claim RX retained");
}
static void fault_snapshot_and_stop(void){
  Sim s;uint8_t b[36];init(&s);discovery(&s);s.fault_at_enter=true;poll(&s,52);
  assert(!s.tx_count&&s.w.reply_pending&&s.q.first_error==MQ_ERR_UART&&s.stops==1);
  poll(&s,53);assert(s.tx_count==1&&(s.tx[48]&WIRE_RX_FAULT));
  assert(s.tx[90]==MQ_ERR_UART&&s.tx[47]==MQ_STOPPING);crc_valid(s.tx);
  /* Fault was already latched at construction, but MQ had not yet seen it. */
  init(&s);discovery(&s);s.fault_at_ready=true;poll(&s,52);assert(!s.tx_count&&s.q.first_error==MQ_ERR_UART);
  poll(&s,53);assert(s.tx_count==1&&s.tx[90]==MQ_ERR_UART);crc_valid(s.tx);
  init(&s);wire_transport_error(&s.w);command(&s,b,WIRE_STOP,1);receive(&s,b,36);
  poll(&s,50);assert(s.w.has_request&&s.w.ack_result==MQ_OK&&s.w.last_request==1);
  assert(s.q.first_error==MQ_ERR_UART&&s.stops==2&&s.w.rx_read==s.w.rx_write);
  poll(&s,52);assert(s.tx_count==1&&s.tx[42]==1&&s.tx[46]==MQ_OK);crc_valid(s.tx);
  puts("PASS new fault rejects stale reply, rebuilt reply has fault state and CRC, buffered STOP survives");
}
static void start_fault_schedules(void){
  Sim s;uint8_t b[36];
  for(unsigned mode=0;mode<2;mode++){
    init(&s);
    s.q.task_id=1;s.q.committed=true;s.q.state=MQ_READY;s.q.epoch_at_upload=1;
    s.q.count=1;
    command(&s,b,WIRE_START,1);s.clock=64;receive(&s,b,36);
    if(mode)s.error_at_view=true;else s.fault_at_enter=true;
    poll(&s,64);
    assert(s.w.ack_result==MQ_UNHEALTHY&&s.q.first_error==MQ_ERR_UART);
    assert(s.q.state==MQ_STOPPING&&s.w.transport_fault&&s.w.last_request==(mode?1u:0u));
    assert(mode?s.arms==1:s.arms==0);
  }
  init(&s);
  s.q.task_id=1;s.q.committed=true;s.q.state=MQ_READY;s.q.epoch_at_upload=1;
  s.q.count=1;command(&s,b,WIRE_START,1);
  s.clock=64;receive(&s,b,36);poll(&s,64);assert(s.arms==1&&s.w.ack_result==MQ_OK);
  poll(&s,66);assert(s.tx_count==1&&s.tx[46]==MQ_OK);
  s.busy=false;s.clock=67;receive(&s,b,36);s.fault_on_enter=s.enters+2u;poll(&s,67);
  assert(s.arms==1&&s.w.ack_result==MQ_UNHEALTHY&&s.q.first_error==MQ_ERR_UART);
  init(&s);
  s.q.task_id=1;s.q.committed=true;s.q.state=MQ_READY;s.q.epoch_at_upload=1;
  s.q.count=1;command(&s,b,WIRE_START,1);
  receive(&s,b,36);poll(&s,50);assert(s.w.ack_result==MQ_OK&&s.w.reply_pending);
  wire_transport_error(&s.w);poll(&s,52);
  assert(s.tx_count==1&&s.tx[46]==MQ_UNHEALTHY&&s.tx[90]==MQ_ERR_UART&&s.stops==1);
  puts("PASS START rejects pre-claim fault and raw hardware error observed after memory-only arm");
}
static void echo_and_stop_frames(void){
  Sim s;uint8_t b[36],echo[110];init(&s);command(&s,b,WIRE_STOP,1);
  wire_telemetry(&s.q,echo,1,50,0,0,false,false,0);
  memcpy(echo+30,b,36);p16(echo+108,wire_crc16(echo+2,106));
  receive(&s,echo,110);poll(&s,50);assert(!s.stops&&!s.w.has_request&&!s.w.used);
  /* A STOP is allowed to span the TX claim/TC boundary if its bytes arrive. */
  s.busy=true;s.clock=60;receive(&s,b,10);poll(&s,60);assert(s.w.used==10);
  s.busy=false;s.clock=61;receive(&s,b+10,26);poll(&s,61);
  assert(s.stops==1&&s.w.has_request&&s.w.ack_result==MQ_OK);
  /* Replay remains idempotent despite arrival across busy boundaries. */
  s.clock=62;receive(&s,b,36);poll(&s,62);assert(s.stops==1);
  poll(&s,64);assert(s.tx_count==1&&s.tx[42]==1&&s.tx[46]==MQ_OK);
  init(&s);discovery(&s);poll(&s,52);assert(s.busy);
  memcpy(echo,s.w.tx,sizeof(echo));command(&s,b,WIRE_STOP,1);
  s.clock=60;receive(&s,b,36);poll(&s,60);
  assert(s.stops==1&&s.w.reply_pending&&!memcmp(s.w.tx,echo,sizeof(echo)));
  s.busy=false;poll(&s,62);assert(s.tx_count==2&&s.tx[42]==1&&s.tx[46]==MQ_OK);
  puts("PASS 110B echo envelope suppresses embedded command, intact STOP spans TX/TC, replay unchanged");
}
static void long_operation_fault(void){
  Sim s;uint8_t b[46]={0};init(&s);
  command(&s,b,WIRE_BEGIN,1);p16(b+4,38);p64(b+26,1);
  p16(b+34,1);p32(b+36,123);p32(b+40,MQ_LEASE_MS);p16(b+44,wire_crc16(b+2,42));
  s.clock=64;receive(&s,b,46);s.error_at_view=s.expect_view_unmasked=true;poll(&s,64);
  assert(s.w.ack_result==MQ_UNHEALTHY&&s.q.first_error==MQ_ERR_UART);
  assert(s.q.state==MQ_STOPPING&&s.q.critical_latched&&!s.arms&&s.stops==1);
  puts("PASS long BEGIN operation remains unmasked and post-operation raw fault overrides OK");
}
static void invalid_sync_config(void){
  Sim s;uint8_t b[36];
  for(unsigned mask=1;mask<7;mask++){
    init(&s);WireOps bad=ops;
    if(!(mask&1))bad.critical_enter=NULL;
    if(!(mask&2))bad.critical_exit=NULL;
    if(!(mask&4))bad.rx_pending=NULL;
    wire_init(&s.w,&s.q,&bad,&s,0);poll(&s,50);
    assert(s.w.transport_fault&&s.q.first_error==MQ_ERR_UART&&!s.tx_count&&!s.enters);
    command(&s,b,WIRE_STOP,1);s.clock=60;receive(&s,b,36);poll(&s,60);
    receive(&s,b,36);poll(&s,61);
    assert(s.w.ack_result==MQ_OK&&s.stops==2&&!s.enters&&!s.exits);
  }
  puts("PASS incomplete synchronization hooks fail closed without an unmatched critical entry");
}
int main(void){
  rx_commit_schedules();fault_snapshot_and_stop();start_fault_schedules();
  echo_and_stop_frames();long_operation_fault();invalid_sync_config();
  puts("LIMIT software schedules only; no FE cause, DE timing, collision delivery or hardware deadline proven");
  return 0;
}

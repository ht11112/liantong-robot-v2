#include "wire.h"
#include <string.h>

enum { RX_QUIET_MS=2, RX_FRAGMENT_TIMEOUT_MS=12, REPLY_TIMEOUT_MS=50 };

static uint16_t r16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static uint32_t r32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static uint64_t r64(const uint8_t *p){return r32(p)|((uint64_t)r32(p+4)<<32);}
static void p16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void p32(uint8_t *p,uint32_t v){p16(p,(uint16_t)v);p16(p+2,(uint16_t)(v>>16));}
static void p64(uint8_t *p,uint64_t v){p32(p,(uint32_t)v);p32(p+4,(uint32_t)(v>>32));}
static void pf(uint8_t *p,float f){uint32_t v;memcpy(&v,&f,4);p32(p,v);}
static float rf(const uint8_t *p){uint32_t v=r32(p);float f;memcpy(&f,&v,4);return f;}
uint16_t wire_crc16(const uint8_t *data,size_t length){
  uint16_t c=0xffff;size_t i;unsigned b;
  for(i=0;i<length;i++){c^=(uint16_t)data[i]<<8;for(b=0;b<8;b++)c=(uint16_t)((c&0x8000)?(c<<1)^0x1021:c<<1);}
  return c;
}
static void finish(uint8_t *out,size_t n,uint8_t kind){
  out[0]=0xa5;out[1]=0x5a;out[2]=WIRE_VERSION;out[3]=kind;
  p16(out+4,(uint16_t)(n-8));p16(out+n-2,wire_crc16(out+2,n-4));
}
static bool sync_valid(const Wire *w){
  return (!w->io.critical_enter&&!w->io.critical_exit&&!w->io.rx_pending)||
    (w->io.critical_enter&&w->io.critical_exit&&w->io.rx_pending);
}
static uint32_t enter(Wire *w){return sync_valid(w)&&w->io.critical_enter?w->io.critical_enter(w->user):0;}
static void leave(Wire *w,uint32_t saved){if(sync_valid(w)&&w->io.critical_exit)w->io.critical_exit(w->user,saved);}
static bool pending(Wire *w){return sync_valid(w)&&w->io.rx_pending&&w->io.rx_pending(w->user);}
size_t wire_telemetry(const MQ *q,uint8_t *o,uint32_t seq,uint32_t pub,
 uint32_t ack,uint8_t result,bool ack_valid,bool rx_fault,uint32_t watermark){
  uint16_t flags=0;
  if(q->odom.initialized)flags|=WIRE_INITIALIZED;
  if(q->odom.valid && (uint32_t)(pub-q->odom.measurement_tick_ms)<MQ_SAMPLE_TIMEOUT_MS)flags|=WIRE_ODOM_VALID;
  if(mq_is_executing(q))flags|=WIRE_EXECUTING;
  if(q->arrival_reached)flags|=WIRE_ARRIVAL;
  if(q->physical_stop_confirmed)flags|=WIRE_STOP_CONFIRMED;
  if(rx_fault)flags|=WIRE_RX_FAULT;
  if(q->committed)flags|=WIRE_COMMITTED;
  if(q->has_started)flags|=WIRE_STARTED;
  if(ack_valid)flags|=WIRE_ACK_VALID;
  p64(o+6,q->boot_id);p32(o+14,q->odom.epoch);p32(o+18,seq);p32(o+22,pub);
  p32(o+26,q->odom.sample_sequence);p32(o+30,q->odom.measurement_tick_ms);
  p64(o+34,q->task_id);p32(o+42,ack);o[46]=result;o[47]=(uint8_t)q->state;
  p16(o+48,flags);p16(o+50,q->index);p16(o+52,q->count);pf(o+54,q->progress);
  pf(o+58,q->odom.x);pf(o+62,q->odom.y);pf(o+66,q->odom.yaw);pf(o+70,q->odom.v);pf(o+74,q->odom.w);
  p32(o+78,q->odom.position_left);p32(o+82,q->odom.position_right);
  p16(o+86,(uint16_t)q->odom.left_tenths_rpm);p16(o+88,(uint16_t)q->odom.right_tenths_rpm);
  p16(o+90,(uint16_t)q->first_error);p32(o+92,q->first_drive_error);p32(o+96,q->first_error_tick_ms);
  p32(o+100,mq_lease_remaining_ms(q,pub));p32(o+104,watermark);
  finish(o,WIRE_TELEMETRY_SIZE,WIRE_TELEMETRY);return WIRE_TELEMETRY_SIZE;
}
void wire_init(Wire *w,MQ *q,const WireOps *io,void *user,uint32_t now){
  memset(w,0,sizeof(*w));w->queue=q;w->io=*io;w->user=user;
  w->last_rx_ms=now-RX_QUIET_MS;
  if(!sync_valid(w))w->transport_fault=true;
}
void wire_rx_byte(Wire *w,uint8_t byte,uint32_t now){
  uint16_t next=(uint16_t)(w->rx_write+1u);w->last_rx_ms=now;
  if((uint16_t)(next-w->rx_read)>WIRE_RING_SIZE){w->transport_fault=true;return;}
  w->ring[w->rx_write&(WIRE_RING_SIZE-1u)]=byte;w->rx_write=next;
}
void wire_transport_error(Wire *w){w->transport_fault=true;}
bool wire_service_fault(Wire *w,uint32_t now){
  if(w->transport_fault&&!w->fault_reported){
    w->fault_reported=true;mq_fault(w->queue,MQ_ERR_UART,now);
  }
  /* Preserve buffered/partial RX: a complete clean STOP after the fault must
     still reach the unchanged CRC/identity/replay gates. A corrupt prefix can
     still consume a later frame; collisions are not reliable delivery. */
  return w->transport_fault;
}
static bool sync_fault(Wire *w,uint32_t now){
  if(!sync_valid(w)){wire_transport_error(w);return wire_service_fault(w,now);}
  uint32_t saved=enter(w);(void)pending(w);
  bool fault=wire_service_fault(w,now);leave(w,saved);return fault;
}
static bool same_request(const Wire *w,const uint8_t *f,uint16_t n){
  return n==w->previous_length && !memcmp(f+2,w->previous+2,n-4u);
}
static void stage_reply(Wire *w,uint32_t req,MQResult result,uint8_t op,uint32_t now){
  w->ack_request=req;w->ack_result=(uint8_t)result;w->reply_queued_ms=now;
  w->reply_requires_healthy=result==MQ_OK&&op!=WIRE_STOP&&op!=WIRE_QUERY;
  w->reply_pending=true;
}
static void reject(Wire *w,uint32_t req,MQResult result,uint32_t now){
  /* A malformed or stale STOP must not displace a pending valid reply. */
  if(!w->reply_pending)stage_reply(w,req,result,0,now);
}
static void dispatch(Wire *w,const uint8_t *f,uint16_t n,uint32_t now){
  MQ *q=w->queue;uint8_t op=f[3];bool stop=op==WIRE_STOP;MQResult r;
  if(n<36u||op<WIRE_BEGIN||op>WIRE_RENEW)return;
  /* The master waits for a reply before sending another ordinary request.
     STOP still runs while a reply waits or its transmit is in flight. */
  if(!stop&&(w->reply_pending||!w->io.tx_ready(w->user)))return;
  uint64_t boot=r64(f+6),task=r64(f+26);
  uint32_t epoch=r32(f+14),req=r32(f+18),reserved=r32(f+22);
  if(!req){
    if(op==WIRE_QUERY&&n==36u&&!boot&&!epoch&&!reserved&&!task)stage_reply(w,0,MQ_OK,op,now);
    return;
  }
  if(boot!=q->boot_id){reject(w,req,MQ_IDENTITY,now);return;}
  if(reserved){reject(w,req,MQ_INVALID,now);return;}
  sync_fault(w,now);
  /* Critical UART/overflow is boot-latched, independently of MQ task cleanup.
     Check before cached ACKs: even an old successful START is now unhealthy. */
  if(w->transport_fault && op!=WIRE_STOP && op!=WIRE_QUERY){
    reject(w,req,MQ_UNHEALTHY,now);return;
  }
  if(w->has_request && req==w->last_request){
    uint32_t saved=enter(w);(void)pending(w);wire_service_fault(w,now);
    bool same=same_request(w,f,n);
    r=same?(MQResult)w->previous_result:MQ_IDENTITY;
    (void)pending(w);
    if(wire_service_fault(w,now)&&op!=WIRE_STOP&&op!=WIRE_QUERY)r=MQ_UNHEALTHY;
    leave(w,saved);
    if(!same)reject(w,req,r,now);
    else if(!w->reply_pending||w->ack_request!=req||w->ack_result!=(uint8_t)r)stage_reply(w,req,r,op,now);
    return;
  }
  if(w->has_request && (int32_t)(req-w->last_request)<=0){reject(w,req,MQ_ORDER,now);return;}
  if(!stop && epoch!=q->odom.epoch){reject(w,req,MQ_IDENTITY,now);return;}
  if(n!=((op==WIRE_BEGIN||op==WIRE_APPEND)?46u:36u)){
    reject(w,req,MQ_INVALID,now);return;
  }
  switch(op){
    case WIRE_BEGIN:r=mq_begin(q,task,epoch,r16(f+34),r32(f+36),r32(f+40));break;
    case WIRE_APPEND:{MQSegment s;s.kind=r32(f+36);s.value=rf(f+40);r=mq_append(q,task,r16(f+34),s);break;}
    case WIRE_COMMIT:r=mq_commit(q,task);break;
    case WIRE_START:{
      if(!sync_valid(w)){wire_transport_error(w);wire_service_fault(w,now);r=MQ_UNHEALTHY;break;}
      uint32_t saved=enter(w);
      (void)pending(w);wire_service_fault(w,now);
      r=w->transport_fault?MQ_UNHEALTHY:mq_start(q,task,now);
      (void)pending(w);
      if(wire_service_fault(w,now))r=MQ_UNHEALTHY;
      leave(w,saved);break;
    }
    case WIRE_STOP:mq_stop(q,now);r=MQ_OK;break;
    case WIRE_RESET:r=mq_reset_odom(q,now);break;
    case WIRE_RENEW:r=mq_renew(q,task,now);break;
    default:r=MQ_OK;break;
  }
  /* Long operations (queue CRC/reset) remain outside the IRQ mask. If an IRQ
     reports a fault during them, publish it before an OK ACK can escape. */
  if(sync_fault(w,now)&&op!=WIRE_STOP&&op!=WIRE_QUERY)r=MQ_UNHEALTHY;
  stage_reply(w,req,r,op,now);
  w->last_request=req;w->has_request=true;w->previous_result=(uint8_t)r;
  memcpy(w->previous,f,n);w->previous_length=n;
}
static void consume(Wire *w,uint8_t byte,uint32_t now){
  if(!w->used){if(byte==0xa5)w->frame[w->used++]=byte;return;}
  if(w->used==1u && byte!=0x5a){w->used=byte==0xa5?1u:0u;return;}
  w->frame[w->used++]=byte;
  if(w->used==6u){
    uint16_t len=r16(w->frame+4);
    bool telemetry=w->frame[3]==WIRE_TELEMETRY&&len==WIRE_TELEMETRY_SIZE-8u;
    bool known_version=w->frame[2]==WIRE_VERSION||w->frame[2]==11u||w->frame[2]==10u;
    if(!known_version||(!telemetry&&(len<28u||len>WIRE_COMMAND_MAX-8u))){w->used=0;return;}
    w->wanted=(uint16_t)(len+8u);
  }
  if(w->used>=6u && w->used==w->wanted){
    /* Consume full legacy frames and telemetry echoes without dispatching
       embedded command-shaped bytes. RX during TX remains available to STOP. */
    if(w->frame[2]==WIRE_VERSION&&w->frame[3]!=WIRE_TELEMETRY&&
        r16(w->frame+w->wanted-2u)==wire_crc16(w->frame+2,w->wanted-4u))dispatch(w,w->frame,w->wanted,now);
    w->used=0;
  }
}
void wire_poll(Wire *w,uint32_t now){
  if(w->io.now_ms)now=w->io.now_ms(w->user);
  if(w->reply_pending&&(uint32_t)(now-w->reply_queued_ms)>=REPLY_TIMEOUT_MS)w->reply_pending=false;
  /* Bound each poll even if an interrupt producer continually adds garbage. */
  unsigned budget=WIRE_RING_SIZE;
  wire_service_fault(w,now);
  while(budget-- && w->rx_read!=w->rx_write){
    uint8_t b=w->ring[w->rx_read&(WIRE_RING_SIZE-1u)];w->rx_read=(uint16_t)(w->rx_read+1u);consume(w,b,now);
  }
  if(w->used&&(uint32_t)(now-w->last_rx_ms)>RX_FRAGMENT_TIMEOUT_MS)w->used=0;
  if(!w->reply_pending||w->used||w->rx_read!=w->rx_write||
      (uint32_t)(now-w->last_rx_ms)<RX_QUIET_MS||!w->io.tx_ready(w->user))return;
  bool built_fault=w->transport_fault,built_reported=w->fault_reported;
  uint8_t result=built_fault&&w->reply_requires_healthy?MQ_UNHEALTHY:w->ack_result;
  wire_telemetry(w->queue,w->tx,w->pub_sequence+1u,now,w->ack_request,result,
      true,built_fault,w->has_request?w->last_request:0);
  if(!sync_valid(w)){wire_transport_error(w);wire_service_fault(w,now);return;}
  uint32_t saved=enter(w);bool rx_pending=pending(w);wire_service_fault(w,now);
  if((!built_fault&&w->transport_fault)||(!built_reported&&w->fault_reported)||rx_pending||
      (w->io.now_ms&&w->io.now_ms(w->user)!=now)||w->used||w->rx_read!=w->rx_write||
      (uint32_t)(now-w->last_rx_ms)<RX_QUIET_MS||!w->io.tx_ready(w->user)){
    leave(w,saved);return;
  }
  if(w->io.tx_start(w->user,w->tx,WIRE_TELEMETRY_SIZE,now)){
    w->pub_sequence++;w->pub_tick_ms=now;w->reply_pending=false;
  }
  wire_service_fault(w,now);
  leave(w,saved);
}

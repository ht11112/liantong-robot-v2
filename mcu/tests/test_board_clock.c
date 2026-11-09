/* Execute the real board loop with a RAM-only clock, CAN peer and watchdog. */
#ifndef MQ_HOST_TEST
#error "Host clock fixture must not enter firmware"
#endif
#include "drive.h"
#include "wire.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static Drive drive;
static MQ queue;
static Wire wire;
static uint32_t clock_ms, receive_advance;
static struct { uint32_t KR; } watchdog;
#define IWDG (&watchdog)
static uint32_t HAL_GetTick(void){return clock_ms;}
static void HAL_Delay(uint32_t ms){clock_ms+=ms;}
static void uart_service_fault(uint32_t now){(void)now;}
static bool ready(void *u){(void)u;return true;}
static bool can_send(void *u,const uint8_t bytes[8]){(void)u;(void)bytes;return true;}
static uint32_t errors(void *u){(void)u;return 0;}
static bool serial_send(void *u,const uint8_t *b,uint16_t n,uint32_t at){
  (void)u;(void)b;(void)n;(void)at;assert(!"No UART command was submitted");return false;
}
static void can_receive(void){
  const uint8_t speed[8]={0x43,0x6c,0x60,3,0,0,0,0};
  clock_ms+=receive_advance; /* SysTick can occur while draining the CAN FIFO. */
  drive_receive_can(&drive,0x581u,8u,speed,HAL_GetTick());
}
static void iteration(void){
#include "board_iteration.inc"
}
static void receive_across_tick(uint32_t before,uint32_t advance,bool stale_health){
  memset(&drive,0,sizeof(drive));
  clock_ms=before;receive_advance=advance;watchdog.KR=0;
  drive.bus=(DriveBusOps){can_send,ready,errors};
  drive.view=(MQDriveView){.state=MQ_DRIVE_STOPPED,.stop_verified=true,.guard_valid=true};
  drive.cache_mask=8u|16u|32u; /* Actual status, fault and current cached objects. */
  drive.status_tick=drive.fault_tick=drive.current_tick=before-(stale_health?250u:1u);
  drive.last_aux_start=drive.last_guard_start=before;
  drive.last_speed_start=before-50u;
  drive_poll(&drive,before); /* Submit a real production speed SDO. */
  assert(drive.transaction.pending&&drive.transaction.index==0x606cu);
  mq_init(&queue,1u,drive_ops(),&drive);
  const WireOps io={.tx_ready=ready,.tx_start=serial_send};
  wire_init(&wire,&queue,&io,NULL,before);
  iteration();
  assert(drive.speed_tick==before+advance);
  assert(drive.sample.valid==!stale_health);
  assert(queue.odom.valid==!stale_health);
  if(!stale_health)assert(queue.odom.measurement_tick_ms==before+advance);
  assert(watchdog.KR==0xaaaau);
}
int main(void){
  receive_across_tick(100u,1u,false);
  receive_across_tick(UINT32_MAX,1u,false);
  receive_across_tick(100u,2u,false);
  receive_across_tick(1000u,1u,true);
  puts("PASS actual board loop accepts new CAN timestamps across SysTick/wrap without rejuvenating stale health");
  return 0;
}

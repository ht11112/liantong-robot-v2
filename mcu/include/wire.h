#ifndef MQ_WIRE_H
#define MQ_WIRE_H
#include "mq.h"
#include <stddef.h>

/* 串口请求与回复使用 V13；固定长度遥测包含任务和停车状态。 */
#define WIRE_VERSION 13u
#define WIRE_BAUD 115200u
#define WIRE_TELEMETRY_SIZE 110u
#define WIRE_COMMAND_MAX 64u
#define WIRE_RING_SIZE 128u
enum { WIRE_BEGIN=1, WIRE_APPEND, WIRE_COMMIT, WIRE_START,
       WIRE_STOP, WIRE_RESET, WIRE_QUERY, WIRE_RENEW, WIRE_STEER, WIRE_TELEMETRY=128 };
enum { WIRE_INITIALIZED=1, WIRE_ODOM_VALID=2, WIRE_EXECUTING=4,
       WIRE_ARRIVAL=8, WIRE_STOP_CONFIRMED=16, WIRE_RX_FAULT=32,
       WIRE_COMMITTED=64, WIRE_STARTED=128, WIRE_ACK_VALID=256, WIRE_STEER_VALID=512 };

/* 发送、时钟和临界区由板级适配；中断只接收字节，不执行队列。 */
typedef struct {
  bool (*tx_ready)(void *user);
  /* Nonblocking. Keep data until physical UART TC, then release RS485 DE. */
  bool (*tx_start)(void *user,const uint8_t *data,uint16_t length,uint32_t expected_tick_ms);
  uint32_t (*now_ms)(void *user);
  /* All three are required together on an interrupt-driven target. Portable
     single-threaded tests may leave all three NULL. Restore the saved mask. */
  uint32_t (*critical_enter)(void *user);
  void (*critical_exit)(void *user,uint32_t saved_mask);
  /* Called only inside the critical section. Observe pending RX/error/BUSY;
     latch a transport fault on hardware errors, without consuming a byte. */
  bool (*rx_pending)(void *user);
} WireOps;

typedef struct {
  MQ *queue;
  WireOps io;
  void *user;
  uint8_t tx[WIRE_TELEMETRY_SIZE],frame[WIRE_TELEMETRY_SIZE];
  uint8_t previous[WIRE_COMMAND_MAX];
  volatile uint8_t ring[WIRE_RING_SIZE];
  volatile uint16_t rx_write,rx_read;
  volatile uint32_t last_rx_ms;
  volatile bool transport_fault;
  uint32_t pub_sequence,pub_tick_ms,reply_queued_ms,ack_request,last_request;
  uint16_t used,wanted,previous_length;
  uint8_t ack_result,previous_result;
  bool reply_pending,reply_requires_healthy,has_request,fault_reported;
} Wire;

uint16_t wire_crc16(const uint8_t *data,size_t length);
void wire_init(Wire *w,MQ *queue,const WireOps *io,void *user,uint32_t now);
/* ISR-safe producer only: never dispatch MQ or transmit from the ISR. */
void wire_rx_byte(Wire *w,uint8_t byte,uint32_t rx_tick_ms);
void wire_transport_error(Wire *w);
/* Main-context publication only; returns the current boot-latched fault. */
bool wire_service_fault(Wire *w,uint32_t now);
void wire_poll(Wire *w,uint32_t now);
/* Portable codec also used by native/WASM tests; no packed C structs. */
size_t wire_telemetry(const MQ *q,uint8_t *out,uint32_t pub_sequence,
                      uint32_t pub_tick_ms,uint32_t ack_request,uint8_t ack_result,
                      bool ack_valid,bool rx_fault,uint32_t accepted_request_watermark);
#endif

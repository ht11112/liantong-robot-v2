#ifndef MQ_WIRE_H
#define MQ_WIRE_H
#include "mq.h"
#include <stddef.h>

#define WIRE_VERSION 10u
#define WIRE_BAUD 115200u
#define WIRE_PERIOD_MS 50u
#define WIRE_TELEMETRY_SIZE 110u
#define WIRE_COMMAND_MAX 64u
#define WIRE_RING_SIZE 128u
enum { WIRE_BEGIN=1, WIRE_APPEND, WIRE_COMMIT, WIRE_START,
       WIRE_STOP, WIRE_RESET, WIRE_QUERY, WIRE_TELEMETRY=128 };
enum { WIRE_INITIALIZED=1, WIRE_ODOM_VALID=2, WIRE_EXECUTING=4,
       WIRE_ARRIVAL=8, WIRE_STOP_CONFIRMED=16, WIRE_RX_FAULT=32,
       WIRE_COMMITTED=64, WIRE_STARTED=128, WIRE_ACK_VALID=256 };

typedef struct {
  bool (*tx_ready)(void *user);
  /* Nonblocking. Keep data until physical UART TC, then release RS485 DE. */
  bool (*tx_start)(void *user,const uint8_t *data,uint16_t length,uint32_t expected_tick_ms);
  uint32_t (*now_ms)(void *user);
} WireOps;

typedef struct {
  MQ *queue;
  WireOps io;
  void *user;
  uint8_t tx[WIRE_TELEMETRY_SIZE],frame[WIRE_COMMAND_MAX];
  uint8_t previous[WIRE_COMMAND_MAX];
  volatile uint8_t ring[WIRE_RING_SIZE];
  volatile uint16_t rx_write,rx_read;
  volatile uint32_t last_rx_ms;
  volatile bool transport_fault;
  uint32_t origin_ms,attempted_frame,pub_sequence,pub_tick_ms;
  uint32_t grant_frame_start,last_used_grant,ack_request,last_request;
  uint16_t used,wanted,previous_length;
  uint8_t ack_result;
  bool has_published,ack_valid,has_request,fault_reported;
} Wire;

uint16_t wire_crc16(const uint8_t *data,size_t length);
void wire_init(Wire *w,MQ *queue,const WireOps *io,void *user,uint32_t now);
/* ISR-safe producer only: never dispatch MQ or transmit from the ISR. */
void wire_rx_byte(Wire *w,uint8_t byte,uint32_t rx_tick_ms);
void wire_transport_error(Wire *w);
void wire_poll(Wire *w,uint32_t now);
/* Portable codec also used by native/WASM tests; no packed C structs. */
size_t wire_telemetry(const MQ *q,uint8_t *out,uint32_t pub_sequence,
                      uint32_t pub_tick_ms,uint32_t ack_request,uint8_t ack_result,
                      bool ack_valid,bool rx_fault,uint16_t open_ms,uint16_t close_ms,
                      uint32_t accepted_request_watermark);
#endif

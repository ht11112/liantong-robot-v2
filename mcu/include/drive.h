#ifndef MQ_DRIVE_H
#define MQ_DRIVE_H
#include "mq.h"

/* 驱动层维护 CAN 事务、实测轮速和停车证明，供任务队列读取。 */
typedef struct {
  bool (*send)(void *user,const uint8_t data[8]);
  bool (*tx_ready)(void *user);
  uint32_t (*bus_errors)(void *user);
} DriveBusOps;

enum { DRIVE_ERR_STOP=40,DRIVE_ERR_STARTUP=52,DRIVE_ERR_CAN=72,
  DRIVE_ERR_UART=81,DRIVE_ERR_LIMIT=82,DRIVE_ERR_ARM_TIMEOUT=83,
  DRIVE_ERR_GUARD=86,DRIVE_ERR_SAMPLE=87 };
enum { DRIVE_SDO_NONE=0,DRIVE_SDO_TIMEOUT,DRIVE_SDO_ABORT,DRIVE_SDO_MALFORMED,DRIVE_SDO_SEND };

typedef struct {
  uint16_t index;
  uint8_t sub,bytes,kind,owner;
  uint32_t value,started_ms,generation,completed_ms;
  bool pending,done,ok;
} DriveTransaction;

typedef struct {
  MQDriveView view;
  DriveBusOps bus;
  void *bus_user;
  DriveTransaction transaction;
  MQSample sample;
  uint32_t now,first_error_tick,first_can_error;
  uint32_t speed_tick,left_tick,right_tick,status_tick,fault_tick,current_tick;
  uint32_t position_left,position_right,speed_raw,status_raw,fault_raw,current_raw;
  uint32_t sample_sequence,sample_overruns,late_replies,malformed_replies;
  uint32_t last_can_status,sdo_error_tick,sdo_abort_code;
  uint32_t sdo_timeout_tick;
  uint16_t sdo_error_index,sdo_timeout_index;
  uint8_t sdo_error_sub,sdo_error_reason,sdo_timeout_sub;
  uint32_t limit_values[5],generation,desired_generation,last_target_ms;
  uint32_t flow_started_ms,stop_wait_started_ms,zero_sequence,zero_count;
  uint32_t last_speed_start,last_aux_start,last_guard_start,last_rx_ms;
  uint16_t flow_step;
  uint8_t flow,aux_index,cache_mask;
  int16_t desired_left,desired_right;
  bool target_pending,sample_ready,tx_suppressed;
  bool zero_ok,shutdown_ok,disable_ok,stationary_ok,status_ok,fault_ok;
  bool guard_known,stop_explicit,boot_finished,uart_failed,sdo_ambiguous;
} Drive;

void drive_init(Drive *d,const DriveBusOps *ops,void *user,uint32_t now);
void drive_poll(Drive *d,uint32_t now);
void drive_receive_can(Drive *d,uint32_t id,uint32_t dlc,const uint8_t data[8],uint32_t now);
bool drive_pop_sample(Drive *d,MQSample *sample);
void drive_uart_fault(Drive *d,uint32_t now);
const MQDriveOps *drive_ops(void);
#endif

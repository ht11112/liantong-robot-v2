#ifndef MQ_H
#define MQ_H
#include <stdbool.h>
#include <stdint.h>
#if defined(MQ_PRODUCTION) && defined(MQ_HOST_TEST)
#error "Production and host test builds must be separate"
#endif

/* 任务容量、租约和采样超时均由 MCU 独立约束。 */
#define MQ_CAPACITY 128u
#define MQ_LEASE_MS 30000u
#define MQ_PI 3.14159265358979323846f
#define MQ_DIAMETER_M .20f
#define MQ_SEPARATION_M .30f
#define MQ_SAMPLE_TIMEOUT_MS 250u

/* 状态区分上传、执行、段间停车和最终停车确认。 */
typedef enum { MQ_IDLE=0, MQ_LOADING, MQ_READY, MQ_ARMING, MQ_MOVE,
  MQ_TURN, MQ_BARRIER, MQ_STOPPING, MQ_COMPLETE, MQ_FAILED } MQState;
typedef enum { MQ_OK=0, MQ_INVALID=1, MQ_BUSY=2, MQ_IDENTITY=3,
  MQ_ORDER=4, MQ_CRC=5, MQ_UNHEALTHY=6, MQ_STALE=7, MQ_EXPIRED=8 } MQResult;
typedef enum { MQ_ERR_NONE=0, MQ_ERR_CANCEL=1, MQ_ERR_ODOM=2,
  MQ_ERR_DEADLINE=3, MQ_ERR_DRIVE=4, MQ_ERR_APPLY=5,
  MQ_ERR_HANDOFF=6, MQ_ERR_UART=7, MQ_ERR_BOOT=8 } MQError;
enum { MQ_KIND_MOVE=1, MQ_KIND_TURN=2 };
/* 队列由 MOVE/TURN 段组成；段内容和顺序由 CRC 绑定。 */
/* 里程计和驱动回读均携带真实测量时间和样本序号。 */
typedef struct { uint32_t kind; float value; } MQSegment;
typedef struct {
  uint32_t sequence, tick_ms;
  int16_t left_tenths_rpm, right_tenths_rpm;
  uint32_t position_left, position_right;
  bool valid;
} MQSample;
typedef enum { MQ_DRIVE_STARTUP=0, MQ_DRIVE_STOPPED, MQ_DRIVE_ARMING,
  MQ_DRIVE_ENABLED, MQ_DRIVE_STOPPING, MQ_DRIVE_FAILED } MQDriveState;
typedef struct {
  MQDriveState state;
  uint32_t error, applied_generation, applied_tick_ms;
  int16_t applied_left, applied_right;
  bool stop_verified, guard_valid;
} MQDriveView;
typedef struct {
  void (*arm)(void *user, uint32_t now);
  uint32_t (*target)(void *user, int16_t left, int16_t right, uint32_t now);
  void (*stop)(void *user, uint32_t now);
  MQDriveView (*view)(void *user);
} MQDriveOps;
typedef struct {
  float x,y,yaw,yaw_total,travel,v,w,previous_v,previous_w;
  uint32_t sample_sequence, measurement_tick_ms, epoch, discontinuities;
  int16_t left_tenths_rpm,right_tenths_rpm;
  uint32_t position_left,position_right;
  uint32_t recovery_sequence,recovery_tick,recovery_start_tick,recovery_count;
  bool initialized,valid,recovery_allowed;
} MQOdom;
typedef struct {
  MQOdom odom;
  MQSegment segments[MQ_CAPACITY];
  MQDriveOps drive;
  void *drive_user;
  uint64_t boot_id,task_id;
  uint32_t expected_crc,lease_started_ms,started_ms,first_error_tick_ms;
  uint32_t first_error,first_drive_error,epoch_at_upload;
  uint32_t target_generation,target_requested_ms,last_target_ms;
  uint32_t barrier_started_ms,barrier_sample,zero_count;
  uint16_t count,received,index;
  MQState state;
  int16_t desired_left,desired_right;
  float segment_travel,segment_yaw,progress,yaw_deviation,lateral_deviation;
  float segment_x,segment_y;
  bool committed,has_started,arrival_reached,physical_stop_confirmed;
  bool barrier_applied,target_pending,critical_latched;
} MQ;

void mq_init(MQ *q,uint64_t boot_id,const MQDriveOps *ops,void *user);
void mq_sample(MQ *q,const MQSample *sample);
void mq_tick(MQ *q,uint32_t now);
void mq_fault(MQ *q,uint32_t error,uint32_t now);
MQResult mq_begin(MQ *q,uint64_t task,uint32_t epoch,uint16_t count,
                  uint32_t crc,uint32_t lease_ms);
MQResult mq_append(MQ *q,uint64_t task,uint16_t index,MQSegment segment);
MQResult mq_commit(MQ *q,uint64_t task);
MQResult mq_start(MQ *q,uint64_t task,uint32_t now);
MQResult mq_renew(MQ *q,uint64_t task,uint32_t now);
uint32_t mq_lease_remaining_ms(const MQ *q,uint32_t now);
void mq_stop(MQ *q,uint32_t now);
MQResult mq_reset_odom(MQ *q,uint32_t now);
uint32_t mq_segments_crc(const MQSegment *segments,uint16_t count);
bool mq_is_executing(const MQ *q);
#endif

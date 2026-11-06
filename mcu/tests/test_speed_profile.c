/* Actual MQ with a fixed stopped sample, independent of the selected RPM magnitudes. */
#include "mq.h"
#include <assert.h>
#include <stdio.h>

typedef struct {
  MQDriveView view;
  int16_t left,right;
  unsigned targets,stops;
} Peer;

static void arm(void *user,uint32_t now){
  (void)now;Peer *p=user;p->view.state=MQ_DRIVE_ENABLED;
}
static uint32_t target(void *user,int16_t left,int16_t right,uint32_t now){
  (void)now;Peer *p=user;p->left=left;p->right=right;return ++p->targets;
}
static void stop(void *user,uint32_t now){(void)now;((Peer *)user)->stops++;}
static MQDriveView view(void *user){return ((Peer *)user)->view;}

static void check_target(MQSegment segment,int16_t left,int16_t right){
  static const MQDriveOps ops={arm,target,stop,view};
  Peer p={.view={.state=MQ_DRIVE_STOPPED,.stop_verified=true,.guard_valid=true}};
  MQ q;mq_init(&q,1,&ops,&p);
  MQSample sample={.sequence=1,.valid=true};mq_sample(&q,&sample);
  assert(mq_begin(&q,1,q.odom.epoch,1,mq_segments_crc(&segment,1),MQ_LEASE_MS)==MQ_OK);
  assert(mq_append(&q,1,0,segment)==MQ_OK&&mq_commit(&q,1)==MQ_OK);
  assert(mq_start(&q,1,0)==MQ_OK);
  mq_tick(&q,1);mq_tick(&q,2);
  assert(p.targets==1&&p.left==left&&p.right==right&&!q.arrival_reached&&!p.stops);
}

int main(void){
  check_target((MQSegment){MQ_KIND_MOVE,.301f},MQ_MOVE_CRUISE_RPM,-MQ_MOVE_CRUISE_RPM);
  check_target((MQSegment){MQ_KIND_MOVE,.30f},MQ_MOVE_NEAR_RPM,-MQ_MOVE_NEAR_RPM);
  check_target((MQSegment){MQ_KIND_TURN,1.f},-MQ_TURN_RPM,-MQ_TURN_RPM);
  check_target((MQSegment){MQ_KIND_TURN,-1.f},MQ_TURN_RPM,MQ_TURN_RPM);
  printf("PASS selected MOVE cruise %d RPM, near %d RPM, TURN %d RPM in both directions\n",
         MQ_MOVE_CRUISE_RPM,MQ_MOVE_NEAR_RPM,MQ_TURN_RPM);
  return 0;
}

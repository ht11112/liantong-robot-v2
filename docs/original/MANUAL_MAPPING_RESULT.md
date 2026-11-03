# 2026-11-03 手推新地图r5保存结果

用户通知“新地图采集完成”后，本轮只结束记录并保存。实际查询记录641.012066秒，`recording_complete=true`、`recording_ended_cleanly=true`、`gap_count=0`、`first_failure=null`。这些结论核对采集连续性和正常收尾，不表示车体几何、地图精度或导航已通过现场验收。

| 项目 | 结果 |
| --- | --- |
| 会话 | `76ebb1d2`，boot `2fee0fae-658d-4245-9bd8-fc65e39f297b` |
| RK目录 | `/home/cat/p2p_navigation_20261103_manual_mapping_155534_76ebb1d2` |
| 最终地图 | 480×640格，分辨率0.05000000074505806米，已知39661格 |
| 地图栅格原点 | `[-5.000000000000001, -16.2, 0.0]`，与软件odom原点是不同概念 |
| 最终PGM SHA256 | `30642d3090dce285fd722678f54620c2a82efdd3b5b0764d72bbb78a755c79fa` |
| 真实数据 | 6264个不同轮速样本/odom，8584帧前180°扫描，523张地图消息 |
| 串口请求 | STATUS 5852，FEEDBACK 23123；STOP/ARM/SETPOINT均0 |
| 原点 | sample83421，source monotonic8356.437404242999；软件`(0,0,0)`，原始编码器未重置 |
| 末次odom | `(0.9400894875164166, -0.17662258724696242, 0.41878477269903225)`；非外部真值或已证明的闭环误差 |
| 收尾 | 查询口正常关闭，query/writer退出，自有gmapping SIGINT退出0 |
| 结束后清点 | monotonic9069.300198662，仅6基础进程与USB0雷达owner，无AMCL/地图/SLAM/odom/控制口owner |

PC备份在[`manual_mapping_r5`](../../output/p2p_redesign_20261103/manual_mapping_r5/)，其中[`final.pgm`](../../output/p2p_redesign_20261103/manual_mapping_r5/new_map/final.pgm)和[`final.yaml`](../../output/p2p_redesign_20261103/manual_mapping_r5/new_map/final.yaml)为保存地图，另存原始grid和原点记录。18成员证据包SHA256为`e722fe456c83f9f70e2ca5d70bc3ed0b496cf2a3cf2332745088ed3712b11ac0`，下载后已校验。旧r1至r4失败与部分地图单独保留。

RK源9文件SHA保存前后不变，同包PC27项及本轮RK27项检查通过。[独立离线审计](../../output/p2p_redesign_20261103/manual_mapping_r5_independent_review.json)已完成，`audit_consistent=true`，报告SHA256为`99e4bd3dc499728c5c82917e8de581786fe651915ea809be03dcf82fd6c53e56`。79952原事件、28975完整收发及业务行逐一CRC/seq核对；6264个真实样本独立复算积分，最大数值差2.78e-15，0缺段。source最大龄.203328秒、CAN最大龄.130秒；完整PGM从原grid逐格重建一致。审计只证明原记录与完整结束结论一致，不替代几何精度核验。

新增[完整PNG](../../output/p2p_redesign_20261103/manual_mapping_r5/new_map/final.png)和[已知区裁图](../../output/p2p_redesign_20261103/manual_mapping_r5/new_map/final_known_crop.png)，逐像素保真，不改变原图；裁图286×385，原PNG偏移left0/top94。可见黑色边界断裂/碎点、外缘放射状白细带与扇区，灰色未知区保留。没有修墙、清噪或补画，也未假定odom与地图重合。

网页已刷新为“记录已结束”，仅显示本轮历史结果。未提供map→odom变换，因此网页中的odom轨迹与SLAM地图分开显示，不把手工标记的odom坐标当作地图定位。雷达外参、轮径及轮距仍为未标定估计，未删除未知区域或补画缺测空间。没有启动任何定位或运动任务。

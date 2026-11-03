# 新 P2P 静止实车接入可复用部件

本笔记仅依据本地源码与已有归档，只读整理于 2026-11-03。本次整理没有连接 RK、打开设备、启动服务或执行历史 runner。当前实车事实由 root 的当次采集确定。本阶段目标是新独立包的 STATUS/STOP/FEEDBACK、前 180°扫描和测得轮速里程计；没有 ARM、任何 SETPOINT（包括零）或旧网页入口。

## 建议的最小抽取入口

优先借用 [query_only_probe.py](C:/Users/Administrator/Desktop/联通智能机器人/output/v9_async_log_20261102/query_only_probe.py) 中 `QueryPort`、`capture()`、`stop_now()` 和单 pending 查询循环，在新 P2P 入口中组装；原脚本 `main()` 不是通用入口：它硬编码旧 boot `6e26019e-04cf-4290-9fc6-c4376a03f990`，只接受 `--query-only-20s`，本身不发布 ROS 里程计，也不采集激光。

三个底层依赖都已在同一小包中，另需一个停车判定 helper：

| 部件 | 本地精确副本 | SHA256 |
|---|---|---|
| wire9 编解码、单 owner/单 pending Client | [control_client.py](C:/Users/Administrator/Desktop/联通智能机器人/output/v9_async_log_20261102/query_probe_package/control_client.py) | `a293816a54a52eed2a60dbb310b639c43763f14146797dc22e6dd8ab992b5e55` |
| 独占串口、保存/恢复 termios | [transport.py](C:/Users/Administrator/Desktop/联通智能机器人/output/v9_async_log_20261102/query_probe_package/transport.py) | `45950cfdd941b6f0d49f27f9a99ad6294ea32443ae3d34e64569f7c1ca7f57ae` |
| 按 USB 身份定位控制适配器 | [rk_link.py](C:/Users/Administrator/Desktop/联通智能机器人/output/v9_async_log_20261102/query_probe_package/rk_link.py) | `d3546a87e31f79d43de63396f4d5c5699b32a532df2182757f5324ee49a99f0b` |
| 新 STOP 后 STATUS 判定 | [stop_observation.py](C:/Users/Administrator/Desktop/联通智能机器人/output/v9_async_log_20261102/query_probe_package/stop_observation.py) | `4492427d57479a884f5efe11da96596eace31605355af5ac7f0ca3c301eac97b` |

[probe_manifest.json](C:/Users/Administrator/Desktop/联通智能机器人/output/v9_async_log_20261102/query_probe_package/probe_manifest.json) 保存这组固定源；[build_query_probe.py](C:/Users/Administrator/Desktop/联通智能机器人/output/v9_async_log_20261102/build_query_probe.py) 记录三个冻结来源与 locator 原路径。它们没有把串口依赖变成 Python 标准库：`exclusive_port()` 运行时需要 RK 的 `serial`/pyserial 与 POSIX termios，`rk_link.locate()` 使用系统 `fuser`。

`QueryPort.write()` 是最内层白名单：严格 bytes、完整 20B、`AA 55 09`、kind 仅 2/3/5、非零序号、12B 零 payload、原 CRC16；它拒绝所有 6/7。`counts` 只是发送尝试计数，不能代替 ACK。新包应保留这个围栏，把 Client 的 `arm()`/`setpoint()` 方法存在与实际允许发送分开。

`rk_link.locate()` 要求 hostname=`ltzx3-robot`、eth0 MAC=`f6:91:b4:0a:da:22`，仅匹配 `1a86:7523/bcdDevice0264` 且恰好一个适配器，`fuser` 必须明确无人占用。历史 `/dev/ttyUSB2` 只是当时结果，不能作为新入口固定端口。

## 可复用的运行次序与参数

1. root 完成当次 boot、进程、ROS 图、串口占用核对；新入口绑定本次 boot 和新目录。激光订阅者可以先启动，不另开串口 owner。
2. `target=locator.locate()`；仅一次进入 `transport.exclusive_port(target['port'])`；包装 `QueryPort`；构造唯一 `Client`；`enable_nonblocking()` 后底层 read/write timeout 都为 0。
3. 在同一 owner 中执行初始 `verify_stop()`；使用本次真实 Client completion ticket 的 `started_monotonic_s/completed_monotonic_s` 保存来源，不能以 ROS 发布时刻重盖。
4. 原小 probe 调度为 FEEDBACK 25ms、STATUS 100ms、pending poll 间隔 1ms，只有一个在途事务。它们是调度目标，不是硬实时保证。Client 原事务 deadline 为 300ms；STOP/反馈来源的当前实际值另记录，不能宣称 site JSON 已更改固件。
5. 同一个 owner 处理有效新 FEEDBACK、更新测得 RPM 里程计；ROS 回调只收激光或事件，不访问该串口。旧重复 CAN sample 保留首次来源时刻。
6. 正常完成、人工中止或异常退出都在物理端口关闭前由同一 owner 尝试最终 STOP 并取得后置 STATUS。磁盘结果在停车/端口关闭后写，停车结果与采集结果分别记录。

新用户反馈容忍已选 750ms，但此历史 Client 的 `fresh()` 仍要求 CAN tick 来源≤250ms，并保留 15RPM 异常阈值；新主机 watchdog 与底层 CAN 新鲜度是不同字段。本笔记未改这两层。

## 真实轮速里程计

可复用 [core.py](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_20261102/package/source/_attended/navigation-demo/green_base_bridge/src/green_base_bridge/core.py) 的纯 `MeasuredOdometry`，避免搬入旧 `point_bridge` 全任务链。它只使用已经过 `fresh()` 的 FEEDBACK 速度：左轮正、右轮反号；RPM 乘 `pi*diameter/60`，用 MCU `speed_tick_ms` 做梯形积分。重复 `(sample_sequence,speed_tick_ms)` 返回 None；序号不连续或 MCU dt 不在 `(0,250ms]` 时标记 discontinuity，不补算缺失路程。

该实现有 `<0.2RPM` 死区，D=0.2m/B=0.3m 是估计，`calibrated=False` 应保留。它输出的是实际反馈驱动的估计里程计，不是以目标速度代替反馈，也不是已标定的编码器距离。新 ROS 适配层需要发布 Odometry/必要 TF，并明确本次原点和 discontinuity。

[wheel_travel.py](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_20261102/package/source/_attended/navigation-demo/local_task_adapter_20261030/wheel_travel.py) 说明原始位置是 I32 counts，`counts_per_revolution` 历史未确认；不要采用 16384 候选值把 counts 自动换算为米。FEEDBACK 解码已经将 606C packed I16 除 10 为 RPM，只除一次。

## 前 180°扫描

新独立纯裁剪已在 [scan_sector.py](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/p2p_navigation_20261103/scan_sector.py) 的 `front_180_scan()`。新 ROS 适配应把当次真实扫描 `ranges/angle_min/angle_increment/header.stamp` 传入并保留源 stamp；后方写 NaN，不能用 Inf 把盲区表示成可通行空间。它不提供障碍净距或安装外参实测。

历史 ROS 接线可参考 [scan_link_node.py](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_20261102/package/source/scan_link_node.py)：`/lidar1/laser_fan` → `/scan_nav`，仅适配 frame 为 `laser_link`、不改时间；默认 `base_link→laser_link` 平移 `(0.13,0,0.20)` 与 yaw0 明确是估计。旧扫描驱动曾把 frame 填成 `map`，不能据此当作真实 map 位姿。

旧过滤/裁剪链参考 [scan_sector_node.py](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_20261102/package/source/scan_sector_node.py)、[scan_sector_core.py](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_20261102/package/source/scan_sector_core.py)、[scan_sector_profile.json](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_20261102/package/source/scan_sector_profile.json)：`/scan_mapping` → `/scan_navigation_sector`，前±90°、保留整体角度布局。新 P2P 无需把旧包的所有碰撞/网页模块带入。旧 `laser_body_filters.yaml` 含具体车体遮罩，不能不核对现场就把其近距点删掉。

## STOP 的可确认含义

`stop_observation.verify_stop()` 先 `Client.stop()` 取得原协议 STOP ACK，再在 1.5s 窗口内循环 `stopping_status()`，间隔20ms。`Client` 自己核 CRC、identity、关联 sequence/clock；仅 ACK 不能证明异步停车结束。

`stopped(packet)` 要求 `state=1`、`stop_verified=1`、`guard_flags&7=5`，且 `status/reserved/error/session/target_pack/uart_critical_short/uart_critical_total` 全零。新最终结果应保存本次 STOP request/ACK 及之后新 STATUS 的原始完成 ticket、raw/reply/时间，并分别记录失败。零目标、任务状态、旧 STATUS、进程 exit0 以及回读到零速度本身都不能替代该链。

若复用日志型父证明，最新 [required_serial_journal.py](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_strict_journal_spsc_20261102/navigation_candidate/src/required_serial_journal.py) 的 `cleanup_stop_evidence(rows,cleanup_started)` 要求 cleanup 之后新 ACK→新 STATUS、完整前缀与精确的已取消 query 分类；原始错误必须保留。新小包可以直接保存同 owner 的本次完成记录，避免复用旧父/子日志交接。

## 最近定位/同 owner 入口与真实结果

- 最简单历史实物成功是 [query_result.json](C:/Users/Administrator/Desktop/联通智能机器人/output/v9_async_log_20261102/actual_rk_query_only_20261102_213922_de9c91/query_result.json)：20.451s、734 FEEDBACK/196 STATUS/2 STOP、初末 STOP 确认、零 ARM/SETPOINT。它不含 ROS 里程计或定位。
- 同 owner 静止60s的历史入口是 [static_trial_launcher.py](C:/Users/Administrator/Desktop/联通智能机器人/output/v9_async_log_20261102/static_launcher_candidate/static_trial_launcher.py)；默认 `_publish_primary_odom:=false/_publish_odom_tf:=false`。实物归档 `output/v9_async_log_20261102/actual_rk_same_owner_static_20261102_222321_37df21/` 通过限定静止 owner 观察，但不能当定位结果。
- 开启真实 odom/TF 的定位入口是 [static_localization_launcher.py](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_20261102/static_localization_launcher.py)；后续调度入口为 [warmup launcher](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_warmup_20261102/src/static_localization_launcher.py)。它先暖 scan/filter/sector/map/AMCL，再启动唯一旧 owner，参数包含 `_enable_motion:=false/_site_motion_authorized:=false/_publish_primary_odom:=true/_publish_odom_tf:=true/_geometry_calibrated:=false`，原总观察120s。
- 最新实际 runner 是 [run_rk_static_localization_r3.py](C:/Users/Administrator/Desktop/联通智能机器人/output/v9_async_log_20261102/run_rk_static_localization_r3.py)，选择 `static_localization_warm_ready_r1/static_localization_package.tar.gz`。该包显式叠 warmup launcher 与 strict journal 两源，468成员，package SHA `59fcf134d10a7d4affe1a91c7ede8e68ae687450b4458b3b361c35e535039bd9`，launcher manifest SHA `9e3e66241297a78356c14d14183245dd89445496e252f2d6aafdb2f5e936cbdf`。这只是追溯入口，不建议为新 P2P 原样运行。
- 原 launcher CLI 为 `--launcher-manifest-sha256 <SHA> --expected-boot-id <当次boot> --static-localization-only`；outer 使用 `/usr/bin/env -i`、`LD_LIBRARY_PATH=/opt/ros/noetic/lib`、`HOME=<新/tmp目录>`、`/usr/bin/python3 -I -B`。child ROS 环境还含 `/opt/ros/noetic/lib/python3/dist-packages`、`ROS_MASTER_URI=http://127.0.0.1:11311`。当前 RK 的导入路径、依赖版本和图仍须重新取得。
- **最新实际不是通过**：`output/v9_async_log_20261102/actual_rk_static_localization_20261103_001037_1af3aa/static_evidence.tar.gz` 内 `static_localization_result.json`，exit2，首次拒绝是当次 `feedback_seen` 龄266.113ms超过旧250ms；state快照龄48.018ms，其余静止检查均真。所有child退出0、无强杀、源未变；但 `external_observed_localization=false`、`final_stop_verified=false`，停车证明报 `Known serial fault is not a verified unbound query cancellation`。不能写成“停车已证”或“最新定位通过”，也不能沿用其中 anchor/seed。

本阶段无需启动 AMCL、map_server、旧462源 bootstrap 或任务 guardian 才能完成 query/STOP、180°扫描与反馈里程计接入。先让新小包取得当次真实记录，再决定定位接线；历史成功只说明当时执行结果。

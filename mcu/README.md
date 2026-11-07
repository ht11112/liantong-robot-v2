# 第四分支：RK 任务续约

## MCU 离线安全修订（待实机验收）

此修订基于 `feature/rk-task-renewal` 的 `24966473db0ec1f6f0dfe3b58e17d7b765d98cd5`。仅做离线测试；未构建目标固件、刷写或操作车辆。上线前必须按下列步骤验收并确认生产镜像不超过 32 KB。

- 单次无效速度样本只在丢失前轮速为零、驱动处于已验证停车且保护正常的情况下允许自动恢复。此后须连续取得三个不同、序号连续且间隔不超过 250 ms 的零速有效样本；从最后可信样本到恢复结束总时长不超过 250 ms。恢复期间不积分，恢复样本作为新积分起点。期间运动、失去停车证明、序号跳变或超窗均取消自动恢复；执行中失效立即走原停车流程，之后须在确认停稳、反馈新鲜后显式重置里程计，不能恢复原任务。零速容差沿用现有 ±0.2 RPM，三个样本沿用停车证明的数量；这不能证明失效间隙内车辆绝对没有移动，使用前须实测确认机械锁止和外力条件。
- `FDCAN ECR` 的 TEC、REC 和 RP 仅记录，PSR 的 Bus-Off 立即触发原 CAN 故障停车。没有凭空指定错误计数或持续时间阈值；长期错误引起的 SDO 无响应仍由现有 50 ms 超时处理，运行时速度、状态、故障码、电流超过 250 ms 未更新仍触发样本故障。持续计数但反馈仍正常的单独停车阈值待硬件实测决定。
- 速度里程计不再依赖 `0x6064` 的新鲜度；位置读数仍轮询并作为辅助信息，驱动状态、故障码、电流及真实 `0x606C` 轮速的检查保留。任何 SDO 超时仍锁存通信歧义，包括位置对象超时，因此不是容忍位置 SDO 超时继续运动。
- RAM 内 `Drive` 的 `last_can_status`/`first_can_error` 保存原始总线状态（bit 0 是 Bus-Off，bit 1–15 是原 ECR）；`sdo_error_index/sub/reason/tick` 保留首个 SDO 失败，`sdo_abort_code` 保存 abort 原因；`sdo_timeout_index/sub/tick` 单独保留首个超时。超时后任何后续响应均不再作为成功事务或缓存依据，仍尝试原停车命令但不能证明停车。这些是调试器可读的 RAM 字段，不改变 V12 线协议，断电即丢失。

离线回归（在完整仓库根目录）：

```text
python -B mcu/build.py --host-tests --name mcu-safety-host
python -B mcu/build.py --firmware --name mcu-safety-production
```

本次主机测试的 8 个 C 测试程序通过，Python 76 项通过、1 项跳过；跳过项为已有的 C 黄金向量交叉检查，原因是现有测试读取的环境变量与构建脚本设置的不一致。目标构建所需 ARM 编译器及 STM32CubeG4 SDK 在本环境不可用，固件尺寸和目标编译尚未验证。离线模拟不能证明总线时序、停车距离或物理静止。

实机验收须另行授权并由具备相应经验的人员执行：先核对硬件接线、急停、悬空车轮及独立断电手段；完成目标构建和 32 KB 检查并保存原固件备份；在受控静止条件下验证启动、真实零速、单次无效采样恢复和超窗/运动失效锁存；注入瞬态 ECR、持续反馈中断、Bus-Off、位置读数延迟及 SDO 超时/迟到响应，逐项记录时间戳、对象、停车动作和不可确认停车的状态；最后在单独批准的安全场地以低速逐步验证停车距离。任何异常立即停止并保留记录，不自动恢复或重跑。阈值和恢复窗口须结合驱动器及车辆实际特性复核。

`feature/rk-task-renewal` 从第三分支的 RK 主站和 STM32 队列执行派生。第四分支的 RK 与 STM32 都使用 V12。[V12 线协议](docs/PROTOCOL_V12.md)

## 运行方式

MCU基于真实0.1RPM轮速及自身时间持续积分odom，以名义20Hz读取CAN轮速并在RK查询时返回累计结果。轮径0.20m、轮距0.30m仍未标定；不是用目标速度估计位移，也未把未标定编码器计数换成米。CAN的真实采样缺失会使odom失效；主机丢失遥测只会少看到几帧，不使MCU漏积分。

`measurement_tick_ms`实际记录MCU收到CAN读速响应的完成时刻，序号也是本机采纳序号；驱动器没有在此SDO中提供内部测量时间，不能把接收时刻称为驱动内部传感器时间。

MOVE使用两轮等速前进给定，按实测中心轮程结束；不纠偏，不因偏航超过8°停车。TURN按有符号的连续相对角度结束。默认直行巡航 2 RPM、剩余 30 cm 内 1 RPM、转向 1 RPM。普通 MOVE 容差 2 cm，末段 MOVE 容差 10 cm，TURN 容差 3°。

段间须零目标已应用，随后取得五个不同且严格晚于应用时刻的真实零速样本；该交接最多2秒。最终到点直接STOP，取消旧到点封存与停车后位置复验。`arrival_reached`只是动作终止条件已满足，`physical_stop_confirmed`另报告停车事实；二者不等于地图定位或地面终点精度已验。

队列最多 128 段，仅存 RAM。完整上传及 COMMIT 后处于 READY，显式 START 启用运动并发放 30 秒租约。RK 导航进程会给自己启动的活跃任务续约。任务 ID 在同一次 MCU 启动内严格增加，完成或置零后旧 ID 不能重跑。断电清空队列。

最后一次租约到期后，STM32 执行停车。MCU 仍负责 CAN 和驱动故障、电流、驱动 500 ms 看门狗、硬件 IWDG、停车与备用停车。执行路线使用保存地图和里程计，动态障碍需要现场监控。

CANopen SDO没有事务编号，任何超时之后的同对象迟到ACK无法可靠归属。本版因此锁存`sdo_ambiguous`：继续尝试物理停车，但不把后来回包升级为原停车证明，odom保持失效；显式STOP、置零或新任务不会清除此类故障。需要先调查原因、确认实际停稳，再重新启动MCU，不自动重跑队列。

## 精简与构建

原工程虽声明512KB Flash，本版链接器应用区严格为前32KB，并预留4KB栈。生产目标仅链接`board_g474.c`、`drive.c`、`mq.c`、`wire.c`与必要HAL；测试程序、旧v9控制、固定调试mailbox及长诊断分页不进入生产镜像。测试目标以`MQ_HOST_TEST`单独编译真实生产C及模拟CAN，生产目标使用`MQ_PRODUCTION`，两种宏不能同时启用。未整文件丢弃旧`bench_test.c`中的实际保护逻辑，其必要部分已经重构。

从仓库根目录运行：

```text
python -B mcu/build.py --host-tests --name v12-host-check
python -B mcu/build.py --firmware --name v12-production
```

构建参数 `--move-cruise-rpm`、`--move-near-rpm`、`--turn-rpm` 分别设置直行巡航、近端和转向的轮速给定。给定以整数 RPM 表示，范围由 CAN 双轮目标字段决定，方向由动作决定。MCU 在使能时沿用驱动器的最大转速配置。实测轮速按样本有效性和连续性进入里程计，V11 已移除 15 RPM 软件超速停车。

输出目录必须是新名字，结果在 `build/名字/`，含测试日志或生产 `mcu-v12.bin`、`.elf`、`.map` 及 SHA 和内存报告 `receipt.json`。构建是离线的，32 KB 超限即失败。

Linux主机测试使用`cc/gcc/clang`；Windows默认使用本机Unity的Emscripten/Node，路径可由`--emscripten`指定。生产构建使用STM32CubeG4 1.6.3及ARM GCC，路径通过`--sdk`、`--compiler-dir`指定；保持Cortex-M4、16MHz HSI、soft-float，与原硬件引脚及CAN500kbit/s配置一致。SDK不复制入仓库，构建报告记录实际SDK源与工具SHA。

## RK 工具

RK 使用 pyserial。第四分支沿用工具入口文件名 `mcu_v11.py`，传输版本为 V12。真实通信需显式指定端口与 `--execute`。按[静止验证顺序](docs/PROTOCOL_V12.md#静止验证顺序)先验证 QUERY、上传到 READY 和轮询基准。

```text
python3 -B mcu/tools/mcu_v11.py query --execute --port /dev/ttyUSBn
python3 -B mcu/tools/mcu_v11.py upload --execute --port /dev/ttyUSBn --queue mcu/examples/queue.json
python3 -B mcu/tools/mcu_v11.py benchmark --execute --port /dev/ttyUSBn --duration 65
```

上传只提交队列。单独运行命令行 `start --task-id` 时，任务只持有 START 发放的首个 30 秒租约。自动续约由网页配套的 RK 导航进程负责。`stop` 分别报告命令 ACK 和实测停稳。[队列和停车](docs/PROTOCOL_V12.md#队列和停车)

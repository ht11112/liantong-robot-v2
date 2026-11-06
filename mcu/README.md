# 第四分支：RK 任务续约

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

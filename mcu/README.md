# MCU v10：累计里程计与独立动作队列

本目录是第二分支`feature/mcu-odom-queue`的独立交付，从第一分支`f86ab2b`及操作者提供的G474 v9源码派生。已提供默认离线的部署和静止采集工具；首次隔离部署因独立读回异常中止，尚未完成静止或运动验收。RK costmap/A*、一次AMCL和8766网页接入留到下一阶段。

2026-11-05 新增[UART 收发交接加固候选](docs/UART_HARDENING.md)，处理离线模拟发现的同步、故障发布和 STOP 接收边界。候选仅在本地验证，未刷入设备，现场 FE 原因仍未定位；旧部署工具的固件身份不能直接用于新候选。

## 运行方式

MCU基于真实0.1RPM轮速及自身时间持续积分odom，以名义20Hz主动发布累计结果。轮径0.20m、轮距0.30m仍未标定；不是用目标速度估计位移，也未把未标定编码器计数换成米。CAN的真实采样缺失会使odom失效；主机丢失遥测只会少看到几帧，不使MCU漏积分。

`measurement_tick_ms`实际记录MCU收到CAN读速响应的完成时刻，序号也是本机采纳序号；驱动器没有在此SDO中提供内部测量时间，不能把接收时刻称为驱动内部传感器时间。

MOVE使用两轮等速前进给定，按实测中心轮程结束；不纠偏，不因偏航超过8°停车。TURN按有符号的连续相对角度结束。速度向下量化：直行最高2整数RPM（约0.02094m/s），近终点1RPM；转向1RPM（约4°/s），均低于原0.03m/s和6.9°/s上限。普通MOVE容差2cm、末MOVE10cm，TURN容差3°；MOVE在剩余30cm内减速。

段间须零目标已应用，随后取得五个不同且严格晚于应用时刻的真实零速样本；该交接最多2秒。最终到点直接STOP，取消旧到点封存与停车后位置复验。`arrival_reached`只是动作终止条件已满足，`physical_stop_confirmed`另报告停车事实；二者不等于地图定位或地面终点精度已验。

队列最多128段、仅存RAM，总时限默认1200秒。完整上传及COMMIT不授予运动，只有显式START才启用。任务ID在同一次MCU启动内严格增加，重发当前START只返回原任务状态，完成或置零后旧ID也不能重跑。断电清空，不自动续跑。

执行期间不依赖PC、RK或串口心跳，RK断开后仍由MCU完成队列。本阶段无雷达输入，不提供动态停障；下一阶段保留已约定的1m雷达有效下限。MCU仍保留CAN/驱动故障、15RPM反馈超速、电流、驱动500ms看门狗、硬件IWDG、停车与备用停车保护。故障锁存后不自动重新启用；收到STOP的应答也不能代替实测停车结果。

CANopen SDO没有事务编号，任何超时之后的同对象迟到ACK无法可靠归属。本版因此锁存`sdo_ambiguous`：继续尝试物理停车，但不把后来回包升级为原停车证明，odom保持失效；显式STOP、置零或新任务不会清除此类故障。需要先调查原因、确认实际停稳，再重新启动MCU，不自动重跑队列。

## 精简与构建

原工程虽声明512KB Flash，本版链接器应用区严格为前32KB，并预留4KB栈。生产目标仅链接`board_g474.c`、`drive.c`、`mq.c`、`wire.c`与必要HAL；测试程序、旧v9控制、固定调试mailbox及长诊断分页不进入生产镜像。测试目标以`MQ_HOST_TEST`单独编译真实生产C及模拟CAN，生产目标使用`MQ_PRODUCTION`，两种宏不能同时启用。未整文件丢弃旧`bench_test.c`中的实际保护逻辑，其必要部分已经重构。

从仓库根目录运行：

```text
python -B mcu/build.py --host-tests --name v10-host-check
python -B mcu/build.py --firmware --name v10-production
```

输出目录必须是新名字，结果在`build/名字/`，含测试日志或生产`mcu-v10.bin`、`.elf`、`.map`及SHA/内存报告`receipt.json`。构建脚本没有连接设备、烧录或启动功能。32KB超限即失败，不扩大应用区。

Linux主机测试使用`cc/gcc/clang`；Windows默认使用本机Unity的Emscripten/Node，路径可由`--emscripten`指定。生产构建使用STM32CubeG4 1.6.3及ARM GCC，路径通过`--sdk`、`--compiler-dir`指定；保持Cortex-M4、16MHz HSI、soft-float，与原硬件引脚及CAN500kbit/s配置一致。SDK不复制入仓库，构建报告记录实际SDK源与工具SHA。

## PC工具

安装PC端`pyserial==3.5`后，监听命令如下；端口必须显式填写，模块导入和默认监听均不发送命令：

```text
python -B mcu/tools/mcu_v10.py --port COM_PORT
```

串口是半双工。写命令需提供有测量来源的USB/调度延时预算，工具建立MCU/主机时钟偏移区间后，只在可容纳整帧的授权窗口发送。具体字段、单位、CRC及预算文件见[线协议](docs/PROTOCOL.md)。这不是桌面USB硬实时时序保证，后续必须静止测量真实链路；不要把示例值当已测上界。

```text
python -B mcu/tools/mcu_v10.py upload --port COM_PORT --timing-budget measured.json --queue mcu/examples/queue.json
python -B mcu/tools/mcu_v10.py start --port COM_PORT --timing-budget measured.json --task-id TASK_ID
python -B mcu/tools/mcu_v10.py stop --port COM_PORT --timing-budget measured.json
```

`queue.json`只是协议示例，构建、监听、上传都不执行它。`reset`是停稳后的显式MCU odom置零，会增加odom代次并撤销旧队列；下一阶段AMCL只建立map→odom关系，不需要每个目标都把MCU置零。

新固件仅接受v10，不能配第一分支的v9程序控制。原RAR、第一分支及旧固件作为回退基线保留，来源身份见`provenance.json`。部署流程及实际中止结果见[隔离部署](docs/DEPLOYMENT.md)，零发送采集与判定范围见[静止验收](docs/STATIC_CAPTURE.md)。静止通过后才单独安排架空动作和低速地面验收。

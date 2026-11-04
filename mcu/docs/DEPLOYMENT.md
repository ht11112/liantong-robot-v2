# v10隔离部署与静止验收

本阶段只备份、部署已冻结的29,292B生产镜像和零发送静止采集，不上传队列或START。固件仍是原交付SHA `820d0735b2ee43c4add6a1435532c4f9e2e1dab735cb5c9b54a846e50a0cf878`，ELF为`725b523e2746b6e0444305779ef4571de8172f8804ac765ff3f74948248a7f4b`；不因部署工具新增而重建或替换待写镜像。

## 工具边界

`mcu/tools/deploy_v10.py`提供`inspect-backup`、`deploy`和`reset-audit`。默认核本地镜像、原17源构建清单及必要的新ELF布局；没有`--execute`不会构造Programmer或访问目标，也不建立输出目录。生成证据放仓库忽略的build/output或工作区独立目录。原历史部署脚本和失败记录保持。

执行参数包含显式ST-LINK serial、镜像、ELF、构建收据、新输出目录和本轮现场事实收据。现场JSON为`mcu-v10-site-1`，带UUID、时区时间、操作者原话及实际布尔事实，不是替用户自动生成的隔离确认。共同要求停稳/无人；隔离模式还要求电机动力断开、CAN与RS485断开、MCU安全独立供电。恢复模式要求CAN/RS485接通、驱动电源稳定、控制口无旧程序占用。

当前工具限定G474 Device ID `0x469`、512KB、DBANK=1及明确的启动/保护配置。DBANK=1时为2KB页，前32KB即页0–15；遇DBANK=0或其他配置会拒绝，不改变Option Bytes，也不执行mass erase。UID、原始OPTR、完整Option Bytes及ST-LINK身份均独立读取绑定。

## 备份和写入

1. `inspect-backup --execute`取得两份512KB独立完整Flash，并要求逐字节相等；记录真实UID/options和SHA。结束时读取当前DBG冻结寄存器，只清IWDG bit12，其余位保持，最终同次连接写后读回。
2. `deploy --execute --backup-receipt ...`要求同现场会话、同目标及已成功清理的备份。写前再读完整512KB与本轮备份一致；重新核镜像SHA，将镜像补FF至32KB，先落盘/fsync操作意图，再显式擦页0–15并下载、Programmer校验。
3. 两次独立全Flash读回必须等于“新32KB前缀+原备份其余区域”，且UID/options不变。工具不会在该步骤复位。失败保留收据和FAILED_LOCK，不自动重刷、复位或回退；旧20KB部署脚本不适用。

不要依赖不同Programmer调用之间CPU持续halt。整个编程与首启阶段依靠真实电机动力隔离；清IWDG调试冻结也可能允许硬件看门狗自行复位。收据分别记录请求过的操作和实际读回结果，不用CLI退出码替代完整验证。

## 两次启动与静止采集

先在隔离条件下运行`reset-audit --mode isolated --execute --deployment-receipt ...`，验证新程序启动、非零且改变的boot身份、MCU时间推进及空队列。CAN隔离会产生预期SDO超时和不可用odom，这不是健康或实物停车证明。

随后按操作者当次确认恢复驱动电源、CAN和RS485，先让驱动稳定，再执行`reset-audit --mode healthy`。它要求当前仍是同一已核镜像/UID/options，复位前没有执行中的任务，复位后必须是新boot。新ELF解析真实变量地址和字段尺寸（包含1字节枚举），不沿用v9固定mailbox。健康检查包括Drive STOPPED/error0、stop_verified/guard_valid、odom连续且来源新鲜、零速、MQ空队列及未接收协议命令。

纯监听时MQ的`physical_stop_confirmed`可能仍false，因为上电自动STOP属于Drive层；因此启动停车确认由上述Drive实证提供。最终清冻结的同一连接还读取RAM，要求boot/epoch与健康实证相同；之后不再连接SWD，以免重新影响冻结位。

RK只加载`passive_capture.py`及`mcu_v10.py`，独占串口，收原始字节和接收时间。采集绑定最终SWD健康实证的boot/epoch，默认留65秒捕获余量以覆盖至少60秒完整帧；具体分析口径见静止采集说明。它不发送任何v10命令，不以RESET、丢掉坏帧或清理缓冲掩盖问题。报告会分别记录主机接收和MCU发布/测量时间，不把接收抖动当MCU漏积分。

## 离线回归

`python -B mcu/build.py --host-tests --name 新名字`运行原生产C测试及新增部署/采集Python测试。部署测试使用内存Programmer peer，覆盖错目标/选项、双备份不等、写前变化、部分写入、verify成功但独立读回不等、外区变化、no-op reset、新ELF布局及最终清冻结读回。采集测试验证没有发送能力、坏CRC/噪声/身份变化/缺源等不被升级为通过。测试不接设备。

## 2026-11-04实际执行状态

本轮已取得当前隔离事实、识别ST-LINK/MCU、保存两份完全相同的512KB备份，并在第三次写前全读一致后更新前32KB。Programmer自身校验成功，但首次独立完整读回在`0x080044A8–0x080044B7`出现16字节全零窗口，其中14字节与候选不同；其余程序区、FF尾部、32KB外和Option Bytes均符合预期。工具保存失败锁和原始日志后退出，没有请求复位、重刷或回退，最终清IWDG调试冻结并同次读回成功。

随后只读诊断的两份512KB及交叠128字节均匹配预期；期间没有重新写Flash或请求复位。这不能说明首次读回异常已定因，也没有覆盖原部署失败收据。本轮保持隔离，不恢复驱动、不启动后续静止验收；RK仅收到校验过的采集工具，尚未打开控制串口。完整现场身份、备份和原始日志保存在工作区独立证据目录，不提交仓库。

新增工具离线回归为5套生产C测试与46项Python测试，均通过；继承应用178项和手推12/9来源核对通过。这些结果不代表实机静止验收通过。

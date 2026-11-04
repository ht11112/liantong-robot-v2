# v10离线交付验证 · 2026-11-04

本轮只完成MCU及协议工具，没有打开实车串口、连接RK、刷写或执行实车动作。当前8766服务未响应，历史机器人位置和停车记录未作为本轮现场依据。RK规划、AMCL及网页接入仍是后续范围。

最终统一构建：`python -B mcu/build.py --host-tests --firmware --name release-r2`。五套真实生产C测试包含35个场景，另有9项Python测试，全部通过、无跳过；Python实际消费C编码器生成的金样。保留应用178项回归通过，原12手推来源及9项r5身份保持，Python3.8 AST及GCC的`-Wall -Wextra -Werror`语法检查通过。完整源SHA、工具/SDK身份和输出在生成的`build/release-r2/receipt.json`，不将生成日志或固件提交Git。

生产BIN为**29,292字节**，32,768字节应用区内剩余3,476字节；静态RAM2,204字节，另预留4,096字节栈，余92,004字节。BIN SHA256：`820d0735b2ee43c4add6a1435532c4f9e2e1dab735cb5c9b54a846e50a0cf878`；ELF SHA256：`725b523e2746b6e0444305779ef4571de8172f8804ac765ff3f74948248a7f4b`。生产符号中没有旧mailbox、v9 ARM/SETPOINT或测试入口。

跨层使用真实Drive+MQ及独立CAN peer，正负90°的MOVE0.5m→TURN→MOVE0.5m队列均在完全无主机续令时完成。模拟单次执行87,750ms、取得1,760个CAN读速源、最大间隔50ms；这只验证模拟调度及生产状态机，**不是实车20Hz时序或行驶精度实测**。

故障覆盖包括：1200秒总期限及tick回绕、重复/旧任务与START、部分队列与CRC、段间旧非零ACK与五个新零源、负TURN跨pi、MOVE超过8°仍按中心轮程执行、到点在途ACK时直接STOP、停车继续积分、CAN错误/超速/电流/看门狗异常、慢SDO来源公平性、同对象迟ACK、STOP发送FIFO永久不可用、UART错误与RX溢出锁存、半双工忙/半帧/延迟发窗、旧ACK干扰后客户端续号以及C/Python字节互验。

为避免将不可归属的迟ACK升级，任何SDO超时锁存`sdo_ambiguous`；仍尝试物理停车，但原任务、odom及停车证明不因后来回包恢复。STOP额外有2秒总截止，CAN无法发送时也不会永远停在STOPPING。UART/RX溢出具有独立锁存位，即使先前首因是取消，停车完成后也不能通过BEGIN、RESET或缓存START绕过；STOP与QUERY保留。

仍待后续实机验证：实际RS485/USB延时预算和收发方向、20Hz实际样本/发布周期、STOP期间采样、驱动保护与备用停车，以及轮径/轮距标定和低速地面精度。新固件只接受v10，第一分支程序仍需配原v9固件；未自动执行升级或回退。

# MCU v10：累计odom、原地转向与直行队列

此协议替代v9查询/轮速给定协议。生产固件只接收v10；旧网页及v9客户端不能控制v10。PC工具默认仅监听；本目录的测试、构建均不打开串口或烧录设备。

## 执行与身份

MCU持续采纳每个不同的真实轮速样本并累计odom，不依赖上位机收到全部遥测。odom测量seq/tick来自真实CAN采样完成时刻；20Hz发布seq/tick另记，重发旧odom不会改变测量时间。轮速单位0.1RPM，右电机原始正方向与物理前进方向相反；x向初始车头前方、y向左，yaw逆时针为正，单位米/弧度。yaw始终在[-pi,pi]，TURN使用MCU内部连续角累计。位置原始编码器为uint32，不被软件置零改写。

这里的measurement_tick_ms具体是MCU接收到读速SDO响应的完成时刻，sample_sequence是本机采纳序号；原驱动响应不含其内部采样时间，不能宣称两者是驱动器内部传感器时钟。CAN响应已接收、主循环稍后处理时，仍保留原接收完成tick。

boot_id是每次MCU启动的新64位身份；odom epoch在显式静止RESET时变化。task_id是boot内严格增加的非零uint64，不能回绕；普通请求ID为非零uint32，按半范围顺序比较，可从FFFFFFFF到1，不能跨boot复用。遥测分别返回最近ACK的request_id及真正已接受的request水位。**新客户端仅从accepted_request_watermark+1续号**，跳过0；拒绝ACK（包括错boot、旧请求、过期grant）不得成为水位。一次只有一个在途请求，一份完整队列在COMMIT前没有执行权限。

上传按BEGIN→按index顺序APPEND→COMMIT。COMMIT核数量与整个canonical队列CRC32，只进入READY；START必须另行显式发送并指定同task/boot/epoch。重复请求ID必须除grant/CRC外逐字节相同，返回原结果而不再次调用动作；较旧request拒绝，重复START不重跑已开始/已结束任务。新task不得低于已有水位；同task仅原metadata可幂等。RESET清除队列且改变epoch，不清boot内task或request水位。

MOVE段value是正向距离m，TURN段value是有符号角rad（左正右负）。canonical段正好8字节：`uint32LE kind`（1=MOVE，2=TURN）+`IEEE754 float32LE value`，没有C结构体padding。MOVE范围(0,1000]m，TURN非零且绝对值≤2pi；最多128段，整个任务duration_ms在(0,1200000]。CRC32为IEEE reflected/zlib算法（poly EDB88320，初值FFFFFFFF，末异或FFFFFFFF），覆盖顺序拼接的所有8字节段。例MOVE0.5m、TURN-1.5rad的CRC32为`615AABE9`。

MCU按真实odom执行，MOVE两轮等速，不纠偏、不因超过8°自行停车；TURN闭环累计角。偏航/横向偏移可记录为诊断。段间换轴的零目标应用及五个不同真实零样本交接保留；最后到点直接STOP，没有额外到点零速封存或停后位置复验。停止动作和物理停止确认分别记录。PC/RK断开不会中断已START的队列，但也没有RK雷达动态停障；MCU内部CAN/驱动/odom/任务期限保护继续有效。

UART传输错误与RX环形缓冲溢出在同boot内永久锁存。首次触发立即停止；即使随后物理停车通过、CAN恢复且新任务健康，除STOP/QUERY外的BEGIN/APPEND/COMMIT/START/RESET仍返回UNHEALTHY，不调用任务操作。RESET、换task及旧成功请求重放均不能解除此锁存；必须排查原因并重新启动MCU。STOP和QUERY仍保留各自CRC、boot及请求防重放要求。

## 帧与半双工窗口

115200、8N1、半双工RS485，小端序。每帧：`A5 5A | version u8=10 | opcode u8 | payload_len u16 | payload | CRC16 u16`。CRC16为CCITT-FALSE（poly1021，初值FFFF，无反射/末异或），覆盖version至payload末尾，排除magic和CRC；`123456789`金样为29B1。普通命令36字节，BEGIN/APPEND46字节，任何命令不超过64字节。

MCU按自己的固定50ms周期尝试一次110字节遥测（线速约9.55ms）。整数tick相位仅0或1ms才允许起发，含1ms tick量化留出0–12ms遥测槽；发送前再读时钟，构帧后tick变化则本次不发送，board在HAL IT调用前核expected_tick。TX还要求无在途TX、无RX积压/半帧、最近RX安静至少2ms。HAL IT异步发送，真正TC后才释放DE，ISR不运行MQ或解析协议。错过发送窗口就丢该次遥测，不排队补发、不影响odom累计。持续RX/总线冲突可能使实际发布低于20Hz，不能称为已实测硬实时20Hz。

每个遥测授予同周期14≤phase<38ms的普通命令窗口，grant token就是publication_sequence。open/close字段是相对publication_tick的整数ms偏移（起发延迟1ms时为13/37，准时为14/38）。一个grant至多执行一个新请求，完整命令必须在窗口内被主循环校验；过期请求拒绝且不消费request水位。ACK捎在下一次成功遥测中，保留到下一请求；没有额外抢总线的ACK帧。

有效STOP要求CRC、正确boot、合法新request ID及完整形状，但**不要求当前epoch/grant/普通时隙**；可停当前任务并废弃剩余队列。重复或旧request不会再次执行STOP。主机仍应避免与遥测碰撞：工具优先使用量化时钟证明可容纳的14–38ms窗口，也允许38–48ms安静尾窗，保留下一遥测前的余量。若无法获得可证明的空闲窗口，工具拒绝发送并报告，不能把未送达的STOP当停车确认。

## 命令payload

所有命令共同28字节：boot_id u64（帧offset6）、epoch u32（14）、request_id u32（18）、grant pubseq u32（22）、task_id u64（26）。payload中的boot/epoch不得用旧会话值猜测。下面offset均从整帧开头计算。

| opcode | 含义 | 额外字段 |
|---|---|---|
|1|BEGIN|count u16@34、canonical CRC32 u32@36、duration_ms u32@40|
|2|APPEND|index u16@34、kind u32@36、value float32@40|
|3|COMMIT|无|
|4|START|无，task必须等于已提交任务|
|5|STOP|无，task/epoch/grant仅作来源记录、不限制当前停车|
|6|RESET|无，仅静止健康时软件odom置(0,0,0)，废弃队列|
|7|QUERY|无，仅ACK/状态查询；通常直接监听即可|

result：0=OK、1=INVALID、2=BUSY、3=IDENTITY、4=ORDER、5=CRC、6=UNHEALTHY、7=STALE、8=EXPIRED。ACK只说明命令处理结果；START ACK不表示段已完成，STOP ACK不表示已停稳，COMMIT ACK不表示已发车。

## 遥测payload（opcode128，总长110字节）

| 整帧offset | 字段/类型 |
|---|---|
|6/14|boot_id u64 / odom epoch u32|
|18/22|publication_sequence u32 / publication_tick_ms u32|
|26/30|真实sample_sequence u32 / measurement_tick_ms u32|
|34|task_id u64|
|42/46|ACK request_id u32 / result u8|
|47/48|MQ state u8 / flags u16|
|50/52|当前段index u16（从0开始）/总段count u16|
|54|当前段progress float32（MOVE为m、TURN为rad）|
|58/62/66|累计x/y/yaw float32|
|70/74|最新实测v(m/s)/w(rad/s) float32|
|78/82|原始左右编码器 uint32|
|86/88|原始左右轮速 int16（0.1RPM）|
|90/92/96|首个MQ错误 u16 / 原驱动错误 u32 / 首因tick_ms u32|
|100/102|grant open/close相对publication_tick偏移 u16 ms|
|104|accepted_request_watermark u32；grant隐式使用offset18的pubseq|
|108|CRC16 u16|

state：0 IDLE、1 LOADING、2 READY、3 ARMING、4 MOVE、5 TURN、6 BARRIER、7 STOPPING、8 COMPLETE、9 FAILED。flags：bit0 odom已初始化、bit1 odom连续且测量源龄<250ms、bit2正在执行/停车、bit3已触发到点、bit4物理停车已确认、bit5串口错误锁存、bit6队列已提交、bit7该task已START、bit8 ACK字段有效。没有样本时不得把发布tick作为测量tick，源龄需uint32回绕差值判断；ROS转换应保留MCU boot/epoch及原测量seq/tick。

## PC工具与时间预算

`python mcu/tools/mcu_v10.py --port 实际端口`只监听。写操作必须显式选择`upload/start/stop/reset/query`且提供`--timing-budget 实测预算.json`。此JSON必须含`rx_latency_min_ms`、`rx_latency_max_ms`、`tx_latency_max_ms`、`clock_drift_ppm`和非空`provenance`；数字必须来自本次USB转换器/操作系统/读取方式的实测保守边界，文档不提供可冒充实测的默认值。RX边界包含最后线缆字节到Python读返回的USB/驱动/线程延时；TX最大值包括最后窗口复核到MCU首字节的调度/USB/驱动延时。

工具至少用5个不同新遥测建立host−MCU时钟偏移区间；减去110B的实际线速时间，并计入1ms tick量化、实测收发延时和时钟漂移。只有整个命令线速时间、TX最坏延时及1ms保留量都容于当前授权区间才发送。区间矛盾、旧缓存、重复pubseq、boot变化、源过旧或窗口不足均不续鲜、不发送；没有自动重传START或其他动作。桌面Python与USB不是硬实时，边界失效或超窗时固件还会拒绝普通命令；仅ACK/人工观察可判定真实结果。

队列JSON形状：`{"duration_s":1200,"segments":[{"kind":"move","distance_m":0.5},{"kind":"turn","angle_rad":-1.5}]}`。上传工具从真实遥测选择current task_id+1，按float32实际字节算CRC，COMMIT后返回READY而不自动START。随后必须独立`start --task-id 已返回ID`。进程中断/ACK丢失不自动继续上传或发车；先监听核对当前MCU身份、任务和状态。RESET不会给旧地图定位赋新意义；未来RK的AMCL/map→odom/A*队列桥另行实现，本轮未包含。

构建入口`python mcu/build.py --host-tests --name 唯一名称`先运行真正生产C codec，再把C输出金样提供给Python测试；覆盖字节布局、CRC、canonical队列、分帧/噪声、grant/反重放、首因、测量/发布时钟分离及USB预算拒发，不访问串口。

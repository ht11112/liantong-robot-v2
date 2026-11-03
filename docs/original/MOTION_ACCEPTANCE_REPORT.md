# 独立半米行驶验收

2026-11-03。用户已授权实车验收：当次车头前方0.5米，到点误差10厘米，禁止倒车，不调整终点朝向；速度上限0.03m/s、6.9°/s，前方180°扫描，车体前缘20厘米停障，主机有效反馈来源龄750毫秒，任务最长200秒。用户确认车上无人、周围有空位、在旁可人工停车，并同意用初始车体前缘前方0.5米地面标记测量实际余距。

本次使用新的独立单次runtime，不依赖旧网页。未修改旧启动服务、原地图或MCU固件。轮径0.20米为厂家标称，轮距0.30米、车体与雷达安装仍为估计；AMCL和RPM积分不代替地面测量。额外工程参数只适用于此次有人看护的短程试验，其余全局现场值仍待填写。

## r1：零目标ARM后监督源过期，未行驶

- 固定包46个绑定文件，archive SHA256 `71e91424d5fef3e7ee42e73a3f36f954c4428118863e3c1ac460ab67bd745b38`；PC和RK实际Python3.8.10分别通过同一237项软件检查、0跳过。最初配置回归仍断言只有旧6项用户选择，按新9项确认及条件可选项修正测试后通过；原失败日志保留。
- RK目录 `/home/cat/p2p_navigation_20261103_trial_114714_5c1e069e`，boot `d6a4e5d4-a2fd-4d3f-b768-9cce9bc20e49`。重新停车建立静止锚点，从新的扫描全图搜索并发布种子；五个post-seed新源及单次路径检查通过，目标从当轮位姿计算，没有复用历史预览起点。
- 真实ARM恰好1次；READY为零目标，**SETPOINT为0次，motion_started=false，completed=false**。2264个完整请求对应2264条响应；467 STATUS、2 STOP、1794 FEEDBACK、1 ARM。全部1794个原始轮速及左右编码器均零。
- 首因是 `ROS supervisor source expired`，发生于monotonic8461.676517974。最后pre-ARM真反馈源8461.175741522，其0.5秒TF期限8461.675741522，与失败相差0.776毫秒；ARM阶段8461.231311548至READY8461.624236090只查询状态。READY后新反馈已经收到，但ROS/TF缓存未及时替换旧源，符合衔接空窗机制。r1没有保存失败瞬间health快照，因此不能唯一排除并发监督心跳过期，后续增加原始各源年龄诊断。
- 最终STOP ACK2242→失能零目标STATUS2246→84616～84620五个不同新零速源，停车通过；原失败未清除。全部自有ROS进程SIGINT正常退出，控制口释放；实际清点仅基础ROS/雷达/语音，只有雷达占USB0。46个源文件前后SHA一致。
- 原始22文件在[actual_trial_r1](../../output/p2p_redesign_20261103/actual_trial_r1)，证据tar SHA256 `6650b05d2aba5a8805981073be4fe3b97d745be2ec229e40bf01d8b8254f9c5b`；独立逐原始字节审查见[actual_trial_r1_independent_review.json](../../output/p2p_redesign_20261103/actual_trial_r1_independent_review.json)。

## r2：准备阶段未返回路线，未ARM；后续独立停车恢复

- 新46源包SHA256 `c256cc2996aa1f234a7318fbdb3731b7fe2adc6d5469f66a90b546b984f72134`，PC和实际RK分别241项通过、0跳过。修复为零目标READY后取得真反馈，等odom／TF覆盖该源再允许运动；最多0.20秒且预留原500毫秒租约中的新命令时间，不改state2/4反馈、不重盖旧来源时间。
- RK `/home/cat/p2p_navigation_20261103_trial_115650_af1968c7`。本轮新扫描、seed、五个新定位源通过；make_plan未返回路线，ARM／SETPOINT均0次，未行驶。r2没有触及新READY交接阶段，不能把其软件测试写成该环节实车通过。
- 发现准备错误被恢复分支吞掉，且nomotion提前停止，最终100秒准备超时。规划器在定位seed前就启动，可能留下错误初始位姿的障碍图；缺失败当时完整grid，不能唯一归因。后续修正启动顺序并保存准备首因和完整原图，不清除障碍或允许未知区域。
- 该轮最终STOP没有在原300毫秒内收到回执，`final_stop_error`保留，原停车证明不通过。之后独立新STOP恢复成功：1次STOP、6次STATUS、15次FEEDBACK，91356～91360五个不同新零速源、失能零目标状态；控制口正常关闭。该恢复不改变r2失败结论。
- 24个原始文件在[actual_trial_r2](../../output/p2p_redesign_20261103/actual_trial_r2)，证据tar SHA256 `0cf5b3b06ae1cee48fee8aebeda52eb5bed2c0ee268eee51188f876087e30a78`，46绑定源前后不变。最后实际清点只见基础ROS／雷达／语音，控制口空闲；四自有ROS子进程正常退出。

## 当前状态

半米到点验收尚未通过。用户整车重启后UART故障已清，r5未ARM；r6实际发出19个非零给定，约1.30秒后因AMCL原源超过2秒而停车。轮速与编码器只有很小变化，不能据给定已接受推定实际行驶距离。恢复STOP及另一次独立五新零源证明通过，末次清点控制口空闲。正在修低速里程计与AMCL真实扫描更新的衔接；等待用户按原地面标记测量余距，再确定后续目标。

## r3与r4：规划通过，静止定位更新生命周期仍有缺口

| 轮次 | 软件包与实际结果 |
|---|---|
| r3 | 46文件，包SHA `a8fa621f7feb67870cecbf38c520bea4181846cd1ab7f1546e44bc6e0545b4f5`，PC／RK各245项过；唯一路线23点、0.526794米，58个中心采样格代价0。准备记录2.15776秒，实际重新验证AMCL时原源已3.93924秒，正确触发2秒门。ARM／SETPOINT均0，初末STOP五新零源通过 |
| r4 | 46文件，包SHA `faa10efbd146e1399a2fb6b56abe44745c002f34bebc4995afe6170a56634fe2`，PC／RK各247项过。准备结束后重新请求真实AMCL，0.1744秒内获新源、全源复核通过；随后等待DWA可表达命令时提前停nomotion，2秒后AMCL再次过期。ARM／SETPOINT均0 |

两轮的源文件均前后SHA一致，原始文件独立封存。r3证据tar SHA `773bec113ca35a9a442f3d19d03b3eeb65fd1ae2dfdf0f003a0c5168af074021`；r4为 `2c0341103ae90c4b84cb9f7946f77ece447781295425228f944805bea548356a`。r3的`first_bad_pose`是历史seed前数据，真正失败源为seq13，不能混淆；独立审查另存准备时序说明。

r4最终首STOP2480无ACK，唯一追加STOP2481有CRC有效ACK，但UART critical=1、error81；STATUS2482因此被原Client正确拒绝。之后单独新STOP也不能完成健康停车证明。之前反馈均零，故障后返回session0／target0／stop_verified1，但这些字段不能替代缺失的完整健康STOP链。源协议中的UART计数为MCU生命周期锁存，所有DIAG页均只读，STOP不清计数、ARM必须拒绝。

实际只读DIAG0／1各查询一次、保存原TX／RX：critical1、unknown1，ORE／FE／NE／PE／TX error等计数均0，首末事件tick9971276。该时刻靠近停止收尾重叠，但unknown不能唯一证明总线物理碰撞。没有清错、复位或运动命令。记录见[actual_uart_readonly_diagnostic_after_r4.json](../../output/p2p_redesign_20261103/actual_uart_readonly_diagnostic_after_r4.json)。

后续最小修复是让nomotion保持整个真实`waiting_ready`阶段、在`arming`前原子关闭；仅在本轮从未ARM／SETPOINT的静止等待故障时，撤权后按原期限收完原2／5查询，再发送STOP。已ARM阶段的故障停车策略不放宽。现场复位后仍须新STOP／五零样本、当前扫描与重新定位，不能直接沿用上述历史结果。

这两项修复已完成并经配对独审。新46源包SHA `0df977ead701bb7ad28e10b210816b6529a71760b619f483ac3050f0326b9c7c`，PC完整251项测试通过、0跳过；runtime `6e041ba0…`、owner `49ded8c6…`。新增有界preARM原始命令／量化结果／拒绝原因记录，原始接收时间不刷新。该包随后在用户重启后绑定新boot、通过实际RK251项测试并用于r5；历史四轮失败未覆盖。

## r5：重启后停车健康，扫描匹配拒绝，未ARM

新boot为`2fee0fae-658d-4245-9bd8-fc65e39f297b`。独立STOP预检及末STOP的五个不同新零源均通过，UART critical=0。r5新扫描、全图seed、五源定位、22点/0.504300米路径通过；随后某个三扫描窗口未达到原60端点或0.65匹配比例，未ARM／SETPOINT，全部1941个轮速和编码器样本为零。失败窗口原ranges未保存，不能唯一推断人或家具移动；用户随后确认已站后方、前方无人且物体未移动。

首因前DWA曾有两条正向可量化命令，不能写成“规划一直零”。未ARM故障时原只读pending在原期限内收完，随后STOP成功。24个原文件在`actual_trial_r5`，tar SHA `9ea637e5d94e4fd4cc8608c637f822f5f2c78ccf3ce874ca4fe7518c17974944`。静止anchor末回调另有序列gap记录，不改变真实停车链或该轮失败。

## r6：真实给定约1.30秒，低速里程计归零使AMCL超龄

- 新46源包SHA `aaeda79403ef7417d1bba39593e47438c6d6d9d0faf049c55eecd382eca9a233`，PC／实际RK各252项过、0跳过。costmap TF容差显式对齐既有0.50秒；增加首坏三扫描完整原始证据，匹配门不变。新的seed、五源与路径均通过，READY后新反馈／odom／TF衔接通过。
- ARM一次，19个非零目标均被接受，18个有后继应用证明，最后一个在应用前因源故障取消。给定间隔51.665～77.366毫秒。36个armed反馈只有0或0.1RPM，编码器左0→8、右0→1；这不是半米实际位移证据。
- 失败时AMCL原源龄2.00356874秒是唯一超龄项；heartbeat、odom、scan、TF均在原门内。本轮无扫描匹配失败。冻结里程计的`abs(RPM)<0.2`死区吞掉了全部0.1RPM，odom积分保持零，AMCL自然更新阈值没有触发。未扩大2秒或乘十修改反馈单位。
- 原末STOP证明失败保留：STOP后STATUS已健康，但第一反馈四个CAN来源仍早于该STATUS14～38毫秒，其中旧left=-0.5RPM被误作新停车读数。一次恢复STOP→新STATUS→五个真正后置新零源通过；另独立STOP的五新源8806～8810全raw零、UART0。此恢复不把失败轮改成通过。
- 2590个完整原TX对应2590个CRC有效原RX；业务行2589是取消时原STATUS2555未提升为有效观测，原字节保全。46源不变，四自有子进程SIGINT退出0，串口正常关闭。mono1444.39096382最终清点只见6基础进程、雷达USB0 owner，控制口空闲。

24个原文件在[actual_trial_r6](../../output/p2p_redesign_20261103/actual_trial_r6)，tar SHA `b02293f3d2ade5123cfb4789c04f4bf59485dd45440543499079610dcc097264`。详细原始字节独审见[actual_trial_r6_independent_motion_review.json](../../output/p2p_redesign_20261103/actual_trial_r6_independent_motion_review.json)。

后续软件分两处：停车证明先排除旧CAN来源，再使用原五新零门；独立trial里程计保留真实低RPM，并定期触发下一个真实扫描测量。官方AMCL服务只设强制更新标志，随后仍用真实odom delta和原扫描时间，不能用服务时刻刷新旧pose。静止噪声、协方差及三扫描检查仍须通过；当前修改尚未构成新RK或行驶通过结论。

# 离线与历史实测验证

## 指令应用回包与停车采样修复

实车旧任务`9ed7b073-a799-469f-b230-be6525193db2`在转直控制中报告“Accepted target not observed applied within 300ms”。原450次请求、450帧响应与450业务记录CRC、序号、原字节一致。seq422在6010.736157016秒被接收，seq425回包在57.071ms后已报告applied422；此后SETPOINT回包持续报告应用进度，旧程序只消费STATUS，后段STATUS被反馈查询挤占，312.334ms后误报超时。

本次修复统一消费STATUS与SETPOINT回包中的应用序号、目标值、session和身份；当前已应用项与待应用项保留有界，不把接收ACK当作应用。反馈与状态查询采用到期先后调度，300ms真实未应用门不放宽。

旧任务STOP438已发并有回包，健康失能STATUS444也通过。原首个有效postSTATUS轮速为(-0.4,+0.2)RPM，旧程序立即终止，未等完整2秒。本次改为原2秒窗口内等待五个连续不同的postSTATUS样本满足原0.2RPM零速门，非零重置连续计数；状态、CAN来源等故障仍拒绝。原任务停车里程计样本60205→60207、间隔266ms的缺段继续视为位置未验证，不补积分，不重写历史失败。新取得的五个静止反馈只证明新的观察时刻，不覆盖原停车链。

本次PC全app107项通过、0skip，独立包简版49项与remote9项通过。RK新目录`/home/cat/p2p_navigation_20261103_applied_fix_203300_42d19a6b`系统Python同49+9项软件测试通过、0skip。26个app/scripts Python文件通过3.8语法检查，原手推12项来源和r5九源核验通过。

新包为`dist/applied-feedback-fix-20261103-r1/simple-p2p.tar.gz`，SHA256为`b4b9befce920440ba0523948b6b130ee41fa6a8f08e2d52a0a397714ec57e494`，14项文件SHA绑定、15个tar成员。新owner SHA256为`b609a518177d3ee4e5396a75200d82a69081bb674d21b6b1c3ee0be7d8e039e9`。新跟踪器离线重放原123条SETPOINT及相应STATUS回包，所有应用字段和原300ms时限通过；末尾尚未证明应用的seq437仍保留pending。软件审查通过不表示已经重新行驶验收。

网页改接上述新目录，没有自动重跑该实车任务。额外净距仍为0，原750ms来源门、CAN250ms、MCU500ms租约及速度/任务时限保持。

## 取消额外20cm净距：先前检查点，配置仍生效

用户明确要求取消额外20cm停障净距，当前配置为`stop_clearance_m=0`。车体占用与原地转向扫掠范围、原轮速/扫描来源750ms、200秒上限、10cm到点容差及禁止倒车等边界保留。

本次PC全app回归100项通过、0skip，原12项来源与r5九源核验通过。独立包内简版42项与remote9项通过；14项文件SHA绑定，含manifest共15个tar成员。

本次包为`dist/local-p2p-zero-clearance-20261103-r1/simple-p2p.tar.gz`，SHA256为`fdbc6417fef6f7d98e37727ca1a330f0df2bb7ead61654377e9bdda9d2719f54`。RK新目录`/home/cat/p2p_navigation_20261103_zero_clearance_201400_7d634c29`实际运行简版42项与remote9项软件测试通过、0skip，并核配置额外净距为0。网页改接新目录，修改后没有启动行驶任务。下方测试数、包SHA和20cm净距验证均属于变更前的历史版本，不作为当前零额外净距配置的验证结果。

本次修改前，任务`8986…`在ARM交接阶段失败，核对时无活动任务。取消额外净距不构成该故障的修复证明，也不触发自动重试。

## 取消净距前的本地网页与RK任务入口（历史）

当时在 `feature/simple-point-to-point` 增加PC本地网页服务和RK `local_p2p_remote.py`。独立运动包包含remote及它的离线测试，PC服务/UI不进入RK包。CI继续运行全app测试、原手推构建与包内回归，并分别运行简版和remote包内测试。

该历史冻结实现的离线检查如下。网页离线测试不等于运动验收；不能把下方网页开发前的包SHA、35项结果或只读置零观察当作新增网页代码的实车证明。原手推12项来源与其中历史9源身份核验当时仍通过。

| 检查 | 历史结果 |
| --- | --- |
| PC全app离线回归 | 97项通过、0skip：原手推33、简版39、remote9、PC网页16 |
| 独立RK包内回归 | 简版39项与remote9项通过、0skip；二者均是全app测试的子集 |
| 包内完整性 | 13个Python源/测试与1份配置共14项SHA绑定，另加manifest，共15个tar成员逐字节核对通过 |
| Python3.8语法 | 包内13个Python文件及PCserver/测试、launcher、打包器，共17个文件解析通过 |
| 默认配置与PC入口 | 包内默认配置可读取；`python -S -B scripts/run_local_p2p.py --help`正常退出，不连接设备 |
| 原手推来源 | 原12项来源与r5九源核验通过，原手推包构建流程不变 |
| RK实际软件检查 | 系统Python在新目录分别运行简版39项与remote9项通过，0skip；没有据此执行网页任务 |
| 实际网页操作范围 | 静止置零与0.5米目标预览通过；没有启动实车运动 |

该历史包为 `dist/local-web-p2p-20261103-r1/simple-p2p.tar.gz`，SHA256为 `e998142f5d9581aaa8915fd6da90c9519247f9120655e31d8dc55707e50a8263`。RK软件检查目录为 `/home/cat/p2p_navigation_20261103_local_web_200000_a9401d7c`。该检查点在纯PC源龄修复后重启网页服务，并再次完成静止置零与半米目标预览，未启动实车运动。先前CLI零点观察、离线测试及网页预览均不作为行驶验收通过证明。

```text
python -S -B -m unittest discover -s app -p "test_*.py"
python -S -B scripts/verify_source.py --verify-manual-import --verify-r5-baseline
python -B scripts/build_simple_p2p_package.py --name local-web-p2p-20261103-r1
python -S -B -m unittest discover -s dist/local-web-p2p-20261103-r1/package -p "test_simple*.py"
python -S -B -m unittest discover -s dist/local-web-p2p-20261103-r1/package -p "test_local_p2p_remote.py"
```

上面记录历史输出名称；该目录已存在时构建会拒绝覆盖，重建需换一个名称并同步测试路径。

离线回归覆盖设软件原点/预览不ARM、空闲编码器变化后拒用旧参考、执行目标与预览绑定、取消仅作用当前任务、PC重启/失联恢复及原CRC/来源龄/速度量化/停车位姿语义。独审发现跨RK boot旧任务结果可能被短时标为有效坐标，已补boot校验和历史状态回归：旧boot结果不更新当前pose或encoder anchor。恢复核对中的SSH等待已移到状态锁外，另有阻塞SSH时页面仍可读缓存的回归。原odom年龄使用独立单调时间参考，inventory或无新源的响应不再刷新该年龄，包含`.05→10.05→inventory后仍10.05秒`的回归。地图仅只读展示，不纳入控制器或作为局部坐标定位证明。实际现场仍须按当次状态验收。

## 网页开发前的简版CLI基线（历史）

2026-11-03在手推基线分支外新增简版odom控制，没有合入main。原12项手推来源和9源r5身份核验保持通过。

| 检查 | 本分支结果 |
| --- | --- |
| 全app离线回归 | 68项通过、0skip，含原33项和新简版35项 |
| 独立简版包 | 35项通过、0skip；默认配置可读取，13个tar成员逐项核SHA |
| Python3.8 | 11个包内Python源及打包器均可解析；RK系统Python实际运行35项软件测试通过 |
| 最终包SHA256 | `08bc99bfec517388ccf797b2c6a6147c6892d5447f9bee8a9e48c5d2fa917785` |
| 控制边界 | 转/直分离、量化不越速、原反馈去重/750ms、scan原header/750ms、CAN250ms、MCU500ms租约 |
| 历史车体净距（当时配置） | 前180°及≤1°束间距；转向按车体角点扫掠半径加20cm检查，后方仍需人工现场核对 |
| 成功条件 | 转直切换先五个新实测零源；最终STOP五新零并核停车后odom距离；刹车缺段或漂出容差判失败 |
| 已在容差内 | 无ARM时也须原点后编码器始终未变、初末STOP完整，绑定停车末新源再核最终距离 |

实际RK只做了软件验证与一次默认只读零点观察。首次包`975c48fbd0e2c4b3bef64637f142be23e55e0dfc3b010daf28ce08e4cf8f391b`的软件33项通过后，只发STATUS1次和FEEDBACK1次：原反馈两轮0RPM，记录软件`(0,0,0)`，无STOP/ARM/SETPOINT，串口正常关闭、程序退出。该观察不代表常驻odom发布或行驶验收。

随后独审发现“无ARM已在容差内”分支缺少最终位置证明，已补齐并新增两项测试。最终包只改变owner和对应测试，RK软件35项通过；未再次置零或运行运动任务。实际首次置零证据与最终软件证明分别保存，未把首次包当作最终运动代码实测通过。

在上述CLI基线交付检查点，尚未进行原地转向、直行、到点误差或停障的实车运动验收，没有刷固件、改变系统服务、导入地图或启动AMCL。该历史结果不作为后续网页任务的运行状态。GitHub CI保留原手推独立包检查，分支结果以对应Actions为准。

## 以下为手推v0.1.1历史验证

日期：2026-11-03。目标标签：`v0.1.1-manual-mapping-20261103`。

本次只精简仓库范围和说明，不改12个保留app文件的原始字节，不连接RK或操作真实设备。

| 检查 | 结果 |
|---|---|
| 当前来源子集 | 12个app文件与原工作区/首次导入字节及SHA一致 |
| 已实际运行的手推身份 | 原r5九源SHA全一致 |
| 当前测试 | 建图runtime/query和只读viewer，33项通过、0跳过 |
| 独立手推软件包 | 27项通过、0跳过 |
| 包内范围 | 原7运行源+2手推测试+新manifest |
| Python3.8语法 | 11个app/deps Python文件和3个工具解析通过 |
| 当前仓库范围 | 手推程序、4个主机依赖、HTML、3个对应测试及必要工具/说明 |
| 移除范围 | 导航/定位/预览/运动及阶段联调入口、无关配置/依赖/测试和原阶段说明 |

本地使用Windows Python3.10.11标准库运行33项测试，没有加载NumPy、Paramiko或pyserial；测试也不加载ROS。独立包27项是33项的子集，不相加为60项不同测试。

GitHub Actions会运行当前33项、离线构建及包内27项；远端执行结果以Actions页面为准。手推r5的历史实测结果与地图精度限制见BASELINE.md。本次精简没有新增实机验证，也不重写历史标签。

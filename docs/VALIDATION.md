# 离线与历史实测验证

## 本地网页与RK任务入口

本轮在 `feature/simple-point-to-point` 增加PC本地网页服务和RK `local_p2p_remote.py`。独立运动包包含remote及它的离线测试，PC服务/UI不进入RK包。CI继续运行全app测试、原手推构建与包内回归，并分别运行简版和remote包内测试。

本轮冻结实现的离线检查如下。网页离线测试不等于运动验收；不能把下方网页开发前的包SHA、35项结果或只读置零观察当作新增网页代码的实车证明。原手推12项来源与其中历史9源身份核验仍通过。

| 检查 | 本轮结果 |
| --- | --- |
| PC全app离线回归 | 97项通过、0skip：原手推33、简版39、remote9、PC网页16 |
| 独立RK包内回归 | 简版39项与remote9项通过、0skip；二者均是全app测试的子集 |
| 包内完整性 | 13个Python源/测试与1份配置共14项SHA绑定，另加manifest，共15个tar成员逐字节核对通过 |
| Python3.8语法 | 包内13个Python文件及PCserver/测试、launcher、打包器，共17个文件解析通过 |
| 默认配置与PC入口 | 包内默认配置可读取；`python -S -B scripts/run_local_p2p.py --help`正常退出，不连接设备 |
| 原手推来源 | 原12项来源与r5九源核验通过，原手推包构建流程不变 |
| RK实际软件检查 | 系统Python在新目录分别运行简版39项与remote9项通过，0skip；没有据此执行网页任务 |
| 实际网页操作范围 | 静止置零与0.5米目标预览通过；没有启动实车运动 |

本轮包为 `dist/local-web-p2p-20261103-r1/simple-p2p.tar.gz`，SHA256为 `e998142f5d9581aaa8915fd6da90c9519247f9120655e31d8dc55707e50a8263`。RK软件检查目录为 `/home/cat/p2p_navigation_20261103_local_web_200000_a9401d7c`。纯PC源龄修复后已重启最终网页服务，并再次完成静止置零与半米目标预览，未启动实车运动。先前CLI零点观察、离线测试及网页预览均不作为行驶验收通过证明。

```text
python -S -B -m unittest discover -s app -p "test_*.py"
python -S -B scripts/verify_source.py --verify-manual-import --verify-r5-baseline
python -B scripts/build_simple_p2p_package.py --name local-web-p2p-20261103-r1
python -S -B -m unittest discover -s dist/local-web-p2p-20261103-r1/package -p "test_simple*.py"
python -S -B -m unittest discover -s dist/local-web-p2p-20261103-r1/package -p "test_local_p2p_remote.py"
```

上面记录本轮输出名称；该目录已存在时构建会拒绝覆盖，重建需换一个名称并同步测试路径。

离线回归覆盖设软件原点/预览不ARM、空闲编码器变化后拒用旧参考、执行目标与预览绑定、取消仅作用当前任务、PC重启/失联恢复及原CRC/来源龄/速度量化/停车位姿语义。独审发现跨RK boot旧任务结果可能被短时标为有效坐标，已补boot校验和历史状态回归：旧boot结果不更新当前pose或encoder anchor。恢复核对中的SSH等待已移到状态锁外，另有阻塞SSH时页面仍可读缓存的回归。原odom年龄使用独立单调时间参考，inventory或无新源的响应不再刷新该年龄，包含`.05→10.05→inventory后仍10.05秒`的回归。地图仅只读展示，不纳入控制器或作为局部坐标定位证明。实际现场仍须按当次状态验收。

## 网页开发前的简版CLI基线

2026-11-03在手推基线分支外新增简版odom控制，没有合入main。原12项手推来源和9源r5身份核验保持通过。

| 检查 | 本分支结果 |
| --- | --- |
| 全app离线回归 | 68项通过、0skip，含原33项和新简版35项 |
| 独立简版包 | 35项通过、0skip；默认配置可读取，13个tar成员逐项核SHA |
| Python3.8 | 11个包内Python源及打包器均可解析；RK系统Python实际运行35项软件测试通过 |
| 最终包SHA256 | `08bc99bfec517388ccf797b2c6a6147c6892d5447f9bee8a9e48c5d2fa917785` |
| 控制边界 | 转/直分离、量化不越速、原反馈去重/750ms、scan原header/750ms、CAN250ms、MCU500ms租约 |
| 车体净距 | 前180°及≤1°束间距；转向按车体角点扫掠半径加20cm检查，后方仍需人工现场核对 |
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

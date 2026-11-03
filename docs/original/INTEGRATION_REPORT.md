# 新点到点导航：真实路径预览结果

2026-11-03，北京时间。**本轮在新的静止定位窗口中完成“当前位置车头前方约0.5米”的单次GlobalPlanner make_plan预览，机器人没有行驶。** 独立任务服务、导航action执行、DWA运动规划和运动owner仍未实现；结束后的定位与路径是历史记录，不是当前位姿或运动许可。

## 两轮结果与环境修复

| 阶段 | 独立记录与结论 |
|---|---|
| preview r1 | RK目录`/home/cat/p2p_navigation_20261103_loc_105415_2c2a4833`；move_base预热退出-6，make_plan调用0次，预览失败。底盘120秒静止与定位另有通过结果，未将其当路径通过 |
| 插件实际核查 | `/opt/ros/noetic/lib/liblayers.so`实际存在，大小1098464B；`costmap_plugins.xml`存在且声明`library path="liblayers"`。首轮环境`CMAKE_PREFIX_PATH=null`，日志为pluginlib无法找到StaticLayer库 |
| preview r2 | 新目录`/home/cat/p2p_navigation_20261103_loc_preview_r2_110046_f5b633c3`，新nonce与重新采集的定位；启动环境仅增加`CMAKE_PREFIX_PATH=/opt/ros/noetic`，源码和archive不变。单次make_plan通过 |
| 两轮软件包 | 同一39文件archive，SHA256为`a442b168c2cfbf835317e81a24bcdef1bc69cca2f65ada53ee2b608aa7e1ad5a`；PC及各轮实际RK分别运行同一182项测试，0失败、0错误、0跳过，不累加为新测试数量 |
| 实测后源码 | r2读回39个绑定文件，`changed=[]`；首轮失败、第二轮修复和各自原始证据均独立保留 |

两轮均在boot `d6a4e5d4-a2fd-4d3f-b768-9cce9bc20e49`。这是针对明确插件环境故障的新复验，不能把首轮失败覆盖成通过，也不以重复运行挑最好结果。未修改原运行入口、旧地图、MCU或用户待定配置。

第二轮前只清理先前两次试验的5个明确已退出节点。逐项核对同boot、原launch进程退场、实际无对应进程，以及节点API明确ConnectionRefused后，移除60条ROS master注册，耗时2.857596463秒，名单内`entries_after=[]`。清理未发送机器人命令、串口数据、进程信号或重启ROS，也未触及其他namespace。

## 单次路径的实际结果

本次preview在约33.166秒时完成。先等待本轮seed后的五个不同AMCL新源与同owner静止锚点联合通过，再在make_plan前后直接复核当前AMCL、扫描和TF。未从先前结果取历史位姿，未把fresh文件单独当作在线许可。

| 检查 | preview r2实际结果 |
|---|---|
| 规划调用 | 私有`/p2p_preview_f5b633c3_300729/move_base/make_plan`，恰好1次，无重试 |
| 当时定位起点 | 地图坐标`(0.529047961,-0.331863834)m`，yaw=`-1.430531306rad` |
| 相对预览目标 | 沿该时刻车头方向前0.5m，换算为`(0.598950730,-0.826953324)m`；当前朝向仅用于请求，不代填终点朝向策略 |
| 返回路径 | 22个点，长0.512142543m；56次路径中心栅格采样，最大live costmap代价0 |
| 未知／障碍处理 | 请求起点和目标先独立检查；返回路径逐段采样，拒保存地图未知／占据格，以及live costmap未知／致命／内切障碍格 |
| 强补目标／清图 | `make_plan_add_unreachable_goal=false`、`make_plan_clear_costmap=false`、`GlobalPlanner.allow_unknown=false`、请求tolerance=0 |
| 调用前后AMCL源龄 | 255.168／300.281ms；位置标准差约0.03988／0.04039m，yaw标准差约1.039° |
| 调用前后扫描源龄 | 114.956／99.349ms；各82个有效端点，墙匹配66/82与64/82 |
| 调用前后TF一致性 | map→base源龄142.324／89.667ms；与请求起点平移差1.446／0.953mm，航向差0.000875／0.000576rad |
| move_base收尾 | 自有PID300827，SIGINT后退出0，无强杀和清理错误 |

路径使用当轮私有map／odom／base／laser帧和私有话题。move_base速度出口重映射到无订阅者的`shadow_unused`，简单目标入口也为私有unused；sidecar没有action目标、速度发布或串口能力。DWA已初始化但未执行局部运动规划。

本次footprint宽0.50m、轴前后各0.20m来自未实测几何；padding0.03m、inflation0.35m等只用于这次预览。0.10m端点与0.15m段长容差是栅格路径表示检查，**不是用户到点误差、车体净距或新导航默认值**。雷达外参`(0.13,0,0.20)m/yaw=0`仍是安装估计，非标定；前180°之外保持未观测。路径报告明确`physical_clearance_validated=false`、`motion_feasibility_validated=false`，代价0不代表车体实际净空或行驶安全已经验收。

## 同owner静止链与定位反证

| 检查 | preview r2完整窗口 |
|---|---|
| 底盘有效静止窗口 | 120.000397500秒 |
| 定位adapter观察窗口 | 110.050480091秒 |
| 完整请求／响应 | STATUS1123、STOP2、FEEDBACK4451；共5576条，仅2/3/5，均完整20B请求，未丢弃旧行 |
| 实测轮速 | 全部4451个反馈左右均0.0RPM |
| 活动窗口反馈 | 4409个响应、1168个不同MCU源；保守反馈来源龄最大158.310ms |
| 活动查询间隔 | FEEDBACK最大102.779ms、STATUS最大185.402ms；主机750ms与原CAN250ms继续分层 |
| 私有扫描／TF／AMCL | 前180°扫描1653、odom→base TF1064、安装估计静态TF1、AMCL新源270；owner全阶段发布1180帧实测odom |
| 初始停车链 | 新STOP ACK1→健康STATUS7→58262～58266五个不同新零速源 |
| 最终停车链 | 新STOP ACK5550→健康STATUS5555→59438～59442五个不同新零速源 |
| 最终底盘状态 | state1、error0、session0、target0、stop_verified1、UART critical0 |
| 正常收尾 | 仅收完原在途STATUS ticket5549，耗时12.071ms；未发新query、未改源时间或扩大120秒窗口 |
| 静止锚点 | 1173个新源，原始编码器[0,0]，净变化／累计绝对变化均0 counts，轮式位置与航向变化均0 |

**定位并非全程持续合格。** `quality_rejections`共5条：2条seed前源被拒，随后已形成五源联合通过；路径预览完成之后又出现3条AMCL协方差超限，分别记录在monotonic5892.413552715、5893.017703822、5893.322030917。每次反证都清除内存中的最新联合通过状态，后续重新累计五个合格的新源才恢复，不能沿用旧通过状态或将该轮写成全程定位质量稳定。

结束前末组五源为`1793675211.709718`、`1793675212.044789`、`1793675212.377750`、`1793675212.707895`、`1793675212.976823`，全部晚于最后一条协方差失败，联合证据原始validated_at为monotonic`5934.997640070`。它证明末组窗口已恢复合格；未完整留存各中间失效区间或首次恢复时间，不能推断连续健康。preview使用的正是joint_first，至最后一次协方差失败相隔37.228秒。末次历史估计约`(0.4846,-0.3422)m/yaw=-81.90°`；协方差和反复nomotion更新不是地面真值。定位adapter结束后不再产生新证据，底盘继续到120秒并完成新STOP，最终仍只报告历史静止定位通过。

全部物理入口继续永久拒绝ARM和所有SETPOINT，包括零SETPOINT。没有导航action目标、轮速给定、行驶、动态避障或到点验收；没有750ms实车断线故障注入。到点允许误差、倒车许可、终点朝向及其余未填写保护值保持待定。

## 收尾与原始证据

adapter自然退出0，AMCL／map_server由自有进程TERM后退出-15，无KILL及清理错误；preview move_base的SIGINT退出0。最新实际清点仅有6个基础ROS／雷达／语音相关进程，控制USB2无owner，雷达仍使用USB0；SSH工作会话已结束。本轮move_base没有幽灵注册，本轮TERM停止的map_server／AMCL仍留下2个节点的master注册，**不能称ROS图完全恢复**，也不代表它们仍有进程。

- [preview r2路径原始结果](../../output/p2p_redesign_20261103/actual_preview_r2/path_preview_result.json)、[完整底盘与定位结果](../../output/p2p_redesign_20261103/actual_preview_r2/localization_result.json)、[定位adapter报告](../../output/p2p_redesign_20261103/actual_preview_r2/localization_adapter_result.json)
- [本轮RK软件182项结果](../../output/p2p_redesign_20261103/actual_preview_r2/offline_software_validation.json)、[完整测试日志](../../output/p2p_redesign_20261103/actual_preview_r2/offline_tests.log)、[39源文件复核](../../output/p2p_redesign_20261103/actual_preview_r2/post_source_check.json)
- [末组联合证据](../../output/p2p_redesign_20261103/actual_preview_r2/joint_evidence.json)、[单次seed回执](../../output/p2p_redesign_20261103/actual_preview_r2/seed_receipt.json)、[限定注册清理记录](../../output/p2p_redesign_20261103/actual_preview_r2/owned_registration_cleanup.json)
- [首轮原失败](../../output/p2p_redesign_20261103/actual_preview_r1/path_preview_result.json)、[插件异常日志](../../output/p2p_redesign_20261103/actual_preview_r1/path_preview_result_move_base.log)、[真实库与环境核查](../../output/p2p_redesign_20261103/actual_costmap_plugin_probe.json)、[同archive与环境修复记录](../../output/p2p_redesign_20261103/localization_package_preview_r2_record.json)
- [结束后进程／串口清点](../../output/p2p_redesign_20261103/actual_inventory_after_preview_r2.json)、[路径预览入口](rk_path_preview.py)
- [路线示意图](../../output/p2p_redesign_20261103/actual_preview_r2_map.png)：蓝S为当时起点，绿G为前方0.5m目标，橙色为返回路径；蓝框只表示估计车体尺寸。
- [终态独立只读复核](../../output/p2p_redesign_20261103/actual_preview_r2_independent_review.json)：5576条TX完整20B／CRC、来源与odom重放、初末STOP链和末组恢复均通过。原始RX字节、完整live costmap／调用前后scan及完整ROS Path消息未保留，部分接收校验和live栅格／得分只能依据冻结代码与运行报告，不能独立从这些JSON重算。

24个原始文件归档SHA256：`3c747951116bac130671c4dd00df635f20dbc903fed5c50dbe186e29e4b20839`。下一步实现RK本地任务服务、导航action／DWA与唯一运动owner；新的运动尝试需要当次定位、现场条件和完整执行链验证，不能直接使用本轮已结束的历史pose或路径。

---

以下为此前167项保存地图静止定位、115项静止底盘接入及首轮失败／恢复的历史原文，完整保留；其中“本轮”“当前”仅指各自历史阶段。

# 新点到点导航：保存地图静止定位接入结果

2026-11-03，北京时间。**本轮私有TF、前180°扫描、保存地图和AMCL静止接线通过；任务服务、规划执行、运动owner及到点验收尚未实现。** 定位结论只覆盖运行中的历史静止观察窗口，程序结束后没有常驻当前位姿或运动许可。

## 本轮包与现场身份

- RK独立目录：`/home/cat/p2p_navigation_20261103_loc_103525_3e90fdcf`，nonce=`3e90fdcf`。
- 37文件包SHA256：`997d22b4ccf8bc30a73ed2a687928d177877123c48e0a6285544c09d525292f5`；37个文件实测后读回无变化。
- PC和实际RK分别通过同一167项软件测试，均0失败、0错误、0跳过；不将两次执行合计成334项。
- 当次boot：`d6a4e5d4-a2fd-4d3f-b768-9cce9bc20e49`；精确USB身份`1a86:7523/bcdDevice0264`定位到当次`/dev/ttyUSB2`，没有固定tty号。
- 保存地图身份：`9e34ebc9f5a36fbbf64e3a6f7bbef4d23683cd97de1963bba00fe9f60507c746`。包内JSON／PGM及实际map_server栅格一致，没有使用历史机器人位姿作为当前种子。

## 实际采集和停车

| 检查 | 本轮实际结果 |
|---|---|
| 底盘有效静止窗口 | 120.000189734秒；同一query-only owner持续查询 |
| 定位观察窗口 | 110.035713666秒，早于底盘退出，联合定位证据在窗口运行中形成 |
| 全程请求／响应 | STATUS 1126、STOP 2、FEEDBACK 4477；5605条完整记录，只有2/3/5 |
| 全部实测轮速 | 4477个反馈响应左右均0.0RPM |
| 活动窗口反馈 | 4435个响应，1164个不同MCU样本 |
| 活动窗口反馈来源龄 | 最大165.057ms；主机750ms及原CAN 250ms分开检查 |
| 活动窗口查询起点间隔 | FEEDBACK最大138.820ms，STATUS最大193.248ms |
| 私有扫描／TF／AMCL | 1653帧前180°扫描、1060帧实测odom→base TF、1个安装估计静态TF、223个AMCL源 |
| 实测轮速里程计发布 | owner全阶段发布1177帧；1060帧处于较短adapter观察窗口，不将二者混为缺帧 |
| 初始STOP链 | 新ACK序号1→健康STATUS序号7→五个不同新鲜零速源41094～41098 |
| 最终STOP链 | 新ACK序号5579→健康STATUS序号5584→五个不同新鲜零速源42265～42269 |
| 最终状态 | state1、error0、session0、target0、stop_verified1、UART critical0 |
| 正常收尾 | 本轮没有在途query；normal_quiesce约0.141ms，未新增查询或扩大120秒窗口 |
| 静止锚点 | 1169个新源；编码器原始值始终[0,0]，净变化／累计绝对变化均0 counts，轮式位姿和航向变化均0 |

物理边界继续拒绝所有ARM及SETPOINT，包括零SETPOINT；没有下发导航目标。本轮没有运动、动态避障、到点误差或行驶平稳性验收，也没有实车750ms断线故障注入。

## 单次种子与运行中的定位证据

前180°扫描保留真实源时间，后方180°仍为NaN未观测。驱动原始错误`frame_id=map`只在新适配器中改为本次私有laser帧；未把激光端点直接当作地图坐标。使用`p2p_3e90fdcf_map/odom/base/laser`和`/p2p_loc_3e90fdcf/*`独立接线，不覆盖原全局odom／TF帧。

雷达相对轮轴中点的`x=0.13m、y=0、z=0.20m、yaw=0`是**安装估计，未标定**；轮径0.20m、轮距0.30m也未在本轮标定。扫描／odom／AMCL的0.75／0.75／2秒为此次静止工程诊断预算，不是代填用户尚未选择的导航时效。

公开的三帧proposal扫描均来自本轮静止锚点之后。PC按完整保存地图已知空闲格和全航向搜索，精化5个不同粗模式；最佳候选与竞争模式分差0.274076，超过0.08门限，三帧独立墙匹配比例分别约0.8111／0.8167／0.8000。它只通过候选准入门，不是物理位置唯一性的证明。运行时又核对boot、地图、nonce、三帧精确源时间、安装假设与当前锚点，并用另外最新三帧扫描复核，才向私有AMCL发布一次initialpose；当前三帧墙匹配约0.8415／0.8313／0.8333。

种子发布ROS时间为`1793673424.6642156`。**两条种子前AMCL源被正确拒绝**，保留在`quality_rejections`；之后不同的新源进入联合判据，不能写成全程零拒绝。最新一组五源为`1793673489.386605`、`1793673489.653404`、`1793673489.914163`、`1793673490.318542`、`1793673490.580649`，均晚于本次seed，逐个通过：

- 当时当前的三帧扫描各至少60个有效端点、墙匹配比例至少0.65；末组实际至少82个端点，最低比例约0.8353。
- AMCL位置标准差不超过0.10m，航向标准差不超过5°；TF与该AMCL pose平移差不超过0.03m、航向差不超过0.05rad。
- 同一owner静止锚点和原始反馈来源仍有效；重复旧源不续期，后续质量反证会清除最新通过状态。

末组五源的最大TF平移差约1.802mm、航向差约0.00013185rad。最新联合证据的原始`validated_at_monotonic_s=4212.662531922`，对应北京时间约10:38:10.790；没有在120秒结束后给旧位姿刷新时间。

末次历史AMCL估计约为地图坐标**`(0.4813,-0.3518)m，yaw=-82.03°`**；其协方差标准差约x=0.02146m、y=0.0002843m、yaw=0.08997°。这些是模型输出和内部不确定度，**不是尺量误差、标定精度或地面真值**。报告明确保留`historical_static_localization_verified=true`、`actual_map_localization_valid=false`、`hardware_execution_authorized=false`。

## 收尾、证据和下一步

底盘初末新STOP链通过后，控制口正常关闭；adapter自然退出0，AMCL和map_server由本进程发送TERM后退出-15，无KILL、无清理错误；工作线程退出，本次私有参数已删除。同boot的`/proc`清点只剩基础ROS／雷达／语音进程，USB2控制口无owner。ROS master仍保留本次TERM停止的map_server／AMCL及其TF等幽灵注册，**不能称ROS图完全恢复**；这些注册不代表对应进程仍存活。本次历史位姿不能直接作为未来起步位置。

- [完整定位与底盘原始结果](../../output/p2p_redesign_20261103/actual_loc_r1/localization_result.json)、[实际RK软件结果](../../output/p2p_redesign_20261103/actual_loc_r1/offline_software_validation.json)、[完整RK日志](../../output/p2p_redesign_20261103/actual_loc_r1/offline_tests.log)
- [锚点后三帧扫描](../../output/p2p_redesign_20261103/actual_loc_r1/scan_snapshot.json)、[PC全图候选](../../output/p2p_redesign_20261103/actual_loc_r1_global_proposal.json)、[单次seed回执](../../output/p2p_redesign_20261103/actual_loc_r1/seed_receipt.json)、[五源联合证据](../../output/p2p_redesign_20261103/actual_loc_r1/joint_evidence.json)
- [定位适配器原始报告](../../output/p2p_redesign_20261103/actual_loc_r1/localization_adapter_result.json)、[37源文件复核](../../output/p2p_redesign_20261103/actual_loc_r1/post_source_check.json)、[结束后进程／串口清点](../../output/p2p_redesign_20261103/actual_inventory_after_localization.json)
- [定位运行入口](rk_localization_runtime.py)、[静止锚点](localization_anchor.py)、[私有ROS／TF适配器](localization_ros_adapter.py)

下一步实现RK本地任务服务、GlobalPlanner／DWA及move_base适配和运动owner。到点误差、是否倒车、终点朝向、扫描／定位／规划命令等时效及侧后保护仍按用户待定保留，不以此次工程诊断值替填。运动前需新的有效定位、当次现场条件和完整执行链验证。

---

以下为此前r2静止接入及r1失败／独立恢复的历史原文，保留原结果；其中“当前”只指该历史阶段，不替代上面的本轮定位记录。

# 新点到点导航：实车静止接入记录

2026-11-03，北京时间。**静止底盘／扫描／里程计接入通过；地图定位、运动执行和到点导航尚未启用。**

## 当前实车结果

新包位于RK独立目录 `/home/cat/p2p_navigation_20261103_r2_095820`。没有改旧导航入口、旧地图、开机服务或MCU固件。测试完成后所有新进程已正常退出，控制串口已释放，目录保留用于复核；不是常驻导航服务。

当次boot：`d6a4e5d4-a2fd-4d3f-b768-9cce9bc20e49`。精确匹配 `1a86:7523/bcdDevice0264` 得到当次 `/dev/ttyUSB2`；程序按USB身份查找，不将该tty号写成固定配置。

| 检查 | r2实际结果 |
|---|---|
| RK Python 3.8.10纯软件测试 | 115项通过，0失败、0跳过 |
| 有效静止采集窗口 | 20.0004秒 |
| 全过程物理请求 | STATUS 201、STOP 2、FEEDBACK 804；只有2/3/5 |
| 活动窗口原始反馈 | 759个响应，191个不同MCU样本 |
| 活动窗口保守反馈来源龄 | 最大145.363ms |
| 活动窗口查询起点间隔 | FEEDBACK最大45.059ms，STATUS最大112.795ms |
| 新180°扫描 | 450帧；后方180°为NaN未观测，原始扫描保留 |
| 私有真实轮速里程计 | 发布并实际收到205帧，源时间及坐标帧匹配 |
| 起始／最终停车 | 各自新STOP ACK→后置DISARMED STATUS→5个不同新鲜零速样本通过 |
| 最终底盘状态 | state1、error0、session0、target0、stop_verified1、UART critical0 |
| 最终5个零速样本 | 序号18752～18756，左右实测均0.0RPM |
| 进程与源码收尾 | 新进程无残留、控制口无占用，24个包内源文件前后SHA一致 |

物理写入边界拒绝全部ARM和SETPOINT，包括零SETPOINT。完整写入20B、CRC、序号、零载荷及唯一PID／线程检查均在最内层。没有导航目标、运动许可或自动启动路径。

## 首轮失败与修复

r1包SHA256为 `576d9206769add04be05e62a5a56e617a5729ec250d040c76817c638329ddf02`。首次20秒活动数据健康，但结束STOP在300ms内未得到匹配ACK，因此**r1接入失败**，结果未改写。

原始记录有980个已捕获响应，完整写入982帧，差额恰为末尾一个在途FEEDBACK和一个无ACK的STOP。原Client取消只清除本机待处理事务，无法撤回已写到物理线上的查询；半双工竞争可在内存串口模型复现。实车没有字节级总线采集，不能把碰撞写成已证实的唯一根因。

r1之后另做一次独立查询／STOP恢复：新STOP、后置失能状态及序号15781～15785五个零速样本均通过，控制口正常关闭。该恢复证明不把r1改为通过。

r2只改**固定时长正常结束**：停止发起新查询，按原Client的300ms期限收完已在途查询，再发送STOP。故障和人工取消仍立即进入STOP，不走这段等待。r2实际在途查询为kind5，正常收尾仅5.780ms，没有新查询，也没有改写反馈来源时间或扩大20秒有效窗口。

r2包SHA256：`9a8de0bd5a49259387e020c187266766f62ef3135b47f8491db99ed4fe7c4474`。改动有定向回归和独立审查；PC及RK分别运行同一115项测试后，才执行本次新静止验证。

## 750ms和导航边界

750ms已接入新主机反馈来源龄检查，重复样本不续期，达到门限即失败；明确底盘故障、取消和超出静止速度包络不等待750ms。本轮有效反馈一直健康，**没有进行实车750ms断线故障注入**。

现有Client仍独立执行CAN来源≤250ms、串口事务300ms等检查；MCU目标租约500ms仍是源码基线，未做固件修改或实物看门狗测时。这些机制并未被一个750ms参数统一改写。

本次私有里程计是未标定几何下的实测RPM积分，非目标速度积分；它使用独立的 `p2p_static_odom → p2p_static_base_link` 帧名，没有发布全局TF。里程计在底盘采集结束后停止发布，扫描观察器晚几秒正常退出，报告末尾里程计年龄自然变大，不将其视为活动窗口数据超龄。

当前雷达驱动原始 `frame_id` 仍为 `map`；静止观察保留该原始标签以取证。**这个标签不代表雷达点已转换到地图坐标**，当前扫描不直接交给规划器。导航前需要明确安装外参、正确的laser/base TF及地图定位。本次map、AMCL和TF均缺失并明确标记，`actual_map_localization_valid=false`。

用户已选速度上限0.03m/s、6.9°/s、前180°扫描、20cm前缘停车净距、750ms反馈来源龄、200秒任务时限和RK本地继续任务；本轮没有通过运动验证这些限制。到点误差、倒车许可、终点朝向仍待定，其他工程时效和侧后保护参数也未擅填。

## 证据和后续入口

- [r2完整实车结果](../../output/p2p_redesign_20261103/actual_r2_static_result.json)
- [r2实际RK软件测试](../../output/p2p_redesign_20261103/actual_r2_offline_software_validation.json)、[完整测试日志](../../output/p2p_redesign_20261103/actual_r2_offline_tests.log)
- [r2结束后进程与串口清点](../../output/p2p_redesign_20261103/actual_inventory_after_r2.json)、[源文件读回复核](../../output/p2p_redesign_20261103/actual_r2_post_source_check.json)
- [r1原失败](../../output/p2p_redesign_20261103/actual_r1_static_result.json)、[独立停车恢复](../../output/p2p_redesign_20261103/actual_r1_stop_recovery_result.json)
- [静止接入入口](rk_static_runtime.py)、[底盘只查询适配器](base_static_adapter.py)、[ROS扫描观察器](ros_sensor_adapter.py)

后续先接正确TF／保存地图定位，再接任务服务和运动执行器。每次实车复验使用新的目录和当次boot／设备清点，不覆盖本次结果。当前没有行驶命令可调用，也不需要打开旧网页。

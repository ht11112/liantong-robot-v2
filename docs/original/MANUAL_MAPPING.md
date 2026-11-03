# 手推创建新地图

用户选择回到原先标记，并已确认“已到标记并停稳”。本轮从这处位置和车头方向建立新的软件odom原点，不加载旧地图，不启动AMCL或导航。原地图与旧试验保留为历史数据。

新串口接口只允许STATUS／FEEDBACK（kind2／5）；STOP、ARM、全部SETPOINT和驱动配置命令均被最内层拒绝。结束记录不会锁轮或改变驱动状态。odom使用真实反馈的MCU时间积分，保留0.1RPM；重复来源不更新时间，缺样不补造运动。轮径0.20米、轮距0.30米和雷达安装仍未完成实测标定。

手推过程中偶发轮速缺样会保留为明确的odom缺段：该段不积分位移，原始样本、间隔和缺段计数都留存，激光建图继续。`recording_ended_cleanly`只表示采集正常结束；有缺段时`odometry_continuity_validated`、`mapping_quality_complete`和`recording_complete`均不能为真。来源真正超750ms、CAN过期、雷达断流或明确故障仍结束本轮记录并保存已有新图，不控制底盘。

`origin_record.json`记录第一个有效样本的原始编码器、源时间、boot及软件零点。odom零点取轮轴中心的估计位置，车头朝向为X正方向；软件置零不清MCU编码器。PGM地图的栅格原点另有其自己的YAML定义。

新录图使用独立namespace和新目录。输出包括`mapping_status.json`、`origin_record.json`、`new_map/`和最终`mapping_result.json`。180°输入仅保留实际观测的前方光束，后方不填成空地。原点建立前的扫描不参与新图。

地图保存使用单个后台writer，最多一份正在保存、一份最新待存快照，主循环不等待大文件写入。结束时先确认查询口已关闭、writer已退出，再保存最终图；没有这些条件就不报完整成功。

准备工具位于`output/p2p_redesign_20261103/`：先构建新的最小软件包，核对RK软件测试与当前boot，再启动记录。20分钟为本轮记录上限。向当前会话目录写入`stop_recording.request`表示结束记录并保存新图；这不是底盘停车命令。PC断开不会自动结束已启动的RK记录进程。

当前是否正在录图，以本轮RK状态和首个真实地图消息为准；软件文件存在或测试通过不能代替开始记录的证明。

## 独立只读网页

最新r5已在用户通知完成后正常结束并双份保存，0缺段；见[本轮保存结果](MANUAL_MAPPING_RESULT.md)。下方网页重连检查点是历史记录，当前页面显示“记录已结束”。

本机入口为 [http://127.0.0.1:8766/](http://127.0.0.1:8766/)。`manual_mapping_viewer.py`在PC通过SFTP读取选定会话已保存的地图、`mapping_status.json`和`origin_record.json`；`manual_mapping_viewer.html`只调用GET接口显示数据。页面不访问串口，没有速度、ARM、STOP、目标或结束采集按钮。暂停刷新、关闭标签或网页断开均不改变RK采集。

页面提供地图、独立odom车头与轨迹、原点及前180°扫描叠加、缩放/适配和缺段计数。没有明确map→odom变换时，odom参考与SLAM地图分开显示；扫描按最新odom和估计外参叠加，非逐束校正。PC拉取成功时间与传感器原始来源年龄分开，过期、失败或断连会保留最后快照并标明历史，不能把页面仍有地图当作采集仍在运行。轨迹缺段不连线，不冒充完整轨迹。

2026年11月3日最终重连检查点：旧IAB标签停在后端重启期间的`data:…ERR_CONNECTION_REFUSED`错误页，root在同IAB新tab3已正常打开上述入口，PC只读后端最终为PTY`13078`，早期`98743`已关闭。15:35:06页面显示r4 nonce`27088651`，目录`/home/cat/p2p_navigation_20261103_manual_mapping_152058_27088651`，累计734秒、地图448×480/known15412、gap1、odom=(0,0,0)，首原点sample63708/source mono约6363.156。1次缺段已标明连续性未通过；早期542秒/gap0检查点已被此更新取代。查看[当次浏览器截图](../../output/p2p_redesign_20261103/manual_mapping_viewer_reconnected.png)及[保存的独立状态快照](../../output/p2p_redesign_20261103/manual_mapping_r4_web_reconnect_state.json)。这些都按各自来源时间解释，不能当作永久当前状态或建图验收通过。

只读后端已修复HTTP抓取回执误续源龄的问题：用读取RK uptime之前的PC monotonic参考推进status/odom年龄，包含地图下载、转换与缓存耗时，不能靠网页刷新重盖来源时间。后端`d22983…`／测试`f31cdf…`的6项离线检查通过，独审通过。地图metadata读取出错会保留旧图并提示；legacy回退只用于明确结束、`recording_active=False`的未核验历史记录，不冒当前录图。

r4包SHA为`312707b5d01dae045438899f7c8213165add1f44d86daf6497486c9bc7c1e0f9`，runtime`eeeb2c0c…`、query`bd11f67a…`，PC/RK各27项软件检查通过、0skip。此前r3因待发odom源超750ms失败，虽保存部分地图，仍`recording_complete=false`；原始证据保留于[manual_mapping_r3](../../output/p2p_redesign_20261103/manual_mapping_r3)，不由r4当时active状态覆盖。是否可以继续手推，应以当前会话的新鲜状态与现场通知为准。

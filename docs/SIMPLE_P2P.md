# 简版点到点控制

本分支提供RK本地单目标程序，以及PC本地网页操作入口。控制算法不使用地图、AMCL、DWA或Pure Pursuit，也不加载手推地图；网页可只读展示保存地图，并用独立局部坐标图设置目标：

| 状态 | 动作 | 结束条件 |
| --- | --- | --- |
| 对准 | 线速度0，原地转向目标；接近角度时减速 | odom航向误差≤3° |
| 直行 | 两个物理车轮等速正向，角速度命令0；接近目标减速 | 距离≤10cm |
| 重对准 | 航向误差>8°时先零给定、实测停稳，再转向 | 重新对准后直行 |
| 结束 | STOP、健康失能状态、五个不同且来源在停车状态之后的新零速反馈 | 停车确认完成才成功 |

障碍、反馈/扫描超时、里程计缺段、取消或单任务超时均终止并停车；不绕行、不倒车、不调整终点朝向，失败不自动重新ARM。页面关闭和PC断网不改变RK本地任务；没有网页在线心跳要求。

## 坐标与置零

用户把车推回地面标记并停稳后，`--reset-odom`以首个健康静止反馈把**本次进程**的位置和车头设成`(0,0,0)`。这是软件参考系，不重置STM32编码器。`x`沿起始车头向前，`y`向左，角度逆时针为正。目标单位为米。

直接CLI的每次独立启动均创建新零点，必须使用新的输出目录；不跨未记录的手推、重启或程序退出恢复旧位姿。`origin.json`是本次零点证据，不是常驻odom服务。只建立零点的程序结束后，若再次移动，下一次执行应在选定起点重新停稳置零。不能拿上轮目标或手推地图的map坐标直接当成本轮odom目标。

网页的“设置原点”同样只建立软件局部参考，不清零MCU编码器。预览不启动运动；执行时会核对原点参考所对应的编码器。空闲时手推、重启或发现编码器改变后必须停稳重设原点，不能继续沿用页面中的旧坐标。网页状态保存用于显示与核对，不等于持续运行的里程计。

例如，起始车头为+X、目标`(1,1)`，先左转约45°，再直行约1.41米，实际终止由测得的odom和10厘米容差决定。轮径、轮距、雷达外参和车体边界尚未完成现场标定，所以该例不是地面精度保证。

## 参数

配置为[`config/simple-p2p.json`](../config/simple-p2p.json)。按用户当前要求，额外停障净距`stop_clearance_m=0`，取消原先在车体边界外增加的20cm距离；车体占用和原地转向扫掠范围检查保留。其余参数沿用：前进上限0.03m/s、转向上限6.9°/s、到点误差0.10m、单次200s、RK原轮速来源750ms失效。MCU原CAN250ms、500ms目标租约、±5整数RPM和15RPM异常门限保持。

3°对准、8°重对准、0.30m/20°开始减速是本版可配置工程值。名义轮径0.20m、轮距0.30m；车体前后各0.20m、半宽0.25m和雷达`(0.13,0,0)`是沿用估计，实际需核尺寸。障碍检查仍使用这些车体边界和转向扫掠范围，不从雷达原点算起；额外净距为0不改变上述车体尺寸。

硬件给定只能整数RPM。程序向下量化并重新核算速度上限：名义几何下直行最高2RPM（约0.02094m/s），近目标1RPM；原地转向1RPM（约4°/s），不会把2RPM约8°/s当作≤6.9°/s。命令零不当作实测停稳。

只有前180°雷达，无法证明整个原地转向扫掠区域已清空。需要转向时，操作者须核对车体四周和后方；`--turn-area-clear`记录这次现场检查，雷达仍检查它实际覆盖的前方障碍。缺该标记时需要转向的任务拒绝运动，不用前方净空冒充全周净空。

## 离线检查与打包

```text
python -B -m unittest discover -s app -p "test_*.py"
python -B scripts/verify_source.py --verify-manual-import --verify-r5-baseline
python -B scripts/build_simple_p2p_package.py --name simple-p2p-build-1
python -B -m unittest discover -s dist/simple-p2p-build-1/package -p "test_simple*.py"
python -B -m unittest discover -s dist/simple-p2p-build-1/package -p "test_local_p2p_remote.py"
```

独立包只含简版源码、RK `local_p2p_remote.py`、必要通信依赖、对应测试和配置，不包含地图、凭据、固件、PC网页服务或旧导航。manifest记录两个运行入口及文件SHA，构建结果仍明确 `deployed=false` / `launched=false`。输出目录已存在时拒绝覆盖。打包本身不部署或启动。

## 本地网页

按[USAGE中的本地网页命令](USAGE.md#本地局部坐标网页)启动PC `scripts/run_local_p2p.py`，浏览器访问 `http://127.0.0.1:8766/`。设原点和预览均不发车；每次执行必须显式选择目标并确认当次现场。地图仅只读参考，局部坐标、原点和目标单独显示，不假定存在地图定位。

PC通过RK的 `probe/start/poll/cancel` 任务入口工作，不直接访问底盘串口。RK进程独占串口并在本机完成任务；页面关闭或PC断开后，原200秒及传感器/停车边界继续有效。任务输出保留在本次新目录，取消只作用于本次任务。原手推只读网页源文件和它的9源包不变。

## RK使用

使用RK现有系统Python3.8、pyserial；运动时另需现有ROS Noetic的rospy和sensor_msgs。不要用PC requirements覆盖RK的系统ROS依赖。新包放入独立目录，以下示例均在其`package`目录执行；`BOOT_UUID`须换成当前`/proc/sys/kernel/random/boot_id`，输出目录须是新目录。

只建立零点（不订阅ROS、不STOP、不ARM、不下轮速）：

```text
python3 -s -B simple_p2p_runtime.py --expected-boot-id BOOT_UUID --output-dir /home/cat/simple-origin-001 --reset-odom
```

显式执行单次相对目标的命令如下。只有在当次现场允许运动、车上无人且可人工停车，已检查整个转向区域时才运行：

```text
python3 -s -B simple_p2p_runtime.py --expected-boot-id BOOT_UUID --output-dir /home/cat/simple-goal-001 --reset-odom --goal-x 1 --goal-y 1 --turn-area-clear --execute
```

RKROS环境应已设置`PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages`、`ROS_MASTER_URI`等当前系统参数；代码不修改系统服务。扫描默认`/lidar1/laser_fan`，原header时间计龄，重复帧/反馈不续鲜。MCU定位用本机USB身份识别，不固定ttyUSB编号；同一个进程独占串口。

取消可给该进程SIGINT/SIGTERM，或在当次输出目录创建`cancel.request`。取消只终止任务并确认停车；没有任务时不要给其他进程发信号。程序结束后保存`result.json`和`origin.json`，原始收发在有界内存保存并在串口关闭后落盘；失败首因和最终停车结果分别记录。不要把普通进程退出或STOP回执单独当作停车确认。

当前分支的自动化测试只验证软件行为。实车原地转向、到点误差与停障仍需当次任务验收；这次提交不会自动运行示例目标，也不更改旧手推网页或开机服务。

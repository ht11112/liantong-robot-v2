# V12 保存地图导航

PC 网页使用 `http://127.0.0.1:8766/`。RK 独占 V11 串口，PC 通过已核对主机密钥的 SSH 隧道访问 RK 的本机 API。每次规划都在保存的 PGM/YAML 地图上进行，网页同时显示路线、车头和里程计估计轨迹。

1. 小车停稳后，在地图上标出当前位置及车头方向，点击「单次 AMCL 定位」。RK 使用本轮激光对应时刻的 MCU odom 对齐地图，随后结束临时 AMCL 会话。
2. AMCL 识别失败时，在同一张地图上标出当前位置和方向，点击「置 0」。界面显示「人工对齐」和新的局部零点。地图位置由这个标记和后续 MCU 累计里程计推算。
3. 在地图上选目标并预览。RK 在静态 costmap 上规划，按 TURN 和正向 MOVE 检查整段扫掠。成功定位后可连续预览多个目标。
4. 默认 `ready` 模式将新队列上传至 READY。`drive` 模式在核对新任务 READY 后发送 START。任务开始后 STM32 发放 30 秒租约，RK 收到新鲜反馈并看到剩余约 5 秒时自动续约。任务状态显示剩余授权时间。失联后 STM32 在最近一次租约到期时执行停车。
5. 停车操作分别显示 STOP 应答和新轮速给出的停稳证明。路线预计耗时供参考，持续续约的任务可以超过原先的 20 分钟，总队列最多 128 段。

## 构建与运行

PC 从仓库根目录构建，PGM 和 YAML 使用同一份已保存地图，哈希分别来自地图原件。生成的归档包括 RK 所需源码、地图与文件清单。

```text
python -B scripts/build_v11_map_package.py --name v12-map-r5 --map-pgm FINAL_PGM --map-yaml FINAL_YAML --map-sha256 PGM_SHA256 --yaml-sha256 YAML_SHA256
```

将 `dist/v12-map-r5/v12-map-rk.tar.gz` 解包到一个新的 RK 目录并核对清单，与 V12 STM32 固件配套使用。启动前核对当前串口身份与归属、底盘新鲜反馈和实际停车状态。使用 RK 已有的 ROS Noetic 环境和 Python 3.8。

```sh
source /opt/ros/noetic/setup.bash
python3 -B app/v11_map_runtime.py --port CURRENT_RK_PORT --map-pgm map/final.pgm --map-yaml map/final.yaml --map-sha256 PGM_SHA256 --yaml-sha256 YAML_SHA256
```

默认雷达话题为 `/lidar1/laser_fan`，可用 `--scan-topic` 指定。地图定位使用前 180° 及 1 m 有效下限。仅使用手动对齐时可加 `--manual-only`。现场进入行驶阶段时以 `--mode drive` 启动运行端。

精度调试时可加 `--no-modeled-error-margin`。预览和上传前校验此时按标称路径及车体包络检查障碍和地图边界。实际偏差可能侵入标称路径之外的空间。

PC 使用仓库现有的 `requirements-pc.txt`，启动网页入口后在本机浏览器打开 8766。已知主机键按别名保存时，另传 `--host-key-alias`。

```text
python -B scripts/run_v11_map_web.py --host RK_HOST --user cat --known-hosts KNOWN_HOSTS --map-pgm FINAL_PGM --map-sha256 PGM_SHA256 --map-yaml FINAL_YAML --yaml-sha256 YAML_SHA256
```

提供同一份已校验的地图 YAML 后，网页会显示与 RK 地图对应的基础代价地图，标出障碍、未知区和车体净空。规划器在部分路线中会尝试额外净空，预览仍以保存地图上的路线为准。

置零和 AMCL 对齐均绑定当前 MCU 会话和地图。MCU 会话变化或有效里程计中断后重新定位。首轮实车验收依次核对新队列 READY、约 0.5 m 直行、实测停稳与尺量 10 cm 到点标准，再做转向接直行和取消停车。

## 避障测试

沿用 V12 固件。在 RK 的现有启动参数上加 `--obstacle-test`，PC 网页用同一分支的版本。手动对齐模式也需要先加载 ROS 环境，雷达沿用前方 180° 和 1 m 有效下限。

- RK 将与实测里程计对齐的雷达点临时叠加到保存地图。网页在保存地图和基础代价地图上显示橙红色雷达点。
- 行驶中发现障碍挡住当前队列的剩余路线时，RK 发送 STOP。确认停稳后，从当前地图位姿用 A* 规划到原目标，上传新 MOVE/TURN 队列并继续行驶。网页显示新路线和重规划次数。
- 无可行路线时保持停车。触发停车的扫描在本次重规划期间保留，以免障碍移出雷达扇区后被误判为消失。如果移开障碍后仍在等待，点「停车」取消本次自动续行，再重新预览。实测停稳确认失败时也取消自动续行。

首次实车试验在侧方和后方均有空位的场地进行。先选一条直线路线，行驶后将纸箱放在雷达 1 m 有效下限以外的前方路径上，观察停车、绕行和继续到原目标。雷达扇区和量程以外的障碍不参与本测试。

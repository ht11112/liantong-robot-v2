# 第六分支 MCU：里程计与停稳状态恢复

本轮基于 `feature/mcu-safety-recovery` 的 `0e5223a8cfc6143cf9a03220b90f1726951615a0`。配套 RK 为第七分支 `feature/rk-obstacle-fixes` / `3c3b3fab10102138c3f7179cf7f8da60e26b5aad`。只改第六分支 MCU 源码、离线测试和说明，不修改第四分支、RK 程序或 V12 帧格式，不连接设备或刷写。

## 定位结果与改动

用户现场信息是 `source_fresh=false`，RK 侧里程计超过 250 ms。这是综合判定结果，不能单凭它认定底盘在动，也不能单凭它区分 MCU 无效样本和 RK 收包延迟。

1. **确定的板级时序错误**：主循环先读 `now`，再从 CAN 收包；收包记录当时的 `HAL_GetTick()`。若途中跨过一个毫秒，收包时间比缓存的 `now` 晚，`now - sample_tick` 的无符号运算下溢，新样本反而被判过期。现改为 CAN 接收完成后重新取时间，再处理驱动与队列。测试编译执行真实主循环片段，覆盖跨 1/2 ms、32 位计时回绕，以及真实过期健康缓存仍无效。
2. **启动原点过早**：以前停车自检尚未完成时，第一帧有效速度就可初始化里程计，之后启动过程中的缺段会污染这个过早建立的会话。现等原停车验证完成、看门狗有效、无驱动错误且当前实际轮速为零后，才建立初始原点。此前的速度样本不补积分。
3. **静止恢复窗口被误取消**：以前第二个无效样本会因为 `odom.valid` 已被清除而永久取消恢复。现保留同一个固定窗口，新的无效样本重新计数但不推迟窗口起点；静止时正向样本跳号也可进入恢复窗口。仍要求三个连续有效零速样本、总跨度不超过原 250 ms，缺口期间不积分。倒序、运动、停车验证失效或超窗不自动恢复旧位姿。

第四分支 MCU 的初始化后无效样本没有上述恢复路径，因此仅更换 RK 不会改变第四分支 MCU 的行为。第四与第七仍使用相同 V12 格式，问题不能简单归结为协议版本不兼容。

未改变 MOVE/TURN、租约、CANopen 对象、50 ms SDO 超时、250 ms 新鲜度阈值、真实轮速/电流/状态/故障检查和原停车流程。SDO 超时后的迟到 ACK 不恢复事务可信度；Bus-Off、真实驱动故障仍按原流程处理。

## 离线验证

在完整仓库根目录运行，输出名称须尚未存在：

```text
python -B mcu/build.py --host-tests --name mcu-odom-recovery-check
python -B scripts/verify_source.py --verify-manual-import --verify-r5-baseline
git diff --check
```

新增回归先在旧实现上复现失败，再验证修复后通过：真实板级循环的 CAN 跨毫秒接收、静止重复无效样本、固定窗口不可滑动、启动停车验证、正常积分和运动中失效不能自动续行。集成测试使用内存里的生产 Drive + MQ；模拟 20 ms SDO 响应可以恢复，另保留 21 ms 模拟导致恢复总跨度 252 ms 时必须拒绝的边界。这些时间是注入的测试条件，不是实测驱动性能。

本轮结果：9 套 C 测试全部通过；Python 76 项中 75 项通过、1 项既有 V10 C 黄金向量比对跳过（本次构建提供 V12 向量，其比对通过）；第七分支未改动 RK 的 248 项应用回归全部通过；12 个手推来源文件和 9 个 r5 来源文件校验通过，`git diff --check` 通过。第七分支 RK 回归在其独立工作区运行，与生产 Drive + MQ 的 C 集成回归分别验证，不是设备联调。

目标固件需在具备 ARM GCC 与 STM32CubeG4 SDK 的环境另行构建并检查 32 KB 限制：

```text
python -B mcu/build.py --firmware --name mcu-odom-recovery-production --sdk PATH_TO_STM32Cube_FW_G4_V1.6.3 --compiler-dir PATH_TO_ARM_GCC_BIN
```

主机 C 测试不等于已生成可刷写固件或完成实机验收。

本环境的目标构建尝试因配置的 ARM GCC 不存在而失败，未生成新版 BIN/ELF，未验证新版镜像大小；部署前必须完成上面的目标构建与 32 KB 检查。

## 新版 MCU 配合第七分支 RK 的验证

以下是用户后续验收步骤，本轮未执行。先确认实际加载的是本次第六分支 MCU，RK 仍使用上述第七分支；仅更新 GitHub 文件不会改变设备内程序。按项目已有流程完成目标构建和版本核对。

1. **静止读数**：小车静止，先停止占用串口的 RK 导航进程，再单独运行下列只查询命令。`PORT` 替换为实际端口，不要让两个程序同时占串口。

   ```sh
   python3 -B mcu/tools/mcu_v11.py benchmark --execute --port PORT --duration 15 > odom-check.json
   ```

   检查 `distinct_live_samples` 持续增长、`fresh_source_replies` 接近 `acknowledged_queries`，而非只看查询成功；反复看到同一采样序号不算新数据。第一/最后回包的 `sample_sequence` 与 `measurement_tick_ms` 应前进，boot/epoch 保持一致。有效回包应有 `flags & 3 == 3`，`first_drive_error == 0`，左右 `*_tenths_rpm` 的绝对值 ≤ 2，即 ≤ 0.2 RPM。样龄 `(publication_tick_ms - measurement_tick_ms) & 0xffffffff` 应小于 250。`max_source_age_ms` 只统计有效回包，不能单独拿它证明没有无效帧。

2. **停稳证明**：小车已静止、没有待执行任务时，仍由单独的命令行占用串口，执行现有 STOP 验证，不上传或启动队列：

   ```sh
   python3 -B mcu/tools/mcu_v11.py stop --execute --port PORT > stop-check.json
   ```

   结果需 `success=true` 且 `stop_verified=true`，并核对 `proof.status` 中新鲜有效的里程计和双轮零速。仅收到 STOP ACK 不算停稳证明。启动完成时不一定设置遥测 `STOP_CONFIRMED` 标志；这里以显式 STOP 后的证明为准。

3. **网页复核**：命令行退出后再启动第七分支 RK。静止观察 `source_fresh=true` 能保持，再操作“置 0”和路径预览，应不再被原提示拦住。先不点击执行路线。曾经里程计失效后，原定位会被 RK 作废，恢复有效样本后需重新置 0 或定位。

4. **若仍报超过 250 ms**：保存 `odom-check.json`、`stop-check.json` 和 RK 同期日志。MCU 原始 `flags` 不含 valid、样本时间不前进，优先排查 MCU/CAN 采样；MCU 样本有效且样龄小，但 RK 仍超时，继续排查收包耗时、重复/丢包、RK 线程调度与避障计算。RK 的年龄计算还包含发送后等待时间，MCU 修复不能消除真正的主机延迟。不要用无限延长门槛或强制标记 valid 掩盖来源停更。

恢复超过 250 ms、运动中丢失位置或 SDO 歧义仍不会自动复活旧任务。本补丁解决已复现的 MCU 缺陷，不宣称已消除现场所有 `source_fresh=false` 原因，也不替代轮径、安装参数和真实停稳验收。

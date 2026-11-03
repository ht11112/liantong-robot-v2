# 独立源码和工具的使用范围

本仓库从原项目单独提取，下载后不依赖原来的大工作区。`app/` 中保留原始程序字节，`scripts/` 是本次新增的打包和可配置启动工具。

## 离线验证与打包

在仓库根目录按 README 配好 PC Python 环境，然后执行：

```text
python -B -m unittest discover -s app -p "test_*.py"
python -B scripts/verify_source.py --verify-initial-import --verify-r5-baseline
python -B scripts/build_package.py --name baseline-manual-mapping --verify-r5-baseline
```

输出保存在 `dist/baseline-manual-mapping/`：

| 文件 | 用途 |
|---|---|
| `package/` | 9个手推建图源文件及新的软件包清单 |
| `manual-mapping.tar.gz` | 可传到 RK 的软件包 |
| `build-record.json` | 新包 SHA256 和离线构建结果 |

此工具不访问网络、串口或 ROS，不上传或启动程序。输出目录已存在时拒绝覆盖；需要再次构建时换一个 `--name`。重新打包生成的新清单时间与历史包不同，所以新压缩包 SHA256 不会冒充历史 archive；`--verify-r5-baseline` 比较的是九源文件身份。

未来修改后正常构建时省略 `--verify-r5-baseline`。固定的首次来源清单和历史 r5 身份仍保留，Git 记录新的代码变更。

独立软件包也能离线运行自身测试：

```text
python -B -m unittest discover -s dist/baseline-manual-mapping/package -p "test_manual_mapping*.py"
```

## 准备一次新的手推会话

RK 依赖原系统 ROS1 Noetic、系统 Python 3.8、gmapping、rospy、tf2_ros、geometry_msgs、nav_msgs、sensor_msgs 和 pyserial。已运行历史环境使用 NumPy 1.17.4、PyYAML 5.3.1；本仓库 PC 的 requirements 文件单独管理，不能直接覆盖 ROS 系统环境。

打包时可以显式提供四项新会话输入，生成与程序分开的 `session-input.json`：

```text
python -B scripts/build_package.py --name 新包名 --session-nonce 八位小写十六进制 --expected-boot-id 当前RK的boot-UUID --remote-directory /home/cat/p2p_navigation_20261103_manual_mapping_新会话标识 --duration-s 1200
```

示例里的中文值需要替换。四项必须一起提供；工具不会自动读取当前 RK、确认原点、部署或执行。网页需要的 record 可使用该 session-input 文件，其 `origin_recorded=false` 表示尚未记录原点，实际状态由当次 RK 输出提供。

在实机启动前重新核对 RK boot、当前进程、控制串口归属、真实扫描和用户选择的原点。先把包传到新的空目录，配置实际 ROS 环境，再以**该次** boot、nonce 和新输出目录调用 `manual_mapping_runtime.py`。本次归档没有新建或替换 RK 服务，也没有执行这一步。

手推程序只查询 STATUS／FEEDBACK（kind 2／5），不发 STOP、ARM 或 SETPOINT。正常结束通过当次输出目录的 `stop_recording.request` 完成，保存地图后释放自身资源。PC 断开不会自动结束 RK 采集。现有几何参数仍有未标定估计，源码存在不等于地图精度已经验证。

## PC 只读网页

原 `app/manual_mapping_viewer.py` 主函数包含旧工作区的默认路径；在本仓库使用新增的可配置入口：

```text
python -B scripts/run_viewer.py --host RK地址 --user RK用户名 --record runtime/session.local.json --known-hosts runtime/known_hosts --port 8766
```

提供当前会话 record 和已核对的 SSH 主机密钥文件。SSH 密码在终端交互输入，仅留在内存；本地 `runtime/` 已被 Git 忽略。工具使用现有只读缓存和 HTTP handler，通过 SFTP 读取 RK 已保存的状态、地图和原点，不发送电机或导航命令。网页只绑定 `127.0.0.1`，默认地址为 `http://127.0.0.1:8766/`。

本次只验证该新增入口的离线行为；没有通过它重新连接 RK 做现场运行验收。此前已实际使用的是同一份原始 viewer/cache 源码。

## 导航源码

`trial_navigation_runtime.py`、`rk_localization_runtime.py`、`rk_path_preview.py` 等作为新设计的开发源码保留。它们仍引用历史地图身份和实验包配置，仓库不携带旧地图、旧现场授权或已运行会话文件。r5 新图尚未接入这套导航链，当前归档不生成或运行旧图导航包。

`app/site_config.json` 和 `app/trial_config.json` 保留对话最新输入，包含待定项和历史现场确认。继续导航开发前按新地图和当前现场更新配置，分别记录离线验证和实机验证。

# 手推建图与只读查看

本仓库独立保存手推功能，不需要原来的大工作区。app中的12个保留文件未改字节，scripts为独立构建/核验/查看工具。

本分支还提供简版局部坐标控制网页。手推和运动使用独立运行入口与会话，二者不能同时占用底盘串口；本页前半部分保留手推用法，网页控制见下文。

## 离线测试和构建

在仓库根目录使用PC Python3.10：

```text
python -B -m unittest discover -s app -p "test_*.py"
python -B scripts/verify_source.py --verify-manual-import --verify-r5-baseline
python -B scripts/build_package.py --name baseline-manual-mapping --verify-r5-baseline
python -B -m unittest discover -s dist/baseline-manual-mapping/package -p "test_manual_mapping*.py"
```

三份测试合计33项，独立手推包包含其中27项，不依赖NumPy或ROS。输出目录已存在时拒绝覆盖，再构建需换名。未来主动修改后，普通构建省略`--verify-r5-baseline`；原历史清单继续保留。

`dist/包名/`包含`package/`、`manual-mapping.tar.gz`、`build-record.json`。package包含7个运行源文件、2个手推测试文件和新manifest。重新打包的时间及archive SHA独立于历史包，历史核验只比较九源身份。构建不上传、不启动、不访问网络/串口/ROS。

## 新的RK手推会话

RK继续使用原系统ROS1 Noetic、Python3.8、gmapping、rospy、tf2_ros、geometry_msgs、nav_msgs、sensor_msgs和pyserial。系统NumPy等原ROS依赖保持原环境，不由本仓库PC requirements覆盖。

新会话输入可显式生成：

```text
python -B scripts/build_package.py --name 新包名 --session-nonce 八位小写十六进制 --expected-boot-id 当前RK的boot-UUID --remote-directory /home/cat/p2p_navigation_20261103_manual_mapping_新会话标识 --duration-s 1200
```

示例中文值须替换。四项输入必须一起提供，仅生成`session-input.json`，不会确认原点、部署或启动。viewer的record可用该文件；实际采集状态和原点以当次RK输出为准。

实机启动前核对当前boot、实际进程、串口归属、真实扫描和用户选定原点。将包上传到新的空目录、配置实际ROS环境后，才能按该次身份和新输出目录运行manual_mapping_runtime.py。本次精简没有执行此步骤。

程序只查询STATUS/FEEDBACK（kind2/5），不发STOP、ARM或轮速给定。向当次输出目录写入`stop_recording.request`表示结束记录并保存，不改变驱动状态；PC断开不会自动结束RK采集。

## PC只读网页

建议创建独立Python虚拟环境，再安装`requirements-pc.txt`中的Paramiko。使用新增可配置入口：

```text
python -B scripts/run_viewer.py --host RK地址 --user RK用户名 --record runtime/session.local.json --known-hosts runtime/known_hosts --port 8766
```

record必须对应选定的手推会话，known_hosts提供已核对主机密钥。密码交互输入，仅在内存；runtime目录被Git忽略。工具复用原viewer/cache/handler，只通过SFTP读文件，HTTP绑定127.0.0.1，默认网页为http://127.0.0.1:8766/。

原app/manual_mapping_viewer.py的main保留历史默认路径，因此独立仓库使用scripts/run_viewer.py。新增入口尚未另做现场连接验收；现有网页数据处理有6项离线回归。

## 本地局部坐标网页

新入口为 `scripts/run_local_p2p.py`，HTTP仅监听本机 `127.0.0.1:8766`。它与原只读查看器使用同一默认端口，选择对应入口运行；端口已占用时不要同时再启动另一个服务。

停障失败后，任务卡会显示本任务首个扫描命中点的距离和车头左/右角度；完整坐标和几何见RK结果的`first_scan_rejection`。点的来源未自动分类，不能凭一帧近点直接屏蔽雷达。若最终STOP无ACK后的一次补充停车证明通过，卡片显示“补充停车已实测确认”，原任务仍失败、最终位置仍未验证，需停稳重新置零后再由操作者选择新目标。

先用独立输出名构建RK包，再将包部署到已核对的新RK目录。打包本身不上传或启动：

```text
python -B scripts/build_simple_p2p_package.py --name local-p2p-web-1
python -B -m unittest discover -s dist/local-p2p-web-1/package -p "test_simple*.py"
python -B -m unittest discover -s dist/local-p2p-web-1/package -p "test_local_p2p_remote.py"
```

RK目录须包含 `local_p2p_remote.py`、简版runtime、manifest与配置。新增remote只提供 `probe/start/poll/cancel` 四种本地任务操作；PC服务和网页资源保留在仓库，不复制进RK运动包。RK仍使用系统Python3.8、ROS Noetic和pyserial，PC依赖不覆盖RK环境。

PC先安装 `requirements-pc.txt`，按实际文件和身份填写参数：

```text
python -B scripts/run_local_p2p.py --host 192.168.1.228 --user cat --known-hosts runtime/known_hosts --host-key-alias fe80::ecdf:6b65:24f6:9595%10 --remote-directory /home/cat/p2p_navigation_20261103_SIMPLE_SESSION --map-pgm PATH_TO_SAVED_FINAL_PGM --map-sha256 EXPECTED_PGM_SHA256 --map-resolution 0.05000000074505806 --state-file runtime/local-p2p.json
```

示例地址、目录、PGM路径及哈希应换成本次实际值。`known-hosts`提供已核对的主机密钥；密码通过交互输入，只留内存，不写入state文件。地图PGM在PC只读显示，并检查所给SHA；不向RK加载地图，不启动AMCL，不推断map→odom关系。

网页操作顺序：

1. 停稳后设置软件局部原点。该操作只查询真实反馈，记录当前编码器参考，不清零MCU编码器，不发ARM或运动给定。
2. 在独立局部坐标视图填写目标，`x`沿设原点时车头向前、`y`向左。预览只显示转向与直行示意，不发车，也不代替执行时的当前扫描检查；保存地图中的像素不是该局部目标坐标。
3. 检查当次路径及车体四周后，显式执行该目标。需要转向时，还需确认包括后方盲区的完整扫掠区域。RK执行前重新核原点对应的编码器和当前传感器，不复用旧检查点的新鲜状态。
4. 执行后可查看进度或取消当前任务。取消请求由RK处理并核实停车；关闭网页、停止PC刷新或断开PC不会替代取消，也不会中断RK本地任务。

空闲时如果手推机器人，保存的局部参考关系会失效；先停稳并重新设置原点再执行。重启、未记录移动或旧state文件也不能恢复一个已验证的当前位姿。原点、预览和成功提示都不表示地图定位或地面精度已验。

按用户当前要求，额外停障净距设为0（`stop_clearance_m=0`），不再在车体边界外增加20cm；车体占用和原地转向扫掠范围仍检查。任务保持200秒上限、原轮速/扫描来源750ms、到点10cm容差、禁止倒车及失败不自动重试。原地转向与直行分离，正常到点先零目标实测停稳并封存位置，再核STOP、五新零和停车编码器保持不变；最终距离仍须在10cm内。取消和故障直接STOP，不等待封存。更多参数与CLI用法见[SIMPLE_P2P.md](SIMPLE_P2P.md)。

# 手推建图与只读查看

本仓库独立保存手推功能，不需要原来的大工作区。app中的12个保留文件未改字节，scripts为独立构建/核验/查看工具。

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

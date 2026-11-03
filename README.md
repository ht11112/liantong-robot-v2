# 联通机器人：手推建图与简版点到点控制

本分支 `feature/simple-point-to-point` 新增独立点到点控制：**原地对准目标 → 两轮等速直行 → STOP并核实零速**。只使用真实轮速odom，不加载地图或AMCL，不自动绕障。直行偏航过大先停稳再对准；靠近目标减速，到点不调整车头。见[简版使用说明](docs/SIMPLE_P2P.md)。main及手推回退标签保持原范围。

新功能默认只建立本次软件odom零点。只有显式 `--execute` 加目标才会ARM；本次开发不自动运行行驶任务。手推建图和只读网页保持原程序字节，网页没有新增车辆控制接口。

原手推基线保存已实现的**手推创建新地图、PC只读查看及其测试**，来源于“重新设计点到点导航”对话最后完成的手推建图 r5。

RK运行程序、主机通信依赖和网页保持原始字节。当前main已移除点到点导航、定位/路径预览/运动联调入口、无关配置和对应测试；不携带STM32固件、现场地图、日志或凭据。

## 当前功能

- RK读取真实轮速和前180°雷达，建立软件odom原点，使用GMAPPING创建新地图并保存记录。
- PC通过SFTP只读显示地图、odom、原点和采集状态，网页断开不改变RK采集。
- 手推r5实际记录约641秒、0次里程计缺段，正常结束并保存。几何精度和轮径/轮距/安装外参仍未完成标定。

## 程序清单

| 文件 | 作用 |
|---|---|
| `app/manual_mapping_runtime.py` | RK手推建图主入口，组织扫描、odom、GMAPPING和地图保存 |
| `app/manual_mapping_query.py` | 只查询底盘状态/轮速，计算真实手推里程计 |
| `app/manual_mapping_viewer.py` | PC只读网页缓存和HTTP后端 |
| `app/manual_mapping_viewer.html` | 配套地图查看页面 |
| `app/base_static_adapter.py` | 手推查询必需的串口围栏、来源追踪和CRC基础类，无独立启动入口 |
| `app/deps/` | 4个必要主机Python依赖：control_client、core、rk_link、transport |
| `app/test_manual_mapping_runtime.py` | 建图运行逻辑测试 |
| `app/test_manual_mapping_query.py` | 手推查询与里程计测试 |
| `app/test_mapping_viewer.py` | 只读网页数据处理测试 |

`app`总共12个文件：8个程序Python文件、3个测试Python文件、1个HTML文件。手推程序只发STATUS/FEEDBACK，不发STOP、ARM或轮速给定。基础依赖文件包含共用辅助代码，保留完整文件便于核验已实测源文件身份。

## 离线检查与构建

在仓库根目录使用PC Python 3.10：

```text
python -B -m unittest discover -s app -p "test_*.py"
python -B scripts/verify_source.py --verify-manual-import --verify-r5-baseline
python -B scripts/build_package.py --name manual-mapping --verify-r5-baseline
python -B -m unittest discover -s dist/manual-mapping/package -p "test_manual_mapping*.py"
```

现有测试只用标准库，不连接设备。构建输出在`dist/`，已存在的构建目录拒绝覆盖；需要再次构建时换一个`--name`。未来修改代码后正常构建可省略历史身份检查参数。

PC查看网页需要Paramiko，可在独立虚拟环境执行`python -m pip install -r requirements-pc.txt`，通过`python -B scripts/run_viewer.py --help`查看参数。RK仍使用ROS1 Noetic、系统Python3.8、gmapping和原有pyserial；PC依赖文件不用于覆盖RK系统环境。

## 来源与回退

当前手推专用版本标签：`v0.1.1-manual-mapping-20261103`。保留的12个文件及原路径/SHA见[来源清单](provenance/source_manifest.json)，实际r5九源身份见[基线](baselines/manual-mapping-r5.json)。

此前完整导入已保存在历史标签`v0.1.0-baseline-20261103`；本次通过普通提交精简main，保留Git历史供回退。新功能通过分支开发，验证后提交和打新标签。

[实际验证范围](docs/BASELINE.md) · [独立使用](docs/USAGE.md) · [本次软件验证](docs/VALIDATION.md) · [开发与回退](docs/VERSIONING.md)

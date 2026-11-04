# 联通机器人：手推建图与简版点到点控制

本分支 `feature/simple-point-to-point` 新增独立点到点控制：**原地对准目标 → 两轮等速直行 → STOP并核实零速**。只使用真实轮速odom，不加载地图或AMCL，不自动绕障。直行偏航过大先停稳再对准；靠近目标减速，到点不调整车头。见[简版使用说明](docs/SIMPLE_P2P.md)。main及手推回退标签保持原范围。

按用户当前要求，正常到点以真实odom进入10厘米容差为准，记录当时位姿后立即STOP，不再等待到点零速封存或复验最终停车位置。STOP停车证明仍单独记录；到点且主停车链成功可完成任务，最终停车位姿标为未验证，下一任务需停稳重新置零。不跨STOP采样空窗补积分。取消或故障仍立即STOP。

新增本地网页入口位于 `http://127.0.0.1:8766/`：查看已保存地图、设置软件局部原点、预览局部坐标目标、显式执行或取消单次RK任务。地图仅作只读参考，局部坐标图独立显示；不把保存地图当作机器人定位。设原点和预览不发车，执行才允许ARM。页面或PC断开后，RK按原任务边界继续运行。

控制以20Hz读取最新的已累计odom；唯一串口owner逐条积分真实反馈，接收等待期间仍运行控制tick。短历史只在连续实测位姿之间插值，不丢弃原始轮速、不跨缺段补位移、不外推。通信查询和轮速给定按独立期限调度，详见[数据与控制时序](docs/SIMPLE_P2P.md#数据与控制时序)。

当前按用户要求将额外停障净距设为0，并把导航使用的雷达距离下限设为1米。原始雷达消息仍保留；原本有效但小于1米的近点不参与导航几何检测，原始坏数据和来源超时仍拒绝。当前车体及转向扫掠范围都位于1米近场盲区内，因此不能依靠此配置检测车体附近碰撞，也不能把远场扫描有效称为近场净空。网页显示该距离下限，此数值不是1米停车距离。失败不自动重试。

停障失败会保存首个拒绝扫描的时间、命中距离/角度、车体坐标和所用几何，网页显示首个命中点；不自动判断为车体回波或噪声。最终STOP未收到匹配回包时，同一串口owner最多再发送一次新序号STOP并独立核验停车。原失败和位置缺证保留，补充停车成功不算任务成功。

原手推建图、配套只读网页及12项来源文件保持原字节。新增入口使用独立的 `local_p2p_server.py` / `run_local_p2p.py`；启动方法见[使用说明](docs/USAGE.md#本地局部坐标网页)。

原手推基线保存已实现的**手推创建新地图、PC只读查看及其测试**，来源于“重新设计点到点导航”对话最后完成的手推建图 r5。

原手推RK程序、主机通信依赖和配套只读网页保持原始字节。当前main已移除旧点到点导航、定位/路径预览/运动联调入口、无关配置和对应测试；本feature独立增加上述局部坐标功能，不携带STM32固件、现场地图、日志或凭据。

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

以上是原手推的12项来源子集：8个程序Python文件、3个测试Python文件、1个HTML文件。手推程序只发STATUS/FEEDBACK，不发STOP、ARM或轮速给定。基础依赖文件包含共用辅助代码，保留完整文件便于核验已实测源文件身份。

简版控制另含 `simple_p2p.py`、`simple_p2p_base.py`、`simple_p2p_history.py`、`simple_p2p_runtime.py`、RK任务入口 `local_p2p_remote.py` 和各自测试。PC网页服务与UI不进入RK运动包；手推包仍独立构建。

## 离线检查与构建

在仓库根目录使用PC Python 3.10：

```text
python -B -m unittest discover -s app -p "test_*.py"
python -B scripts/verify_source.py --verify-manual-import --verify-r5-baseline
python -B scripts/build_package.py --name manual-mapping --verify-r5-baseline
python -B -m unittest discover -s dist/manual-mapping/package -p "test_manual_mapping*.py"
python -B scripts/build_simple_p2p_package.py --name simple-p2p-web
python -B -m unittest discover -s dist/simple-p2p-web/package -p "test_simple*.py"
python -B -m unittest discover -s dist/simple-p2p-web/package -p "test_local_p2p_remote.py"
```

现有测试只用标准库，不连接设备。构建输出在`dist/`，已存在的构建目录拒绝覆盖；需要再次构建时换一个`--name`。未来修改代码后正常构建可省略历史身份检查参数。

PC查看网页需要Paramiko，可在独立虚拟环境执行`python -m pip install -r requirements-pc.txt`，通过`python -B scripts/run_viewer.py --help`查看参数。RK仍使用ROS1 Noetic、系统Python3.8、gmapping和原有pyserial；PC依赖文件不用于覆盖RK系统环境。

## 来源与回退

当前手推专用版本标签：`v0.1.1-manual-mapping-20261103`。保留的12个文件及原路径/SHA见[来源清单](provenance/source_manifest.json)，实际r5九源身份见[基线](baselines/manual-mapping-r5.json)。

此前完整导入已保存在历史标签`v0.1.0-baseline-20261103`；本次通过普通提交精简main，保留Git历史供回退。新功能通过分支开发，验证后提交和打新标签。

[实际验证范围](docs/BASELINE.md) · [独立使用](docs/USAGE.md) · [本次软件验证](docs/VALIDATION.md) · [开发与回退](docs/VERSIONING.md)

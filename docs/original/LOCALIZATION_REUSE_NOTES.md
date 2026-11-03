# 保存地图静止定位：可抽取算法与地图身份

本笔记整理于2026-11-03，仅读本地源码/历史证据；未连接RK、启动ROS/owner或访问设备。新P2P静止owner由root控制，下面只说明独立定位进程可复用的纯算法与数据。历史pose、seed、wheel anchor和“已定位”结果都不是本轮当前位置。

## 已实现的新独立PC算法

[global_pose_proposal.py](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/p2p_navigation_20261103/global_pose_proposal.py) 是新的独立实现，不导入旧462源/owner。纯函数 `global_pose_proposal(map_doc,pgm_bytes,snapshot,expected_map_id,extrinsics=(.13,0,0),...)` 只依赖NumPy及标准库。输入snapshot可用顶层`scans`或`snapshots`，每帧支持精确`header.stamp_secs/stamp_nsecs/source_stamp`、`raw_header`和`ranges`中的null/inf字符串；其精确原时间保留，不把计算时刻当传感器来源。

它在全图free格中心按约0.1m采样、全yaw每5°粗搜，XY按1024分块；全局评分存NumPy矩阵而不是大量Python tuple。依次用位置/航向掩码选出前5个互相分离模式，**每个模式都用相同2.5cm/0.5°细化**，再公平比较；不是只细化第一名后与粗糙第二名比较。PGM与map.json先分别规范化核对同一expected map_id；unknown端点不借邻墙当命中。

默认本地图的expected map_id为`9e34ebc9…07c746`。`extrinsics`显式同时用于粗细搜索和独立三帧端点复核；`endpoint_agreement(info,cells,points,pose)` 可单独复用。输出含`pose/seed_candidate_accepted/mode_score_margin/frame_agreement[3]/map_id/boot_id/nonce/proposal_snapshot_stamps/proposal_snapshot_exact_stamps`，并明确`localization_verified=false/device_permit=false/source_freshness_revalidated=false/stationarity_revalidated=false`。snapshot顶层给boot_id/nonce时原样携带；它不自行创建或推断运行身份。

PC调用示例（从项目根目录执行，把本轮快照和输出路径替换成实际新文件）：

```powershell
python -B navigation-demo/p2p_navigation_20261103/global_pose_proposal.py --map-json navigation-demo/map_localization_20261024/maps/map_bfad7ac3207a/map.json --map-pgm navigation-demo/map_localization_20261024/maps/map_bfad7ac3207a/map.pgm --scan-snapshot <本轮scan_snapshot.json> --output <本轮proposal.json> --numpy-dir tmp/project-review-20261027/test-deps
```

没有全局安装NumPy。CLI的`--numpy-dir`只改当前进程搜索路径；测试先使用可导入NumPy，否则找独立测试目录。RK可用系统NumPy运行同源测试，不依赖旧工程绝对路径。[test_global_pose_proposal.py](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/p2p_navigation_20261103/test_global_pose_proposal.py) 11项实际通过：非原点非对称房间、对称歧义、unknown邻墙、错误map id/PGM、重复/倒序scan、特殊range、精确stamp、外参及快照metadata/三帧frame错配。顶层mounting存在时x/y/yaw必须与显式extrinsics在1e-9内相同；frames存在时laser必须非空且三帧header都相同，旧list输入中有frame时也禁止混杂。源码与测试通过Python3.8语法解析。此次实际解释器为PC Python3.10、独立NumPy2.2.6，不冒称已在RK3.8/NumPy1.17运行。

另外只用历史三帧作为离线数据，在当前608×384本地地图实际跑过全图：9295候选×72yaw、评分矩阵5,353,920B，5模式全部细化，总7.516s。历史帧的端点比例约0.939～0.961，得到proposal而非本轮位置；未写该旧姿态为新seed。这个试验只说明PC算法在实尺寸数据可运行，本轮必须计算本次新快照并在seed前重新核静止与来源。

## 建议的独立入口

最小算法组合是：

1. [room_heading_candidate.py](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/map_localization_20261024/room_heading_candidate.py) 的 `distance_field(occupied,resolution,limit=.4)` 与 `propose(info,cells,scans,center=(0,0),radius=1)`。SHA256 `b473f4f582660e68e6218260712b830a5ea4a2d726b07651378135a3ea0b423a`；[static候选副本](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_20261102/src/room_heading_candidate.py) 与其同字节。纯计算需要 NumPy 和下面的 `scan_match`；不要执行其历史 `main()`，后者会读取旧capture并覆盖旧proposal文件。
2. [maploc_core.py](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/map_localization_20261024/maploc_core.py) 的 `pgm(raw)`、`classify(data)`、`map_identity(info,data)`、`scan_match(...)`、`pose_quality(...)`、`stationary_certificate(...)`、`stability_update(...)` 和 `angle/checked_pose`。SHA256 `646a24ac438a573d4ce27019dfceb8e6b15268573f27ca65ab4ce8225665bcc6`；仅标准库，无ROS/串口导入。
3. [static_localization_core.py](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_20261102/src/static_localization_core.py) 的 `accepted_proposal(value,scan_stamps)` 与 `LocalizationEvidence(seed_ros,seed_mono).observe(...)` 可作为本轮独立观察判定参考。整个模块还依赖 `maploc_core`、`stop_observation`，并含旧journal/frame/250ms结构；无需把这些旧owner数据结构带进新P2P，只抽取明确需要的判定。

建议独立流程：核当次保存地图文件及加载栅格身份 → 新TF和新实际RPM odom建立后记录本轮静止锚点 → 取3帧不同且当前的前180°扫描 → 独立进程计算全图/局部proposal → 计算完成后重新检查本轮静止/反馈/最新扫描 → 向本次私有AMCL initialpose发布一次proposal → 只把seed之后不同AMCL时间戳、实时扫描匹配与当前TF一致结果记为静止定位观察。物理串口始终由现有新P2P唯一owner控制；定位进程没有ARM、SETPOINT或目标入口。

## proposal实际做什么

- 第一帧每4束取样，保留 `max(.3,range_min) ≤ r ≤ min(8,range_max)`；至少60个端点。占据格阈值≥65、可落脚中心格0..25；unknown/越界不当作墙匹配。
- 全yaw粗搜 `[-180°,180°)` 每5°，平移网格10cm；只在粗搜最佳附近±12.5cm/±6°细化，步长2.5cm/0.5°。
- 墙距离场上限0.4m，高斯评分sigma0.10m。竞争模式间距离>0.4m或航向差>20°，保留最多5模式。要求best与第二模式分差≥0.08。
- 对传入的3帧全部做独立端点 `scan_match` 复核（包含用于拟合的第一帧）；每帧 `used≥60`、`near_wall_ratio≥0.65` 才有 `seed_candidate_accepted=true`。输出仍固定 `localization_verified=false`，不是全局定位证明。
- `propose()` 自身只检查至少3帧，不检查三帧是否不同、时间是否新鲜、车是否静止。调用方必须核原始stamp严格递增/3帧不同、当前来源龄、同一静止锚点。`accepted_proposal()`补核恰好3个不同stamp及上述结果门，但仍不能替代来源/静止事实。
- 支持这里origin yaw0的保存地图；`propose()`明确拒绝非零map origin yaw。当前`scan_match`能旋转map origin，但这不代表proposal也支持。

两个函数都硬编码laser相对base为 `x=.13m,y=0,yaw=0`：`propose`构造点与`scan_match`端点计算各有一处。新TF若采用其他安装值，应在新派生模块中把同一外参同时传给这两处；不能只改TF而保留匹配模型。原始LaserScan错误标成`map`不能被当作已有全局坐标。前180°仍保留后方NaN，不在此次定位中擅自恢复后方为可通行。

## 全图搜索与局部搜索

默认 `(center=(0,0),radius=1m)` 只搜索旧起点附近；本轮只有“同房间”，不能当作该位置先验。无新位置提示时可用当前已核地图free格中心的完整bounds构造搜索方形：

```python
center = ((xmin + xmax) / 2, (ymin + ymax) / 2)
radius = max(xmax - xmin, ymax - ymin) / 2 + 0.1
proposal = propose(info, cells, three_current_scans, center=center, radius=radius)
```

这里bounds来自地图中0..25 free格中心坐标，不来自历史车位。`propose`仍按10cm候选网格粗搜方形，并过滤候选中心是否free；“全图”表示覆盖已知free格范围，不表示枚举每个5cm格或证明未知区域。若用户后续给本轮可信近似位置，可用局部center/radius，但搜索basis应标注为新提示，不沿用旧seed。

本地当前保存大图37,184个free格的中心bounds为 x `[-7.824999967589974,11.62500032223761]`、y `[-8.074999971315265,6.925000252202153]`。得到center `[1.9000001773238182,-0.5749998595565557]`、radius `9.825000144913792`。这些是本地文件复算的地图几何；RK文件仍需当次验证。

[历史完整全图proposal](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/local_task_runtime_20261030/fresh_localization_20261031_135742/proposal.json) 有同样搜索参数及“Full current saved-map free-cell bounds and all yaw”记录，可追溯参数选择；它的旧pose不能用于本轮。全图计算比局部重，应独立进程运行，让物理owner继续查询/STOP与750ms来源监控；计算结束不能把旧三帧时间盖新。

## 两幅历史地图必须分清

最新static候选选定身份文件是 [saved_map_identity.json](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_20261102/package/saved_map_identity.json)，已去除旧车位，仅留地图配置身份。其原metadata在 [map_bfad7ac3207a/metadata.json](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/map_localization_20261024/maps/map_bfad7ac3207a/metadata.json)，含旧reference_pose与wheel_pose_at_save，应忽略这两个定位字段。

| 项目 | 后续static所选大图 | manual_relocation_v3所用早期图 |
|---|---|---|
| 名称 | `map_bfad7ac3207a` | `map_61ad34ba8a4f` |
| 原RK目录 | `/home/cat/navigation_flow_20261020_144704/workspace/maps/maploc_20261024_152112_0e258d/map_bfad7ac3207a` | `/home/cat/navigation_flow_20261020_144704/workspace/maps/maploc_20261024_145857_368a50/map_61ad34ba8a4f` |
| 栅格 | 608×384 | 384×384 |
| resolution/origin | 约0.05m，`[-10,-10,0]` | 同左 |
| grid map_id | `9e34ebc9f5a36fbbf64e3a6f7bbef4d23683cd97de1963bba00fe9f60507c746` | `516233f5cf40d42cb00c542f114640c6c8e4400777735f22386ecc3c985de653` |
| map.pgm SHA256 | `b339c617af34659e12112e8a85e135fce753e28a1ef57de4d84fb5bd2b6173c3` | `bf50b934d1eb7d7f9a0299f6befb01ae72ff950aab96640fca3dfc27e586b7e3` |
| map.yaml SHA256 | `3547edd9ea41a63d4853757910323ccc949e5a4644f7b6d9964efdf0cc03e9ca` | **同左** |

本地大图：[map.yaml](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/map_localization_20261024/maps/map_bfad7ac3207a/map.yaml)、[map.pgm](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/map_localization_20261024/maps/map_bfad7ac3207a/map.pgm)、[map.json](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/map_localization_20261024/maps/map_bfad7ac3207a/map.json)。本次已在本地分别对PGM解码数据与map.json计算map_id，两者均等于上表大图身份；map.json文件SHA256为 `4ce2d9a504dcdc7cdf04ecd98abbbf364bd180d139fa370f1cb6651ee98fa06c`。

**只验证YAML不足以区分这两图**：它们YAML字节完全相同（相对image都是`map.pgm`）。至少同时核YAML、PGM字节和当前加载OccupancyGrid规范化身份；路径、尺寸或文件mtime也不能替代内容身份。

`map_identity(info,data)`按width/height、resolution和origin各round到6位、规范化classify(cells)的sorted compact JSON做SHA256。分类0..25→free0、≥65→occupied100，其余→unknown-1。其输入应先严格核尺寸、有限正resolution、合法origin和每格[-1,100]整数，不能把分类函数当非法格验证器。`pgm()`专用8bit P5/255，以254→0、0→100、其他→-1并翻转Y方向；这是此冻结map_saver图的规则，不是任意YAML阈值/negate通用解析。当前YAML为negate0、occupied_thresh0.65、free_thresh0.196。

## manual_relocation_v3的正确用途

[manual_relocation_v3/baseline.json](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/map_localization_20261024/manual_relocation_v3/baseline.json) 明确绑定早期384×384地图与旧session `maploc_20261024_150507_71dc82`。该目录的baseline/result/samples是人工推动时的旧wheel/AMCL对照，不是三帧proposal算法，也不是本轮地图选择或机器人姿态依据。

[capture_manual_relocation.py](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/map_localization_20261024/capture_manual_relocation.py) 只经旧active_session轮询历史服务文件，应当作采集格式参考，不执行。本轮没有要求手推，不需要复用其operator_finished流程。[manual_relocation_acceptance.json](C:/Users/Administrator/Desktop/联通智能机器人/navigation-demo/map_localization_20261024/manual_relocation_acceptance.json) 的旧ready和22秒AMCL龄使用当时stationary重验证语义；不能刷新为本轮正常AMCL结果。

## seed之后的独立定位观察

`LocalizationEvidence`要求seed之后的新AMCL source/receive时间、frame map、36项有限协方差，位置标准差≤10cm/航向≤5°；最新scan端点≥30、墙命中比例≥0.55；非stationary特例AMCL源龄/接收龄≤1.8s；当前TF与AMCL平移差<3cm、角差<0.05rad、TF源龄在[-0.30,0.5]s。只有至少5个不同AMCL stamp的连续满足观察才报告 `external_observed_localization=true`；仍返回 `owner_localization_valid=false/navigation_task_allowed=false/authority_created=false`。

这些是旧纯观察算法的工程判据，不是用户新填的到点误差或运动权限；新P2P应显式记录采纳的观察参数。静止AMCL可用本次私有`request_nomotion_update`触发新更新，不能拿默认启动pose或重复旧stamp累计5次。参考调用链在 [warmup launcher](C:/Users/Administrator/Desktop/联通智能机器人/candidates/v9_static_localization_warmup_20261102/src/static_localization_launcher.py) 的三扫描capture→propose→accepted_proposal→私有seed→LocalizationEvidence；只参考接线，不启动其旧462源树或物理owner。

正确TF应来自本轮真实odom与安装外参，保持单一发布者和本次私有帧/主题；AMCL/map_server只接新包选择的帧与扫描。完整静止结果应将地图身份、proposal、5次AMCL/TF观察、本轮owner静止/源时、初末新STOP链分别记录，失败也保留。无proposal或质量不过就明确未定位，不向导航任务授权。

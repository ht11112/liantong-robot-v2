# 首次归档基线：2026-11-03 独立 RK 上位机

本次归档取自 Codex 对话 **“重新设计点到点导航”** 的最新本地程序，包含独立点到点导航代码、手推建图代码、只读查看器及运行所需的主机侧 Python 依赖。它是可追溯的源码基线；当前整个源码集合没有经过一次统一的实车导航验收。

## 来源与时间边界

| 项目 | 身份 |
| --- | --- |
| 原对话 | `重新设计点到点导航` |
| 对话 ID | `01a19f01-1ef9-76f3-b7d8-5c38faff4b96` |
| 最后完成的用户轮次 | `01a1a072-129d-7792-ad7e-8912685d621f` |
| 最后用户请求 | `新地图采集完成` |
| 该轮完成内容 | 手推新地图 r5 结束、保存、下载备份与原始记录一致性核验 |
| 原本地源码目录 | `navigation-demo/p2p_navigation_20261103/` |

核对来源包括该对话最后一轮、项目 `PROJECT_MEMORY.md` 的最新记录、原目录中的 `README.md`、`MANUAL_MAPPING_RESULT.md`，以及 `output/p2p_redesign_20261103/` 内的冻结包清单和独立审计结果。原始工作区和历史失败证据未被本次整理覆盖。

下文 boot、进程、端口与位置均为对应历史检查点的事实，不能当作机器人当前状态。上传源码不启动建图、定位或运动，也不延续过去的现场运动许可。

## 手推建图：最新实车完成版本

“r5”是一次新的采集会话，复用了 r4 冻结的同一份软件，未改动这九个源文件。r4 包的 PC 软件检查为 27 项通过；r5 在新 RK 会话实际运行同包 27 项检查通过，均无跳过。不能把它们相加当成 54 项不同测试。

| 项目 | 身份或结果 |
| --- | --- |
| 软件包 SHA256 | `312707b5d01dae045438899f7c8213165add1f44d86daf6497486c9bc7c1e0f9` |
| r5 会话 nonce | `76ebb1d2` |
| 当次 boot ID | `2fee0fae-658d-4245-9bd8-fc65e39f297b` |
| 当次 RK 目录 | `/home/cat/p2p_navigation_20261103_manual_mapping_155534_76ebb1d2` |
| 实际采集时长 | `641.012066` 秒 |
| 完整性 | 正常结束，0 次里程计缺段，`recording_complete=true` |
| 数据量 | 6264 个不同真实轮速样本/odom，8584 帧前 180° 扫描，523 张地图消息 |
| 串口请求 | STATUS 5852，FEEDBACK 23123；STOP、ARM、SETPOINT 均为 0 |
| 正常收尾 | 查询口关闭，query/writer 退出，自有 gmapping 正常退出 |

下表文件名采用原冻结包的相对路径。归档中的对应原始文件应保持这些字节和哈希；仓库新增的配置或打包工具不属于这份历史九源实车身份。

| 冻结文件 | SHA256 |
| --- | --- |
| `base_static_adapter.py` | `a529dd532009c70e172ed81fb211b7a4a7e62451c4d20eca7d84a5984107cde7` |
| `deps/control_client.py` | `a293816a54a52eed2a60dbb310b639c43763f14146797dc22e6dd8ab992b5e55` |
| `deps/core.py` | `181f1f6d1e6fec9de6143355ded815d9ce75946dc4b0aad9e77fd7a1f567baba` |
| `deps/rk_link.py` | `d3546a87e31f79d43de63396f4d5c5699b32a532df2182757f5324ee49a99f0b` |
| `deps/transport.py` | `45950cfdd941b6f0d49f27f9a99ad6294ea32443ae3d34e64569f7c1ca7f57ae` |
| `manual_mapping_query.py` | `bd11f67a25bf5e3abd6b12fc1f8b3082bcdef028f686b90a53284f6940b6f692` |
| `manual_mapping_runtime.py` | `eeeb2c0c9ee3e49cf8a660dcc530c420cbd39de2e7b898dc248b2458ca800dde` |
| `test_manual_mapping_query.py` | `8633f482ce3163b3c37bcda01917a3bd12354d39e93254bc1ce2475532ee0390` |
| `test_manual_mapping_runtime.py` | `f566ddc3a3984b96011c0f574e15d2796c1e617d8ac5337db5bd120cbf8d30a4` |

原始采集证据包 SHA256 为 `e722fe456c83f9f70e2ca5d70bc3ed0b496cf2a3cf2332745088ed3712b11ac0`；独立离线审计报告 SHA256 为 `99e4bd3dc499728c5c82917e8de581786fe651915ea809be03dcf82fd6c53e56`。审计核对完整收发、CRC、真实来源、积分、地图重建和正常退出，没有证明地图几何精度或导航通过。

## 导航 trial：最新源码与历史验收的区别

最近一次实际半米行驶试验为 trial r6：46 文件包 SHA256 `aaeda79403ef7417d1bba39593e47438c6d6d9d0faf049c55eecd382eca9a233`，该冻结包在 PC 和 RK 分别通过同一 252 项软件检查、无跳过。实车发出 19 个非零给定，约 1.30 秒后因 AMCL 原始定位来源超过 2 秒而停车，**半米到点验收未通过**。给定被接受不能证明车辆完成对应位移；后续独立恢复停车成功不改变试验失败结论。

当前本地 `base_trial_adapter.py` 已包含 r6 之后的停车证明修复：先排除早于健康停车 STATUS 的 CAN 样本，避免旧运动读数参与反馈续期、里程计或五个新零样本证明；保留原截止时间。其 SHA256 为 `4ab102c479a7dee987db6506c7fb1bb8a2eee335969bf71bc8103940fc2092f7`，配套 `test_base_trial_adapter.py` 为 `2bae1da26b88c76e609b3687c7543ed4c4c0ad2e9e129f30fd31d92afdb78d1f`。这两个文件不同于实际 r6 冻结包，**该后续修复尚无新 RK 或行驶验收记录**，不得沿用 r6 的实车结果为它背书。

`trial_navigation_runtime.py` 仍为 r6 的 `be6e2118ada6e9642ad948e5634d0426459af92a2037dc413f773edbcfb5b105`。当时讨论的低 RPM 里程计与 AMCL 周期扫描更新尚未完成接入和验证；手推建图的真实 RPM 积分正常，不等于 trial 已修复这条链路。用户随后把工作改为手推新地图，旧图 trial 不应自动恢复。

RK 持久任务服务及产品 `submit/status/cancel` 接口仍待实现。当前 CLI 的配置检查、计算和内存模拟不代表实车接口已可用。

## 地图与现场数据未纳入本仓库

本仓库仅归档上位机程序和必要说明，未提交新旧地图、原始串口/ROS 日志、采集证据包或绑定某次运行的现场状态文件。r5 原始 PGM/YAML/grid、odom 原点及证据保留在原工作区和 RK 的历史采集目录。

供以后核对外部备份：r5 最终 PGM SHA256 为 `30642d3090dce285fd722678f54620c2a82efdd3b5b0764d72bbb78a755c79fa`，480×640 格，约 5 厘米/格。地图存在边界断裂和放射状细带，几何精度未验；轮径、轮距和雷达安装仍有未标定估计。没有明确的 map→odom 变换时，不得把软件 odom 原点或末位姿当作地图定位。

重新部署时需要单独提供地图、受信任 SSH 主机密钥和当前会话配置，并按新 boot、真实传感器来源与现场条件重新核对。任何新打包或可移植性适配均需独立记录其软件验证；它们不改变上述历史实车证据的适用范围。

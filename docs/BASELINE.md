# 手推建图基线

当前main仅保留手推建图、只读网页和对应测试。来源对话为“重新设计点到点导航”，ID `01a19f01-1ef9-76f3-b7d8-5c38faff4b96`；最后完成轮次 `01a1a072-129d-7792-ad7e-8912685d621f`，用户请求“新地图采集完成”。

本次按用户要求移除导航及阶段联调部分。保留的12个app文件与此前最新源码逐字节相同，来源见`provenance/source_manifest.json`。此前完整导入保存在`v0.1.0-baseline-20261103`，对应提交`9351dcc0e3ae3136dc65023aebe1861312bf7680`；当前子集不再携带原56项完整清单或旧阶段说明。

## 已实际完成的手推r5

r5是新的采集会话，复用r4冻结软件。实际软件包含7个运行源文件和2个手推测试文件，九源SHA完整保存在`baselines/manual-mapping-r5.json`，原archive SHA为`312707b5d01dae045438899f7c8213165add1f44d86daf6497486c9bc7c1e0f9`。

| 项目 | 历史结果 |
|---|---|
| 采集时长 | 641.012066秒 |
| 完整性 | 正常结束、0里程计缺段、recording_complete=true |
| 记录 | 6264个真实轮速/odom样本、8584帧前180°scan、523张地图消息 |
| 串口请求 | STATUS5852、FEEDBACK23123；STOP/ARM/SETPOINT均0 |
| 软件测试 | 手推包PC27项通过；新r5目录RK同27项通过；均0跳过 |
| 结束 | 串口关闭、query/writer退出、自有gmapping正常退出 |

历史证据tar SHA为`e722fe456c83f9f70e2ca5d70bc3ed0b496cf2a3cf2332745088ed3712b11ac0`，独立审计SHA为`99e4bd3dc499728c5c82917e8de581786fe651915ea809be03dcf82fd6c53e56`。证据文件保留在原工作区和RK，不提交本仓库。

## 只读查看器

manual_mapping_viewer.py、配套HTML及test_mapping_viewer.py保留对话中最新字节。原cache/handler曾实际用于r5显示；本仓库新增`scripts/run_viewer.py`把地址、用户名、record和known_hosts改为显式配置，绕开旧main的工作区默认路径。新增入口做了离线检查，没有另做现场SSH验收。

## 精度和当前状态

r5最终PGM SHA为`30642d3090dce285fd722678f54620c2a82efdd3b5b0764d72bbb78a755c79fa`，480×640格、约5厘米/格。地图几何精度未验，仍有边界断裂和放射状细带；轮径、轮距和雷达外参仍含未标定估计。没有明确map→odom时，不把odom原点或末位姿当地图定位。

原点、地图、boot和历史已结束状态不能当作下次当前状态。重新录图需核对当次设备和原点。本次仓库精简不连接RK、不改变运行程序或现场数据。

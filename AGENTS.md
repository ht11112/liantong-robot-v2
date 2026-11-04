# 手推建图仓库开发约定

- 先读README.md、docs/BASELINE.md、docs/USAGE.md。硬件联调前重新核实际运行状态，历史位姿、会话和现场确认不代表当前状态。
- 当前仓库只维护手推建图、配套只读网页和三份对应测试；不把点到点导航、阶段联调程序或其测试重新加入main。
- 用户已明确授权在`feature/simple-point-to-point`分支新增最简单的odom点到点功能；该分支可增加独立controller/base/runtime、配置和对应测试。保持原手推源及历史清单字节，不自动合入main，不恢复旧AMCL/阶段联调整树。
- 保留的12个原字节文件来源在provenance/source_manifest.json；此前完整导入只在历史Git标签保存。未来改动由Git追踪，不改写历史实测身份。
- 离线回归：python -B -m unittest discover -s app -p "test_*.py"。当前33项；独立手推包27项。测试不得访问设备。
- app/base_static_adapter.py及4个deps是手推程序的必要依赖，不能仅因文件名或共用辅助逻辑将其删除。
- 不加入STM32固件、刷写产物、现场地图、日志或凭据。新增功能使用分支，按docs/VERSIONING.md验证和发布，不强制覆盖远端历史或重写已有标签。

## 第二分支的MCU开发范围

- 用户已明确授权从f86ab2b建立`feature/mcu-odom-queue`，在本分支加入`mcu/`内G474 v10生产源码、协议工具及离线测试；此项覆盖上面的主机专用范围限制。原main、第一分支和12/9来源身份保持。
- 本轮只做MCU与协议工具，不实现RK规划/AMCL或修改现有网页。不得自动连接控制口、刷写或运行实车动作。
- 生产构建严格32KB，测试程序/长诊断/旧v9控制不进入生产镜像。生成BIN/ELF/map、编译缓存和运行证据留在忽略目录，提交源码及可复现构建脚本。
- MCU独立执行RAM队列，RK失联继续；MOVE不纠偏且不以8度偏航停车。段间实测零速交接保留，最终到点直接STOP、不做位置封存。禁止把离线测试称为硬件时序或行驶验收。

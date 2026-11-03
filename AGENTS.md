# 手推建图仓库开发约定

- 先读README.md、docs/BASELINE.md、docs/USAGE.md。硬件联调前重新核实际运行状态，历史位姿、会话和现场确认不代表当前状态。
- 当前仓库只维护手推建图、配套只读网页和三份对应测试；不把点到点导航、阶段联调程序或其测试重新加入main。
- 保留的12个原字节文件来源在provenance/source_manifest.json；此前完整导入只在历史Git标签保存。未来改动由Git追踪，不改写历史实测身份。
- 离线回归：python -B -m unittest discover -s app -p "test_*.py"。当前33项；独立手推包27项。测试不得访问设备。
- app/base_static_adapter.py及4个deps是手推程序的必要依赖，不能仅因文件名或共用辅助逻辑将其删除。
- 不加入STM32固件、刷写产物、现场地图、日志或凭据。新增功能使用分支，按docs/VERSIONING.md验证和发布，不强制覆盖远端历史或重写已有标签。

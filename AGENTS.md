# 上位机仓库开发约定

- 先读 README.md、docs/BASELINE.md 和 docs/USAGE.md。硬件联调前重新核实际运行状态，历史位姿、会话、现场确认和实测结果不代表当前状态。
- 本仓库只维护 RK／PC 上位机；不加入 STM32 固件、刷写产物、旧整套导航源码、现场日志、地图或凭据。
- 首次导入 app/ 的源文件字节和来源在 provenance/source_manifest.json。它是固定导入记录；后续修改由 Git 提交追踪，不伪造已运行版本的来源。
- 现有离线回归：python -B -m unittest discover -s app -p "test_*.py"。测试不得访问真实设备；实机结果单独注明版本和现场条件。
- 手推软件的已运行基线见 baselines/。更新程序不等于更新该历史基线，未实际验证的修复如实标注。
- 新功能使用 feature/ 分支，按 docs/VERSIONING.md 提交、验证和打标签。不要重写已有发布标签，不强制覆盖远端历史。

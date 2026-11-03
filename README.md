# 联通机器人 RK 上位机

本仓库独立保存“重新设计点到点导航”对话截至 **2026年11月3日最后一次手推建图 r5 保存完成**的最新上位机源码，便于后续开发、对比与回退。

包含 RK 上的 Python／ROS 程序、底盘主机通信依赖、PC 只读建图网页和离线测试。STM32 固件、刷写工具、旧导航工程、现场地图、采集日志及登录凭据不在本仓库。

## 当前状态

- 最新实际完成的功能是手推创建新地图和只读查看：r5 记录约641秒，0次里程计缺段，正常结束并保存。地图几何精度尚未验证。
- 新点到点导航的配置、状态机、定位、路径预览和一次性运动试验源码一并保存；半米行驶到点尚未通过验收。
- 当前停车反馈过滤修复比最后一次 r6 运动试验包更新，尚无对应的新 RK 实机验证。
- r5 新地图尚未接入导航。历史导航运行入口仍引用旧地图身份；仓库不携带旧地图，也不提供自动启动这些试验的部署入口。

来源、源文件 SHA256 和准确验证边界见 [基线记录](docs/BASELINE.md) 与 [来源清单](provenance/source_manifest.json)。`docs/original/` 保留原对话的说明文件；其中旧工作区路径和历史运行状态按历史记录理解。

## 目录

| 路径 | 内容 |
|---|---|
| `app/` | 当前新上位机程序、配置、网页和现有测试，保持原始字节 |
| `app/deps/` | 从冻结包提取的6个必要主机 Python 依赖 |
| `scripts/` | 新增的独立离线打包、来源核对及可配置网页启动工具 |
| `baselines/` | 已运行的手推建图软件身份 |
| `docs/` | 当前状态、使用范围与版本回退说明 |
| `provenance/` | 首次导入的源文件来源与 SHA256 |

## 离线检查

PC 使用 Python 3.10。建议在独立环境安装本次验证的依赖版本：

```text
python -m venv .venv
```

Windows 后续使用 `.venv\Scripts\python.exe`，Linux 使用 `.venv/bin/python`。以下 `python` 指该环境的解释器：

```text
python -m pip install -r requirements-pc.txt
python -B -m unittest discover -s app -p "test_*.py"
python -B app/p2p_cli.py validate
python -B app/p2p_cli.py demo
python -B scripts/verify_source.py
python -B scripts/build_package.py --help
python -B scripts/run_viewer.py --help
```

普通 `validate` 会列出尚未填写的现场参数；`validate --complete` 有待定项时返回2，不能据离线校验推断已经可以行驶。测试不连接机器人。首次导入来源检查会比较固定导入清单；未来主动修改源码后，应通过 Git 比较和测试验证变更，原清单保留作来源记录。

RK 继续使用 ROS1 Noetic 和系统 Python 3.8；ROS、消息、TF、gmapping 等由原系统提供。本次 PC 依赖文件不用于覆盖 RK 的系统 NumPy／ROS 环境。

## 使用与版本管理

手推包和只读网页的独立使用方式见 [使用说明](docs/USAGE.md)。源码中的 `trial_config.json` 现场确认属于历史试验记录，不是下一次执行许可。

首次导入标签为 `v0.1.0-baseline-20261103`，表示可追溯的开发基线。后续新功能用 `feature/功能名` 分支，测试后合并，实测记录与软件提交分别标明。具体命令见 [开发与回退](docs/VERSIONING.md)。

上传代码不会改变 RK 正在运行的程序；RK 回退需要重新部署对应版本，并匹配其配置、依赖和外部地图。

# 开发、版本与回退

首次导入：`v0.1.0-baseline-20261103`。这是“重新设计点到点导航”对话的最新开发快照；手推 r5 与未完成验收的导航试验具有不同验证范围，详见 BASELINE.md。

## 新增功能

在本仓库根目录执行：

```text
git switch main
git pull --ff-only
git switch -c feature/功能名称
```

完成一个独立改动后运行现有离线测试，检查 `git diff`，再提交并推送：

```text
python -B -m unittest discover -s app -p "test_*.py"
git status --short
git add app scripts docs
git diff --cached
git commit -m "说明本次功能和变化"
git push -u origin feature/功能名称
```

在 GitHub 创建 Pull Request，记录改动、测试和是否做过实机验证。建议每个功能用一次 Squash and merge 合入 main，让撤销一个功能对应一个提交。GitHub Actions 只做离线测试和软件包构建，不连接 RK。

完成一轮验证后，给对应提交添加新标签，例如：

```text
git switch main
git pull --ff-only
git tag -a v0.1.1 -m "说明验证范围和剩余限制"
git push origin v0.1.1
```

标签指向固定代码。保留原标签，后续修复使用新标签。需要管理地图、配置或 RK 系统依赖时，在发布记录中写明各自的版本或 SHA256。

## 查看原始导入版本

```text
git fetch origin --tags
git diff v0.1.0-baseline-20261103..main --stat
git worktree add ../liantong-robot-v2-baseline v0.1.0-baseline-20261103
```

最后一个命令在另一个目录检出基线，便于比较、测试和离线构建；当前开发目录的改动保留。

## 撤销有问题的新功能

找到对应功能的提交 SHA 后：

```text
git switch main
git pull --ff-only
git switch -c fix/revert-feature
git revert 功能提交SHA
python -B -m unittest discover -s app -p "test_*.py"
git push -u origin fix/revert-feature
```

为这条分支创建 Pull Request 并合并。`git revert` 新增一条撤销提交，保留原功能和撤销过程，适合后续排查。不需要 `reset --hard` 或强制推送。

## RK 回退

源码回退与 RK 运行版本分别管理。从目标标签检出代码，离线测试并重新构建软件包，记录包 SHA256、Python／ROS 依赖和现场配置；再按当次实际运行状态进行部署。现场地图、odom 原点、boot ID 和会话记录保存在仓库外，不能拿旧记录当作下一次当前状态。

首次基线仅提供手推建图的独立构建。历史导航程序依赖旧地图及历史实验条件，不应直接用本标签启动运动验收。

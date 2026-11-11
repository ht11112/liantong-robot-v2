# V13 引导直行与 STEER

第十分支采用 V13 半双工请求应答。RK 是唯一串口主站，STM32 在请求后返回当前累计里程计及队列状态。波特率 115200、8N1，字节序为小端。帧头为 `A5 5A`，版本字节为 13。CRC16 CCITT-FALSE 初值 `FFFF`，覆盖版本至载荷末尾。

## 请求布局

| 偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0 | 2 字节 | `A5 5A` |
| 2 | uint8 | 版本 13 |
| 3 | uint8 | 操作码 |
| 4 | uint16 | 载荷长度 |
| 6 | uint64 | MCU 启动身份 boot |
| 14 | uint32 | 里程计代次 epoch |
| 18 | uint32 | 请求号 request |
| 22 | uint32 | 保留字段，填 0 |
| 26 | uint64 | 任务号 task |
| 34 | 可变 | 操作码附加字段 |
| 最后 2 字节 | uint16 | CRC16 |

BEGIN=1、APPEND=2、COMMIT=3、START=4、STOP=5、RESET=6、QUERY=7、RENEW=8、STEER=9。BEGIN 与 APPEND 各 46 字节，STEER 为 44 字节，其余请求为 36 字节。发现 QUERY 使用 boot、epoch、request 和 task 全零，其他请求绑定当前身份及递增非零请求号。

## 引导队列

每段 canonical 数据为 `<If`，由 uint32 类型和 float32 米或弧度组成。普通 MOVE=1，TURN=2，引导 MOVE=3。CRC32 覆盖全部 canonical 段拼接，任务最多 128 段。上传经 BEGIN、逐条 APPEND、COMMIT 后到 READY，START 才开始执行。

JSON 仍使用 `kind="move"`、`distance_m`，通过布尔值 `guided=true` 选类型 3。`guided=false` 或省略该字段选类型 1。标准 MOVE/TURN 数值范围和 float32 量化验证复用历史 codec，V13 包装层计算引导段类型及新的 CRC32。

```json
{"segments": [{"kind": "turn", "angle_rad": 1.0}, {"kind": "move", "distance_m": 0.5, "guided": true}]}
```

引导段进入时先给零目标，收到有效 STEER 后按该段 MOVE 基速执行差速修正。MCU 按实测中心轮程判断段进度。普通 MOVE 容差 2 厘米，末段 MOVE 容差 10 厘米，TURN 容差 3°。

## STEER

附加字段为 `<HhI`，共 8 字节。

| 偏移 | 类型 | 内容 |
| --- | --- | --- |
| 34 | uint16 | 当前引导 MOVE 段索引 index |
| 36 | int16 | bias_millirpm，范围 -2000～2000 |
| 38 | uint32 | basis_measurement_tick_ms，修正所依赖的真实 MCU 轮速测量时刻 |

偏置单位为 0.001 RPM。`+1250` 表示向左修正 1.25 RPM，通过降低左轮给定实现。负值降低右轮给定，零值维持等速。MCU 以整数 RPM 目标槽累计小数余量，单槽修正幅度受该段基速限制，轮速保持正向。CAN 应用变慢时目标槽顺延。

RK 每约 100 毫秒根据新鲜融合状态发送一次 STEER，使用独立串口进程的一个事务时隙。STM32 接受时要求任务、身份、当前索引和引导直行状态相符，租约有效，驱动健康且原始里程计有效。基础测量时刻相对 MCU 当前时刻必须小于 500 毫秒，并按 uint32 回绕处理，较旧的基础测量被拒绝。接收新的请求号但仍使用相同基础测量，不延长该基础测量的有效期限。

首个有效修正到达前的等待达到 500 毫秒，或后续修正基础测量到龄 500 毫秒，MCU 以任务错误 9 执行停车。30 秒任务租约及 RENEW 独立计时。

## 应答与主站接口

应答仍为 110 字节，类型 128。字段布局、ACK 请求号、结果码、累计里程计、真实测量序号、任务状态和剩余租约沿用 V12。flags 的 bit 9（值 512）为 `STEER_VALID`，表示当前引导 MOVE 已有未过期修正。首次发送 STEER 可在此标志尚未置位时进行。

```python
master.steer(task, index, bias_millirpm, basis_tick)
worker.steer(task, index, bias_millirpm, basis_tick, revision=stop_revision)
```

`RkMaster` 和 `FeedbackWorker` 沿用文件名 `mcu_v11.py` 与 `v12_feedback_worker.py`。STEER 的 ACK 进入共享反馈缓存，导航读取同一缓存。主站按实际发送时刻计算应答期限，APPEND、START 和 STEER 默认等待 1 秒，其余事务默认 100 毫秒，所有控制事务串行处理。串口写入超时仍使用默认 100 毫秒；STEER 应答等待不延长 MCU 的 500 毫秒修正有效期。STOP 的取消代次在命令出队和实际串口写入前核对，已取消的 STEER 不发送。

MCU 显式拒绝保留原始 ACK 和结果。STEER 应答超时或其他结果不确定时，主站结束本次通信会话。每次调用发送至多一条 STEER。V13 主站只接受版本 13 应答，V13 固件只执行版本 13 控制帧。

## 离线验证

主站测试覆盖引导段类型及 CRC、44 字节 STEER、正负小数 RPM 单位、uint32 基础时刻、512 标志、旧 V12 帧拒绝、迟到应答、显式拒绝和 STOP 优先。真实 spawned worker 测试核对 STEER 应答进入共享缓存，以及取消、拒绝和超时后的状态。

```text
python -B -m unittest discover -s mcu/tests -p "test_rk_master.py"
python -B -m unittest discover -s app -p "test_v12_feedback_process.py"
```

# 静止、纯监听的v10采集验收

`passive_capture.py`只负责串口上行数据证据；不发送查询、STOP、ARM、START、RESET、flush或输入缓冲清空，不自动复位。默认操作为离线analyze，导入不加载pyserial。只有显式`capture --execute --port`才打开串口，使用exclusive与115200 8N1，打开前将DTR/RTS设为关闭，无自动复位脉冲；USB设备或操作系统自行开口行为仍需现场核对。它不能代替root的设备身份、刷写回读及SWD驱动健康检查。

## root使用的最小部署与命令

同一目录放`passive_capture.py`及其冻结依赖`mcu_v10.py`即可；PC或RK使用Python3.8+，采集时需要pyserial，离线分析仅标准库。不部署固件源码、旧v9客户端或任何发车入口。

先完成SWD健康核对，取得当次MCU boot/odom epoch。保持机器人静止，确保没有其他串口owner，然后由root显式运行：

```text
python -B passive_capture.py capture --execute --port 实际串口 --output-dir 全新证据目录 --duration 65 --expected-boot 0x当次SWD读取值 --expected-epoch 当次epoch
```

默认记录65秒，为**至少60秒完整帧跨度**留出打开、首末帧边界余量；不以捕获elapsed冒充完整数据跨度。启动故障、旧boot、完整坏CRC或中途噪声不会被自动剔除。打开口时恰落在帧中间的初始≤109字节片段，以及末尾未完成的合法遥测前缀单独记为boundary fragment并保留原字节；完整坏帧仍计CRC错误。若发现故障，应保留此次目录，由root排查并在稳定的新上电身份下另外采集，工具没有“清洗后重试”功能。

离线复核（默认analyze、不打开串口、不改原记录）：

```text
python -B passive_capture.py --input-dir 既有采集目录 --output-dir 全新分析目录 --expected-boot 0x当次SWD读取值 --expected-epoch 当次epoch
```

`capture_stream(reader, output_dir, duration_s=65, monotonic=..., wall=..., expected_boot_id=..., expected_epoch=...)`供纯内存测试使用，不打开reader。实际serial reader只暴露read/close；write、flush、输入/输出清空、break能力被代理拒绝并记录，一旦发生尝试，采集完整性与验收均失败。分析期指定的期望身份不能静默覆盖采集时不同的绑定。

## 原始证据与三个结果字段

- `raw.bin`保留每次read返回的全部原始字节，不丢噪声、CRC失败、重复或其他boot。
- `chunks.jsonl`每次read均记录原始read开始/返回monotonic、返回wall time、连续byte offset/length及raw已flush；空read及读异常也记录。每次read后先flush原始数据，再flush对应记录，结束fsync。
- `capture.json`记录参数、来源、原始文件SHA、工具/codec SHA、期望boot/epoch及采集异常。打开口失败、读异常或禁止动作也保存失败记录。
- `frames.jsonl`记录原byte offset、原hex、包含末字节的read时间、解码字段与候选CRC/格式错误；坏CRC重同步候选可能与后续事件范围重叠，`raw.bin`始终为权威原件。非有限float以null显示且保留原hex，不能因JSON输出失败丢失证据。
- `report.json`分开给出`capture_complete`（是否无异常记录满请求时长）、`data_valid`（采集完整、字节覆盖、帧/来源序列完整）、`acceptance_pass`（所有静止链路门通过）。不能仅凭进程退出码宣布验收。零数据即使等够时间，也不会data_valid或acceptance_pass。

## 验收与局限

MCU publication tick与host read receipt分别统计rate、median、nearest-rank p99、max/min。两者均要求19–21Hz、median49–51ms、p99≤60ms、最大间隔≤100ms；完整帧跨度≥60秒，首尾静默均≤100ms。host receipt是原始read返回时间，不是精确线缆字节时间；USB合包会明确影响host指标，不拿MCU时钟替换它。

publication sequence必须逐帧+1，无丢失、重复或回退，CRC/格式/中途噪声为0；boot与epoch全程唯一且与显式SWD参考一致。每帧需要odom initialized/valid及MCU发布时的原measurement源龄<250ms。采样与发布异相，因此相同sample_seq允许重复，但其measurement tick、pose/v/w、原始编码器与RPM必须逐字节保持原样；前进sample_seq需measurement tick前进，回退拒绝。跨过未观察到的sample序号只报告`skipped_observations`，不冒称MCU内部漏积分。采样名义rate用sample序号跨度/measurement时间跨度计算，固件内部连续性以ODOM_VALID为准。

静止要求两原始RPM绝对值≤2（即0.2RPM）、首因及driver error均0、UART锁存false、MQ_IDLE、count/index/task均0、无START/执行/提交/到点/ACK痕迹、accepted request水位0。odom漂移仅报告原初末pose、位移/角度变化与最大偏移，不自动置零或新增漂移阈值。所有动作相关字段缺证都不能推导成“没有其他设备发过命令”；该工具不是外部RS485总线嗅探器。

正常boot遥测的MQ_STOP_CONFIRMED可以为false，不因此判失败；遥测没有driver state/guard字段，IDLE和error0不能证明驱动健康。报告明确`acceptance_scope=passive_telemetry_only`、`driver_guard_verified_by_tool=false`及`physical_stop_verified_by_tool=false`。root只有把**同boot/epoch**的SWD健康证据与这份链路验收同时核对后，才能报告整体静止烧录验收；任意重启或身份变化需重新核对，工具不会自动认证新boot。

单独离线测试：`python -B -m unittest discover -s mcu/tests -p test_passive_capture.py -v`。全部使用内存reader/临时文件，包括零数据、重复/回退、CRC/噪声、旧故障boot、样本异相、读异常及禁止写能力注入，不访问设备。

# V2 重复采集：启动失败

- 目标：原 V2 配置重复一次正反向有限采集；不改固件、PID、偏置或激励。
- 连接：Jetson 192.168.1.108 SSH 已恢复，三个 Horco DAP 正常枚举，业务口 332107148384 对应 ttyACM2；检查时没有其他进程占用三个 ACM 口。
- 采集脚本 SHA256：85e3d334a5eacb9cc36e87c5990dc96c84e8505dac9ac35aa2c39c752cbfabc8，与上轮一致。
- 执行：sentinel_identify.py --execute 在 hello 握手写入时失败，ProtocolError: Write timeout，退出码 1。
- 原始 summary：complete=false，trials=[]，sent=0，replies=0。未取得维护控制权，未发运动目标，没有运动样本。
- 复核：仅执行 hello，并将串口 write_timeout 从 0.05 s 临时改为 1 s，仍返回 SerialTimeoutException: Write timeout；未改源码或持久配置。
- 已取回：本目录 summary.json、wire.jsonl。
- 下一步：重新插拔业务串口 DAP 332107148384（原 ttyACM2），恢复 USB 串口写入后再执行原配置采集。此轮没有可分析的运动数据。

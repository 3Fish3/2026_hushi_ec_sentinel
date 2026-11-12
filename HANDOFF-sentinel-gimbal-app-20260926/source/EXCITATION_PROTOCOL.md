# 实验协议与集成接线

## 调用链

[board_app](<boards/sentinel_f407/src/board_app.cpp>)创建空默认[deployment](<boards/sentinel_f407/include/board_experiment_deployment.hpp>)，把[session](<robots/sentinel/app/sentinel_protocol.hpp>)中的Controller绑定到[cycle](<robots/sentinel/app/sentinel_cycle.cpp>)。control完成当周期RC/authority/fault更新后tick；commit执行常规executor，之后观察停止并把[autotune_trace](<robots/sentinel/app/sentinel_execution.cpp>)交给[recorder](<robots/sentinel/recording/raw_record.cpp>)。新的原子board入口在[board_raw](<boards/sentinel_f407/src/board_raw.cpp>)。capture-only角色拒绝Profile3/4执行证据采集。

[bridge](<robots/sentinel/app/sentinel_experiment_bridge.hpp>)只组合现存证据，不制造guardian批准。默认[Wire](<robots/sentinel/app/sentinel_experiment_wire.hpp>)为false仿真标志且hardware_blocked；[核心生成器/FSM](<robots/sentinel/app/sentinel_experiment.cpp>)不依赖PC计算。纯离线backend直接传入明确标记的synthetic Guards，不通过executor发送CAN。

## 业务封装

保留UART现有帧/CRC。业务版本3，request=0x60，response=0xca；10 B业务头：version u8、type u8、host session u32 LE、seq u32 LE。外层会话和防重放逻辑仍生效。内层Wire版本1/2共12 B命令头：version、op、axis、pattern各u8，experiment session和nonce各u32 LE。

axis=5 FireYaw /6 Pitch，pattern=1 bounded_step /2 trapezoid。op1 capability、2 status/heartbeat、3 prepare、4 arm、5 start、6 stop、7 result/heartbeat；2/7在身份匹配和授权存在时才续心跳。非prepare命令精确12 B。prepare精确88 B，包括6个非零身份字段、轨迹和预算参数及末尾保留零字节；精确偏移以[Wire解析器](<robots/sentinel/app/sentinel_experiment_wire.cpp>)为准，不能拿示例阈值当真实机械标定。

Wire v1回复32 B/v2回复128 B，加外层10 B；v2含boot身份、运行证据flags、观测epoch、批准版本、轴/stop stage、固件/部署digest。小于所需回复容量不执行Wire命令。hardware_capable只表示deployment通过校验，不等于运行授权。

与experiment identity匹配的STOP在维护权限丢失后仍可请求，但外层仍需有效host session/seq。已在DMA中的帧不可抢占；独立应答槽仅优先交付等待中的STOP应答，不能保证物理停车时间。STOP accepted != stop confirmed。

## Profile4

raw version1/layout0x52415701、32 B header、64 B record保持。Kind6 state与Kind7 aux共享stream、source_sequence、sampled_us、run；recorder接纳时分配连续全局sequence。同一计算tick成对或均不接纳；满连续队列lost+=2且不提交去重缓存。只接受5/6域，普通单条push不能混入Profile4。

固定512条=256对。MCU真正计算节拍可能不同：Fire按其group调度、Pitch按MIT group调度，不为未计算tick伪造样本。参数epoch反映观察到的实际增益变化，不是授权版本；raw run不是experiment session替代品。

stream帧最多3条完整64 B记录，因此同一pair可以跨传输包；分析器按身份/时刻配对，不能按每包恰好一对假设解析。当前实验硬件就绪不认可连续流带宽，使用冻结短窗。

[state/aux布局与Python解析](<robots/sentinel/host/sentinel_raw.py>)保留NaN和本地提交状态，不能将MIT命令或反馈电流自动换算为实际力矩。独立[离线CLI](<robots/sentinel/host/sentinel_experiment.py>)无实机端口入口；[低层client](<robots/sentinel/host/sentinel_client.py>)可编码协议，默认禁止motion命令，并不是完成的现场编排器。

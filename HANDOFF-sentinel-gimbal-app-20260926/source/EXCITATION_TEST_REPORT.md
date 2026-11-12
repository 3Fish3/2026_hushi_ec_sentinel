# 最终离线验收报告

日期：2026-11-01（America/New_York）。无设备连接、探针、烧录或实机运动。没有现场安全通过结论。

## 结果

|项目|结果|
|---|---|
|Host严格警告构建|通过；C++ -Wall -Wextra -Wpedantic -Werror|
|实验/记录/observer/云台相关定向CTest|8/8通过|
|最终全量CTest|61/62通过，36.56秒；不可报告全绿|
|唯一最终失败|doc_link_contracts：53个Markdown、243本地链接，225断链；未做隔离基线对比，不断言全部原本已有|
|先前出现Pitch hold时间预算失败|最终本轮通过；先前一次失败不隐瞒，不据此声称负载下确定性已证明|
|本包native后端+Python host合同|通过；覆盖两轴×两波形及解析/只读分析|
|四组持久化合成示例|各120样本，硬件未验证，未物理STOP确认|
|ARM构建及镜像检查|通过；仅newlib _close/_read/_write/_lseek无实现链接警告|
|ARM bin|245328 B；sha256 bca65c7c8f0062245adbaec1b0dff9d46c40bbb2a2f542eeb2c14082bec54c60|
|ARM elf sha256|0edd665dd3eb807ed38baccc71c48a15734129865b310961572d0db9049a7ada|
|静态RAM/FLASH|107352/131072 B（81.90%）；245328/1048576 B（23.40%）|
|WCET、动态栈峰值、停止时延、实机带宽|NOT_MEASURED|

RAM数为链接器静态布局，剩余23720 B不等于运行时安全余量或栈峰值已验证。CCMRAM使用0。

## 覆盖范围

- 生成器有界性、正负幅值/两种波形、无效参数、可复现随机扫；心跳、时钟回绕/延误和门禁故障。
- 协议长度/身份/序号/重复请求、默认空部署阻止、STOP应答优先、没有新CAN输出的封锁路径、活动实验不能写增益。
- 显式仿真不会进入板端目标接纳/电机提交；不拿硬件部署缺失当作仿真批准。
- Profile4仅域5/6、Kind6/7成对原子容量接纳、连续序号、满队列/丢条、重复源序号、NaN保持、原raw ABI、旧Pitch语义。
- observer使用动态Fire槽；按实际计算周期记录；Fire位置/速度和Pitch MIT参数epoch；CAN拒绝/无尝试不复用旧帧；观察前后CAN帧/控制状态一致。
- 真实recorder就绪与输出证据分离；错轴/缺采集/截止/满容量拒绝；busy不能形成已执行零帧的stop证据。

## 复现与证据

最终命令：

~~~sh
cmake --build build/host-tests -j 6
ctest --test-dir build/host-tests --output-on-failure
cmake --build build/gimbal-remote-verify-arm -j 6
~~~

交付包logs目录保存最终CTest日志；源码内工作日志在[最终CTest日志](<build/host-tests/excitation-final-ctest.log>)。主机/ARM构建摘要随交付提供。CRLF-aware whitespace检查通过，没有为消除警告整体换行格式化。

**接线与离线功能交付完成；实机主动激励未解锁，也未宣称Gate D通过。** 默认运行仍需区分既有遥控执行和新增实验封锁。

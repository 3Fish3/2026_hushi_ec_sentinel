# 云台原始采集主／副 Yaw 反馈映射修复说明

日期：2026-10-02  
交接包：`HANDOFF-sentinel-gimbal-app-20260926`  
交付对象：代码包维护者  
状态：源码修复、主机回归验证、ARM 构建、烧录、镜像读回校验及启动后只读采集验收完成；主／副 Yaw 映射正确，运动测试未执行。

## 1. 修改摘要

本次修复 `Recorder::push()` 在 `control` 采集模式下的域到 CAN 流映射。原代码把主、副 Yaw 的反馈选择反了。

| 修改文件（相对交接包的 source/） | 位置 | 修改内容 |
|---|---|---|
| `robots/sentinel/recording/raw_record.cpp` | `Recorder::push()`，当前第 17–18 行 | 交换映射表的第 4、5 项，并添加地址说明 |
| `tests/capture_tests.cpp` | 新增 `raw_control_domain_contracts()`，当前第 86 行；在 `main()` 第 155 行调用 | 验证全部 9 个 DJI 域的 CAN 地址、总线、流编号、载荷及错误总线过滤 |

代码补丁共 2 个文件，37 行新增、1 行删除。本说明文档另计。

## 2. 问题原因及影响

应用执行域编号和原始 CAN 流编号采用不同顺序：

| 轴 | 应用域编号 | 原始 CAN 流编号 | CAN 总线 | 反馈 ID |
|---|---:|---:|---:|---|
| 主 Yaw `main_yaw` | 4 | 5 | CAN1 | `0x208` |
| 副 Yaw `fire_yaw` | 5 | 4 | CAN1 | `0x20A` |

`Recorder::can()` 定义的 CAN 候选顺序包含 `…0x204, 0x20A, 0x208…`。旧过滤表使用 `mapping[4]=4`、`mapping[5]=5`，导致选择主 Yaw 域时保留副 Yaw 反馈，选择副 Yaw 域时保留主 Yaw 反馈。

修复后使用 `mapping[4]=5`、`mapping[5]=4`。其余域映射保持不变。

### 影响边界

- 错误影响 UART1 原始短窗的 `control` 模式 CAN 反馈选择。
- 主 Yaw 的控制记录仍来自主 Yaw，包括请求目标、采用目标、关节角、速度及归一化输出；旧短窗中的 CAN 记录却来自副 Yaw，不能把两者用于同一电机的输出／反馈关联分析。
- UART2 维护协议单独读取的主 Yaw 角度和设备 effort 不依赖这张过滤表，不受本次映射错误影响。
- 本补丁不修改 PID、目标执行逻辑、CAN 输出编码、参考零位或电流上限。此修复不能作为电机停顿、起动延迟或超调已经解决的证明。
- 已有 `.ecraw` 日志仍保留原来的错配。仅修改其标签无法恢复未采到的主 Yaw 反馈；需要在部署修复后重新采集。

## 3. 完整代码 Diff

以下 Diff 来自本次源码和测试修改，路径以 Sentinel 仓库根目录为基准，不包含本说明文档。

```diff
diff --git a/HANDOFF-sentinel-gimbal-app-20260926/source/robots/sentinel/recording/raw_record.cpp b/HANDOFF-sentinel-gimbal-app-20260926/source/robots/sentinel/recording/raw_record.cpp
index f6c9af6..76d22dd 100644
--- a/HANDOFF-sentinel-gimbal-app-20260926/source/robots/sentinel/recording/raw_record.cpp
+++ b/HANDOFF-sentinel-gimbal-app-20260926/source/robots/sentinel/recording/raw_record.cpp
@@ -14,7 +14,8 @@ void Recorder::push(Record r, MicroTick now) noexcept {
     case Kind::can:
         if (profile_ == Profile::stationary || (profile_ == Profile::rotation && r.stream != 4 && r.stream != 5 && r.stream != 9)) { return; }
         if (profile_ == Profile::control) {
-            constexpr std::array<std::uint8_t, 10> mapping{0, 1, 2, 3, 4, 5, 9, 6, 7, 8};
+            // 主 yaw 域4对应 CAN 流5 (0x208), 副 yaw 域5对应流4 (0x20a).
+            constexpr std::array<std::uint8_t, 10> mapping{0, 1, 2, 3, 5, 4, 9, 6, 7, 8};
             if (r.stream != mapping[selected_domain_]) { return; }
         }
         key = 3 + r.stream; period = profile_ == Profile::rotation ? (continuous_ ? 4000 : 2000) : 1000; break;
diff --git a/HANDOFF-sentinel-gimbal-app-20260926/source/tests/capture_tests.cpp b/HANDOFF-sentinel-gimbal-app-20260926/source/tests/capture_tests.cpp
index 40f68b7..f71e799 100644
--- a/HANDOFF-sentinel-gimbal-app-20260926/source/tests/capture_tests.cpp
+++ b/HANDOFF-sentinel-gimbal-app-20260926/source/tests/capture_tests.cpp
@@ -83,6 +83,40 @@ void raw_contracts() {
     CHECK(tx.frames.size() == 50);
     for (const auto& frame : tx.frames) { CHECK(cap::allowed_read(frame) && frame.data[2] == 0xcc); }
 }
+void raw_control_domain_contracts() {
+    namespace raw = ec::sentinel::recording;
+    struct Expected { std::uint8_t domain, stream, bus; std::uint16_t id; };
+    // 域编号与 CAN 流编号不是同一顺序; 独立列出真实反馈地址作为断言依据.
+    constexpr std::array<Expected, 9> expected{{
+        {0, 0, 2, 0x201}, {1, 1, 2, 0x202}, {2, 2, 2, 0x203}, {3, 3, 2, 0x204},
+        {4, 5, 1, 0x208}, {5, 4, 1, 0x20a},
+        {7, 6, 2, 0x205}, {8, 7, 2, 0x206}, {9, 8, 2, 0x207}
+    }};
+    for (const auto& selected : expected) {
+        raw::Recorder recorder;
+        std::array<std::uint8_t, 12> request{1, 1, 3, selected.domain};
+        raw::put32(request, 4, 100 + selected.domain); raw::put32(request, 8, 100000);
+        std::array<std::uint8_t, 240> reply{};
+        CHECK(recorder.request(request, reply, {1000}) == 32 && reply[2] == 0);
+        // 同时注入所有设备以及错误总线, 必须只保留所选域的真实反馈.
+        for (const auto& source : expected) {
+            const ec::transport::CanFrame frame{source.id, 8, {1, 2, 3, 4, 5, 6, 7, 8}};
+            recorder.can(source.bus == 1 ? 2 : 1, frame, {2000}, {2000}, 0);
+            recorder.can(source.bus, frame, {3000}, {3000}, 0);
+        }
+        CHECK(recorder.size() == 1);
+        request[1] = 2;
+        CHECK(recorder.request(request, reply, {4000}) == 32 && reply[3] == 2);
+        request[1] = 3; raw::put32(request, 8, 0);
+        CHECK(recorder.request(request, reply, {4000}) == 96 && reply[9] == 1);
+        const auto record = raw::decode(std::span<const std::uint8_t>{reply}.subspan(32, 64));
+        CHECK(record && record->kind == raw::Kind::can && record->stream == selected.stream);
+        CHECK((record->data[0] & 255u) == selected.bus);
+        CHECK(((record->data[0] >> 8) & 255u) == 8);
+        CHECK((record->data[0] >> 16) == selected.id);
+        CHECK(record->data[1] == 0x04030201u && record->data[2] == 0x08070605u);
+    }
+}
 void raw_golden() {
     namespace raw = ec::sentinel::recording;
     raw::Record record{raw::Kind::gyro, 0, 3, 0, 4, 0xfffffff0u, 20, 1, 600, 9};
@@ -118,6 +152,7 @@ int main(int argc, char** argv) {
     if (argc == 2 && std::strcmp(argv[1], "--raw-golden") == 0) { raw_golden(); return 0; }
     if (argc == 3 && std::strcmp(argv[1], "--raw-replay") == 0) { return raw_replay(argv[2]); }
     raw_contracts();
+    raw_control_domain_contracts();
     if (argc == 2 && std::strcmp(argv[1], "--golden") == 0) { golden(); return 0; }
     for (unsigned rid = 0; rid < 256; ++rid) {
         const auto f = cap::read_request(static_cast<std::uint8_t>(rid));
```

## 4. 验证结果

### 修复前后对照

先添加测试，保持原映射不变：`ec_capture_tests` 在 CAN 流编号断言处失败，输出 `capture check failed at 113`。修改映射并重新编译后，输出 `capture contracts passed`。

离线独立验证结果：

```text
修复前：domain=4 expected_feedback=0x208 captured=0x20a FAIL
修复前：domain=5 expected_feedback=0x20a captured=0x208 FAIL
修复后：domain=4 expected_feedback=0x208 captured=0x208 PASS
修复后：domain=5 expected_feedback=0x20a captured=0x20a PASS
```

使用现有驱动代码编译的离线 CAN 编码检查也通过：主 Yaw 归一化输出 `+0.0774 / -0.0774 / 0` 分别编码为 `0x1FE` 第四槽 `+1268 / -1268 / 0`，前三槽为零。该检查使用模拟发送端，未抓取实际 CAN 总线帧。

### 完整主机测试

使用交接包现有 `.venv/bin/python` 执行 CTest，共 47 项：41 项通过、5 项失败、1 项跳过。

| 未通过项目 | 原因 |
|---|---|
| `sentinel_raw_calibration_contracts` | 缺少 `numpy`，无法运行 |
| `motor_bench_console_contracts` | 缺少 `numpy`，无法运行 |
| `sentinel_feedback_audit_contracts` | 缺少 `numpy`，无法运行 |
| `gm6020_identification_contracts` | 缺少 `numpy`，无法运行 |
| `doc_link_contracts` | 交接包文档检查发现 224 个断链 |
| `motor_bench_client_contracts`（跳过） | 脚本报告缺少 `numpy`，按既有跳过规则退出 |

本次未安装依赖、修改测试跳过规则或修复无关文档。新增映射回归测试、采集测试和采集流测试通过。全套测试尚未全部通过。

### 复现命令

在交接包根目录执行下列命令。`-S source` 指定源码；`-B .tmp/yaw-mapping-host` 将生成文件放入任务临时目录；`Debug` 保留调试断言；`EC_BUILD_TESTS` 和 `EC_BUILD_EXAMPLES` 启用主机测试及其依赖目标；`Python3_EXECUTABLE` 指定项目隔离环境。仅编译主机程序，不接入硬件。

```bash
cmake -S source -B .tmp/yaw-mapping-host \
  -DCMAKE_BUILD_TYPE=Debug \
  -DEC_BUILD_TESTS=ON -DEC_BUILD_EXAMPLES=ON \
  -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
```

编译采集测试目标，运行映射回归：

```bash
cmake --build .tmp/yaw-mapping-host --target ec_capture_tests
ctest --test-dir .tmp/yaw-mapping-host -R '^sentinel_capture_contracts$' --output-on-failure
```

`--target` 限定编译目标；`--test-dir` 指定测试目录；`-R` 的精确名称只选择采集测试；`--output-on-failure` 显示失败输出。若需验证全套测试，先编译全部主机目标：

```bash
cmake --build .tmp/yaw-mapping-host
ctest --test-dir .tmp/yaw-mapping-host --output-on-failure
```

本机已执行测试的构建目录为 `<PROJECT_ROOT>/.tmp/yaw-diagnostics/can-audit/host-build/`，详细结果在其 `Testing/Temporary/LastTest.log`。

## 5. 板端部署及交接状态

- Sentinel 仓库当前分支为 `main`，上述代码修改尚未提交。
- 已完成源码、测试、主机验证、ARM 镜像构建及板端烧录。未下发运行时 PID 修改或电机运动指令；新固件启动后将使用编译初值。
- 板端已启动修复版，UART1 身份查询确认新构建标签；只读验收时未持有维护权限，主／副 Yaw 均处于待机。修复版构建身份为 `app-gimbal-20261002-yawmap-fix`，镜像大小 207,240 字节，BIN SHA256 为 `0202bd1d4bbf3c4ebf0716ddcb2a4337a0a1c5b56f3ec34054946f646689de56`。离线镜像检查通过，镜像及独立 `IMAGE.json` 位于本机 `<PROJECT_ROOT>/.tmp/yaw-diagnostics/can-audit/arm-gimbal-mapping-fix/` 下。原交接包镜像和 `IMAGE.json` 保留不变，作为回退候选。本次在动力断开、Pitch 支撑及遥控急停确认后执行，擦写范围限定扇区 0–5，未操作 Option Bytes。完整原区域备份为构建目录下 `preflash-sectors0-5-retry.bin`（262,144 字节，SHA256 `2e16281db60d1199be8714eeaa45244456bcc3e5e4c32e4d7531a09cf82a88b1`）。最初全 Flash 备份不完整，不能用于回退。首次写入在擦除后发生算法超时；复位并暂停后重试成功，`verify_image` 通过，独立读回 `postflash-readback.bin` 与新镜像逐字节一致。烧录及读回日志保留在构建目录中。
- 本次使用本机 Arm GNU GCC 16.2.0 和完整 SDK `<PROJECT_ROOT>/Hushi_EC_2026_Refactor/build/deps/STM32Cube_FW_F4_V1.28.0` 构建；SDK 元数据哈希符合项目固定值。原镜像使用的工具链环境未在此交接包中保留，不能宣称新旧二进制只包含映射表字节差异。链接器报告 newlib 的 `_close/_lseek/_read/_write` 无宿主实现警告；链接、项目镜像检查、板端 Flash 校验及启动后的只读采集映射验收通过；运动响应及 PID 调试尚未验收。
- 2026-10-02 启动后分别采集 `control/domain=4` 和 `control/domain=5`，每窗约 100 ms：主 Yaw 采到 50 条 CAN1 `0x208`／流5反馈及 51 条控制域4记录；副 Yaw 采到 50 条 CAN1 `0x20A`／流4反馈及 51 条控制域5记录。两窗丢失计数为0，控制输出均为0，映射验收通过。本次没有申请维护、发送目标或写参数。报告与原始窗口位于构建目录：`runtime-acceptance-20261002-182052.json`、`runtime-main_yaw-20261002-182052.ecraw`、`runtime-fire_yaw-20261002-182052.ecraw`。
- 请保留旧日志，并注明采集映射缺陷；后续调参应使用修复部署后重新采集的数据。

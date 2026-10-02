#!/usr/bin/env python3
"""链接覆盖核对: 确认通用能力的目标代码真的进了 F407 链接产物.

背景: 板级目标启用 --gc-sections, 源码编进静态库并不等于被链接进镜像.
`ec_link_verify` 是一个**非部署**验证产物: 它用静态对象与假 sink 引用各通用组件,
因此这些符号必须出现在它的 nm 清单里; 本脚本只读该清单并核对符号.

用法 (由 CMake 在 ec_link_verify 链接后自动调用):
    python check_link_verify.py <nm 清单文件>

注意: 本脚本不检查也不放宽默认固件 sentinel_s1 的无输出审计规则.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

# 每个条目: (说明, 必须出现的符号子串之一)
REQUIRED = (
    ('速度环', ('VelocityLoop::step',)),
    ('位置串级', ('PositionLoop::step',)),
    ('热保护', ('ThermalController::update',)),
    ('控制权仲裁', ('ControlAuthority::update',)),
    ('执行上下文', ('ExecutionContext::evaluate',)),
    ('PID', ('Pid::step',)),
    ('DJI 设备', ('DjiDevice::receive',)),
    ('DJI 轴门面', ('DjiAxis::tick',)),
    ('DM 设备', ('DmDevice::receive',)),
    ('BMI088 解析', ('parse_bmi088_accel', 'parse_bmi088_gyro')),
    ('姿态估计', ('AttitudeEstimator::update',)),
    ('DR16 输入', ('Dr16Input::accept',)),
    ('UART 解帧', ('UartFrameDecoder::push',)),
    ('全向轮运动学', ('omni_wheel_speeds', 'omni_peak_wheel_speed')),
    ('监控注册表', ('Registry::end_update', 'Registry::read_page')),
    ('会话跟踪', ('SessionTracker::begin', 'SessionTracker::check_intent')),
    ('板级装配', ('assembly_route', 'assembly_step')),
)


def main() -> int:
    raw = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('EC_LINK_VERIFY_SYMS', '')
    if not raw:
        print('FAIL: 需要 nm 清单文件路径 (参数或 EC_LINK_VERIFY_SYMS)', file=sys.stderr)
        return 2
    path = Path(raw)
    if not path.is_file():
        print(f'FAIL: symbol list not found: {path}', file=sys.stderr)
        return 2
    symbols = path.read_text(encoding='utf-8', errors='replace')
    missing = []
    for label, names in REQUIRED:
        if not any(name in symbols for name in names):
            missing.append(f'{label} ({"/".join(names)})')
    if missing:
        print('FAIL: 链接产物缺少以下能力的符号 (可能被 --gc-sections 丢弃):', file=sys.stderr)
        for item in missing:
            print(f'  - {item}', file=sys.stderr)
        return 1
    print(f'result: pass ({len(REQUIRED)} 组符号都在 ec_link_verify 中)')
    return 0


if __name__ == '__main__':
    sys.exit(main())

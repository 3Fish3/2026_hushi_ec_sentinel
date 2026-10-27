"""Project-local launcher: inspect commands without importing hardware tools."""
import argparse
import importlib.metadata
import importlib.util
import json
import os
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# base, relative path, purpose; source tools never fall back to another snapshot.
TOOLS = {
    'monitor': ('common', 'board_tools/board_monitor.py', 'S1 串口观测（会发只读查询）'),
    'monitor-swd': ('common', 'board_tools/monitor_swd.py', 'S1 SWD 观测 / 离线自检'),
    'monitor-probe': ('common', 'board_tools/monitor_probe.py', '串口查询压力验证（硬件收发）'),
    'uart-probe': ('common', 'board_tools/uart_probe.py', '串口回环测试（硬件收发）'),
    'firmware': ('common', 'board_tools/firmware_tool.py', '固件 GUI；烧录探针检测仍依赖 Windows'),
    'bench': ('common', 'motor_bench/motor_bench_console.py', 'S5 GM6020 台架 GUI（可驱动）'),
    'bench-tk': ('common', 'motor_bench/motor_bench.py', 'S5 台架 Tk GUI（可驱动）'),
    'trace': ('common', 'motor_bench/motor_bench_trace.py', 'S5 记录离线分析 / 绘图'),
    'identify': ('common', 'motor_bench/motor_bench_identify.py', 'S5 离线模型辨识'),
    'tune': ('common', 'motor_bench/motor_bench_tune.py', 'S5 离线调参建议'),
    'experiment': ('common', 'motor_bench/motor_bench_experiment.py', 'S5 自动实验（会驱动电机）'),
    'verify': ('common', 'build_verify/check_firmware.py', 'S1/S5 固件 ELF 离线审计'),
    'verify-links': ('common', 'build_verify/check_link_verify.py', '离线核对 nm 符号清单'),
    'console': ('source', 'robots/sentinel/host/sentinel_console.py', 'APP 机构维护 GUI（可控制）'),
    'capture': ('source', 'robots/sentinel/host/sentinel_capture_station.py', '1 Mbps 连续只读采集站（Tk GUI / CLI）'),
    'capture-cli': ('source', 'boards/sentinel_f407/tools/sentinel_capture.py', '源码版本对应的采集 CLI'),
    'client': ('source', 'robots/sentinel/host/sentinel_client.py', 'APP 协议 CLI（可控制）'),
    'raw': ('source', 'robots/sentinel/host/sentinel_raw.py', 'APP 原始轨迹采集 / 导出'),
    'stream': ('source', 'robots/sentinel/host/sentinel_stream.py', '连续只读采集 CLI'),
    'calibrate': ('source', 'robots/sentinel/host/sentinel_calibrate.py', '离线标定'),
    'replay': ('source', 'robots/sentinel/host/sentinel_replay.py', '离线原生后端回放'),
    'dataset-calibrate': ('source', 'robots/sentinel/host/sentinel_dataset_calibrate.py', '数据集离线标定'),
    'dataset-replay': ('source', 'robots/sentinel/host/sentinel_dataset_replay.py', '数据集离线回放'),
    'feedback-audit': ('source', 'robots/sentinel/host/sentinel_feedback_audit.py', '离线反馈审计'),
    'app-test': ('source', 'robots/sentinel/host/sentinel_test.py', 'APP 测试工具（可控制，不是单元测试）'),
    'yaw-test': ('source', 'robots/sentinel/host/sentinel_yaw_test.py', 'Yaw 机构测试（会驱动电机）'),
}


def has_option(arguments, option):
    options = arguments[:arguments.index('--')] if '--' in arguments else arguments
    return any(value == option or value.startswith(option + '=') for value in options)


def linux_gui_defaults(name, source, arguments):
    defaults, warnings = [], []
    if has_option(arguments, '--help') or has_option(arguments, '-h'):
        return defaults, warnings
    if name in ('bench', 'bench-tk') and not has_option(arguments, '--port'):
        defaults += ['--port', '']  # Never choose or open a serial port for the user.
    if name in ('bench', 'console') and not has_option(arguments, '--font'):
        candidates = (
            '/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc',
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
            '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
        )
        font = next((path for path in candidates if Path(path).is_file()), None)
        if font is None:
            raise ValueError('未找到 Linux 中文字体，请显式传 --font /path/to/font.ttf（或 .ttc）')
        defaults += ['--font', font]
    if name == 'console' and not has_option(arguments, '--host-backend'):
        candidates = [source / 'build/sentinel-app-host' / directory / 'ec_sentinel_app_host'
                      for directory in ('', 'Release')]
        backend = next((path for path in candidates if path.is_file() and os.access(path, os.X_OK)), candidates[0])
        defaults += ['--host-backend', str(backend)]
        if not backend.is_file() or not os.access(backend, os.X_OK):
            warnings.append('Linux 仿真后端尚不存在或不可执行；串口 GUI 可打开，仿真连接需 --host-backend 指定已构建的原生文件。')
    return defaults, warnings


def environment_check():
    """Check installed metadata and local capabilities; never import a robot module."""
    print(f'Python: {ROOT / ".venv/bin/python"} ({sys.version.split()[0]})')
    failures = 0
    for line in (ROOT / 'requirements.txt').read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        name, expected = line.split('==')
        try:
            actual = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            actual = '未安装'
        ok = actual == expected
        failures += not ok
        print(f'{"OK" if ok else "FAIL"}: {name}={actual} (要求 {expected})')
    tk = importlib.util.find_spec('_tkinter') is not None
    failures += not tk
    print(f'{"OK" if tk else "FAIL"}: Tk 扩展模块{"存在" if tk else "缺失"}（未创建窗口）')
    print(f'INFO: X11/XWayland DISPLAY {"已设置" if os.environ.get("DISPLAY") else "未设置；GUI 需要桌面会话"}')
    for name in ('cmake', 'ninja', 'arm-none-eabi-gcc', 'openocd', 'pwsh'):
        print(f'INFO: {name}: {shutil.which(name) or "未找到（按需使用）"}')
    print('INFO: 未打开串口、未探测硬件；串口权限及 GUI 渲染需另行验证。')
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(prog='sentinel', description=__doc__, allow_abbrev=False)
    parser.add_argument('--source', type=Path, default=ROOT / 'Hushi_EC_2026_Refactor')
    parser.add_argument('--list', action='store_true', help='列出工具、用途、路径与可用状态')
    parser.add_argument('--dry-run', action='store_true', help='仅输出启动计划，不导入或运行工具')
    parser.add_argument('command', nargs='?', help='工具名称，或 env-check')
    parser.add_argument('arguments', nargs=argparse.REMAINDER, help='原样传给工具的参数')
    args = parser.parse_args()
    source = args.source.expanduser()
    source = (source if source.is_absolute() else ROOT / source).resolve()
    if args.list or args.command == 'list':
        print(f'源码目录: {source}（不自动切换交接版本）')
        print('ui [内置] 同窗口统一界面（默认入口）')
        print('env-check [内置] 环境检查，不打开串口或探针')
        for name, (base, relative, description) in TOOLS.items():
            path = (ROOT / 'common' if base == 'common' else source) / relative
            state = '可用' if path.is_file() else '缺失'
            print(f'{name} [{state}] {description}\n  {path}')
        return 0
    if not args.command:
        args.command = 'ui'
    if args.command == 'env-check':
        if args.arguments or args.dry_run:
            parser.error('env-check 不接受工具参数或 --dry-run；它本身不会启动工具')
        try:
            return environment_check()
        except (OSError, ValueError) as error:
            parser.error(str(error))
    if args.command == 'ui':
        target = ROOT / 'scripts/sentinel_ui.py'
        args.arguments = ['--source', str(source), *args.arguments]
    else:
        if args.command not in TOOLS:
            parser.error(f'未知工具: {args.command}；用 --list 查看可用命令')
        base, relative, _ = TOOLS[args.command]
        target = (ROOT / 'common' if base == 'common' else source) / relative
    if not target.is_file():
        parser.error(f'工具不存在: {target}；用 --source 显式选择包含此工具的源码目录，不自动回退')
    python = ROOT / '.venv/bin/python'
    try:
        defaults, warnings = linux_gui_defaults(args.command, source, args.arguments)
    except ValueError as error:
        parser.error(str(error))
    if args.command == 'firmware':
        if not (source / 'CMakeLists.txt').is_file():
            parser.error(f'固件工程缺少 CMakeLists.txt: {source}；请用 --source 选择工程')
        if has_option(args.arguments, '--root'):
            parser.error('firmware 的工程目录请使用全局 --source，不要另传 --root')
        defaults = ['--root', str(source)]
        warnings.append('固件 GUI 的烧录探针检测仍依赖 Windows Get-PnpDevice；本入口未移植或执行烧录。')
    command = [str(python), str(target), *defaults, *args.arguments]
    pythonpath = [str(source / 'robots/sentinel/host'), str(source / 'boards/sentinel_f407/tools')]
    if args.dry_run:
        print(json.dumps({'argv': command, 'cwd': str(Path.cwd()), 'source': str(source),
                          'pythonpath': pythonpath, 'warnings': warnings}, ensure_ascii=False, indent=2))
        return 0
    help_requested = has_option(args.arguments, '--help') or has_option(args.arguments, '-h')
    gui = args.command in ('ui', 'bench', 'bench-tk', 'console', 'firmware') or (
        args.command == 'capture' and not any(
            has_option(args.arguments, option) for option in ('--inspect', '--capture', '--self-test')))
    if gui and not help_requested and not os.environ.get('DISPLAY'):
        parser.error('GUI 需要 X11/XWayland DISPLAY；请在 Linux 桌面终端启动，或用 --dry-run 预览')
    for warning in warnings:
        print(f'注意: {warning}', file=sys.stderr)
    # Restrict extra project imports to the explicitly selected source tree.
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(pythonpath))
    env.pop('PYTHONHOME', None)
    try:
        os.execve(str(python), command, env)
    except OSError as error:
        parser.error(f'启动失败: {error}')


if __name__ == '__main__':
    raise SystemExit(main())

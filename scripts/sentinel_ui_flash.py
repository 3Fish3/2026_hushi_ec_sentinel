"""OpenOCD flash pipeline for ST-Link and CMSIS-DAP probes.

Every step runs through OpenOCD Tcl with fixed file names inside a private work
directory; user paths never enter a Tcl string. The pipeline aborts unless every
stage reports its success sentinel, and the backup is taken before the first
erase so the previous firmware is always recoverable.
"""
import contextlib
import hashlib
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time

FLASH_BASE = 0x08000000
FLASH_SIZE = 0x100000
SECTORS = [16, 16, 16, 16, 64, 128, 128, 128, 128, 128, 128, 128]  # KiB

STLINK_USB_PIDS = {'3744', '3748', '374b', '374d', '374e', '374f', '3752', '3753', '3754'}

PROBE_CONFIGS = {
    'ST-Link': ('source [find interface/stlink.cfg]\n'
                'transport select hla_swd\n'),
    'CMSIS-DAP': ('source [find interface/cmsis-dap.cfg]\n'
                  'cmsis_dap_backend hid\n'
                  'transport select swd\n'),
}

TARGET_CFG = ('source [find target/stm32f4x.cfg]\n'
                 'adapter speed 200\n'
                 'stm32f4x.cpu configure -event reset-start { adapter speed 200 }\n'
                 'reset_config none\n'
                 'gdb_port disabled\n'
                 'tcl_port disabled\n'
                 'telnet_port disabled\n')

TARGET_CHECK = ('init\n'
                'halt\n'
                'wait_halt 3000\n'
                'set id [lindex [read_memory 0xE0042000 32 1] 0]\n'
                'if {($id & 0xfff) != 0x413} { error "Not an STM32F405/407 target" }\n'
                'set kib [lindex [read_memory 0x1FFF7A22 16 1] 0]\n'
                'if {$kib != 1024} { error "Expected 1024 KiB flash" }\n'
                'echo FLASH_TARGET_OK\n'
                'shutdown\n')

BACKUP = ('init\n'
          'halt\n'
          'wait_halt 3000\n'
          'dump_image backup.bin 0x08000000 0x100000\n'
          'echo FLASH_BACKUP_OK\n'
          'shutdown\n')

PROGRAM = ('init\n'
           'reset halt\n'
           'for {set sector 0} {$sector <= @LAST@} {incr sector} {\n'
           '    flash erase_sector 0 $sector $sector\n'
           '}\n'
           'flash write_image image.bin 0x08000000 bin\n'
           'verify_image image.bin 0x08000000 bin\n'
           'echo FLASH_PROGRAM_VERIFIED\n'
           'reset run\n'
           'echo FLASH_PROGRAM_SUCCESS\n'
           'shutdown\n')


def sector_span(size):
    covered = 0
    for number, length in enumerate(SECTORS):
        covered += length * 1024
        if size <= covered:
            return number, covered
    raise ValueError('镜像超出 1MiB Flash 范围')


def prepare_image(elf, work):
    """objcopy to binary and validate the vector table of an F407 image."""
    objcopy = shutil.which('arm-none-eabi-objcopy')
    if not objcopy:
        raise ValueError('找不到 arm-none-eabi-objcopy；请安装 arm-none-eabi 工具链')
    elf, work = Path(elf).expanduser().resolve(), Path(work)
    if not elf.is_file():
        raise ValueError('ELF 文件不存在: ' + str(elf))
    work.mkdir(parents=True, exist_ok=True)
    binary = work / 'image.bin'
    result = subprocess.run([objcopy, '-O', 'binary', str(elf), str(binary)],
                            capture_output=True, text=True, timeout=60)
    if result.returncode != 0:
        raise ValueError('不是有效的 ELF 固件文件: ' + str(elf))
    data = binary.read_bytes()
    if not 0 < len(data) <= FLASH_SIZE:
        raise ValueError(f'镜像大小超出 Flash 范围: {len(data)} 字节')
    msp, reset = struct.unpack_from('<II', data)
    if msp != 0x20020000 or not reset & 1 or not FLASH_BASE <= reset - 1 < FLASH_BASE + len(data):
        raise ValueError('向量表不符合 STM32F407 镜像，请确认选择了正确的固件 ELF')
    return binary


def stage_cfg(work, name, body, probe='ST-Link'):
    adapter = PROBE_CONFIGS[probe]
    work.mkdir(parents=True, exist_ok=True)
    cfg = work / name
    cfg.write_text(adapter + TARGET_CFG + body, encoding='ascii')
    return cfg


def discover_elfs(root, source):
    """List build artifacts newest-first from the selected source, then the project root."""
    seen, results = set(), []
    for base in (Path(source).resolve(), Path(root).resolve()):
        for path in base.glob('build/**/*.elf'):
            resolved = str(path.resolve())
            if resolved in seen or not path.is_file():
                continue
            seen.add(resolved)
            results.append(resolved)
    results.sort(key=lambda path: Path(path).stat().st_mtime, reverse=True)
    return results


def probe_present(probe='ST-Link'):
    """按所选探针类型枚举 USB 设备."""
    if probe not in PROBE_CONFIGS:
        raise ValueError('未知探针类型: ' + probe)
    if not shutil.which('lsusb'):
        return False, '缺少 lsusb，无法枚举 USB 设备'
    output = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=20)
    if output.returncode != 0:
        return False, 'USB 枚举失败，无法访问调试探针；请确认当前桌面会话有 USB 权限: ' + (output.stderr or '').strip()
    for line in (output.stdout or '').splitlines():
        device = re.search(r'\bID\s+([0-9a-f]{4}):([0-9a-f]{4})\b', line, re.IGNORECASE)
        if not device:
            continue
        vid, pid = (value.lower() for value in device.groups())
        matched = (vid == '0483' and pid in STLINK_USB_PIDS) if probe == 'ST-Link' else (
            'cmsis-dap' in line.lower() or (vid, pid) in {('c251', 'f001'), ('faed', '4870')})
        if matched:
            return True, f'已检测到 {probe} 探针: ' + line.strip()
    return False, f'未检测到 {probe} 探针；请检查探针 USB 连接'


def target_check_command(root, openocd, work=None, probe='ST-Link'):
    """Read-only stage: connect, verify F407 identity, then disconnect."""
    openocd = str(resolve_openocd(openocd))
    scripts = find_scripts(openocd)
    work = Path(work) if work else Path(root).resolve() / 'build/flash-runs/latest'
    cfg = stage_cfg(work, 'check.cfg', TARGET_CHECK, probe)
    return [openocd, '-s', str(scripts), '-f', str(cfg)]


def find_scripts(openocd):
    base = Path(openocd).resolve().parent.parent
    for relative in ('scripts', 'share/openocd/scripts', 'openocd/scripts'):
        if (base / relative / 'target/stm32f4x.cfg').is_file():
            return base / relative
    raise ValueError('未找到 OpenOCD scripts 目录（target/stm32f4x.cfg）')


def resolve_openocd(openocd):
    """Accept a bare name (via PATH) or an explicit path; never invent one."""
    if not openocd.strip():
        raise ValueError('请指定 openocd 程序路径')
    found = shutil.which(openocd)
    if found:
        return Path(found).resolve()
    explicit = Path(openocd).expanduser().resolve()
    if explicit.is_file():
        return explicit
    raise ValueError('找不到 openocd 程序: ' + openocd)


def flash_command(root, elf, openocd, work=None, probe='ST-Link'):
    """Return the shell pipeline: target check -> backup -> erase/write/verify."""
    openocd = str(resolve_openocd(openocd))
    scripts = find_scripts(openocd)
    root = Path(root).resolve()
    work = Path(work) if work else root / 'build/flash-runs/latest'
    binary = prepare_image(elf, work)
    data = binary.read_bytes()
    last_sector, _ = sector_span(len(data))
    digest = hashlib.sha256(data).hexdigest()
    # Fixed work paths only; no user path ever reaches a Tcl string.
    stage_cfg(work, 'program.cfg', PROGRAM.replace('@LAST@', str(last_sector)), probe)
    stage_cfg(work, 'backup.cfg', BACKUP, probe)
    stage_cfg(work, 'check.cfg', TARGET_CHECK, probe)
    command = [sys.executable, '-u', str(Path(__file__).resolve()), '--run-stages',
               '--openocd', openocd, '--scripts', str(scripts), '--work', str(work.resolve()),
               '--digest', digest]
    pipeline = 'set -euo pipefail\nexec ' + shlex.join(command) + '\n'
    script = work / 'flash.sh'
    script.write_text(pipeline, encoding='utf-8')
    return ['bash', str(script)]


def run_openocd_stage(command, work, timeout, marker, label):
    """监控实际 OpenOCD 进程组；截止时间到达即清理该组并报告超时."""
    stop = False
    def cancelled(signum, frame):
        nonlocal stop
        stop = True
    previous = {s: signal.signal(s, cancelled) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        process = subprocess.Popen(command, cwd=work, start_new_session=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
    except BaseException:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        raise
    evidence = threading.Event()
    stdout = process.stdout
    assert stdout is not None

    def read_output():
        try:
            for line in stdout:
                print(line.rstrip('\n'), flush=True)
                if marker in line:
                    evidence.set()
        finally:
            stdout.close()

    def kill_group():
        with contextlib.suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGKILL)
    reader = threading.Thread(target=read_output, name='openocd-stage-output', daemon=False)
    started = False
    try:
        reader.start()
        started = True
        deadline = time.monotonic() + timeout
        while process.poll() is None:
            if stop:
                print(f'FLASH_STAGE_CANCELLED {label}：已取消。', flush=True)
                return 130
            if time.monotonic() >= deadline:
                print(f'FLASH_STAGE_TIMEOUT {label}：超过 {timeout:g} 秒，正在终止 OpenOCD，后续阶段不执行。', flush=True)
                return 124
            time.sleep(0.02)
        code = process.returncode
        # 即使 OpenOCD 自身退出, 派生进程也不能持有日志管道而拖住任务。
        kill_group()
        reader.join(timeout=2)
        if code:
            print(f'FLASH_STAGE_FAILED {label}：OpenOCD 退出码 {code}，已停止后续阶段。', flush=True)
            return code if code > 0 else 1
        if not evidence.is_set():
            print(f'FLASH_STAGE_FAILED {label}：缺少完成标记 {marker}，已停止后续阶段。', flush=True)
            return 1
        return 0
    except KeyboardInterrupt:
        print(f'FLASH_STAGE_CANCELLED {label}：已取消。', flush=True)
        return 130
    finally:
        kill_group()
        process.wait(timeout=3)
        if started:
            reader.join(timeout=3)
        elif process.stdout is not None:
            process.stdout.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def run_stages(openocd, scripts, work, digest):
    for index, (cfg, limit, marker, label) in enumerate((
        ('check.cfg', 20, 'FLASH_TARGET_OK', '目标检查'),
        ('backup.cfg', 90, 'FLASH_BACKUP_OK', '整片备份'),
        ('program.cfg', 180, 'FLASH_PROGRAM_SUCCESS', '擦写、校验、复位')), 1):
        print(f'=== 阶段 {index}/3 {label}（{limit} 秒超时） ===', flush=True)
        code = run_openocd_stage([str(openocd), '-s', str(scripts), '-f', cfg],
                                work, limit, marker, label)
        if code:
            return code
        if cfg == 'backup.cfg' and (not (work / 'backup.bin').is_file() or
                                     (work / 'backup.bin').stat().st_size != FLASH_SIZE):
            print('FLASH_STAGE_FAILED：备份文件不是完整 1 MiB，未开始擦写。', flush=True)
            return 1
    print(f'FLASH_ALL_DONE backup={work / "backup.bin"} image_sha256={digest}', flush=True)
    return 0


def check_probe_main():
    """在后台进程检测探针, 避免 USB 枚举或 OpenOCD 阻塞界面线程."""
    import argparse
    parser = argparse.ArgumentParser(description='只读检查调试探针与 STM32F407 目标')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check-probe', action='store_true')
    mode.add_argument('--flash', action='store_true')
    mode.add_argument('--run-stages', action='store_true')
    parser.add_argument('--root', type=Path)
    parser.add_argument('--work', type=Path)
    parser.add_argument('--elf', type=Path)
    parser.add_argument('--scripts')
    parser.add_argument('--digest', default='')
    parser.add_argument('--openocd', required=True)
    parser.add_argument('--probe', choices=list(PROBE_CONFIGS), default='ST-Link')
    args = parser.parse_args()
    try:
        if args.run_stages:
            if args.work is None or args.scripts is None:
                parser.error('--run-stages requires --work and --scripts')
            return run_stages(args.openocd, args.scripts, args.work, args.digest)
        if args.root is None:
            parser.error('--root is required')
        print('探针预检：' + args.probe, flush=True)
        present, message = probe_present(args.probe)
        print(message, flush=True)
        if not present:
            return 1
        if args.flash:
            if args.elf is None:
                parser.error('--flash requires --elf')
            print('正在准备并检查 ELF 镜像…', flush=True)
            command = flash_command(args.root, args.elf, args.openocd, probe=args.probe)
            bash_bin = shutil.which('bash') or '/bin/bash'
            os.execv(bash_bin, command)  # noqa: S606
        command = target_check_command(args.root, args.openocd, probe=args.probe)
        return run_openocd_stage(command, args.root, 20, 'FLASH_TARGET_OK', '目标检查')
    except KeyboardInterrupt:
        print('FLASH_STAGE_CANCELLED 操作已取消。', flush=True)
        return 130
    except subprocess.TimeoutExpired:
        print('探针检查超时：20 秒内未完成目标连接。请检查 ST-Link/CMSIS-DAP、SWD 接线及目标供电。', flush=True)
        return 1
    except (OSError, ValueError) as error:
        print('探针检查失败: ' + str(error), flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(check_probe_main())

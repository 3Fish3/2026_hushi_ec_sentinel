"""C板固件构建/审计/定界烧录; 导入模块不访问硬件."""
from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
from dataclasses import dataclass
from datetime import datetime

ROOT = Path(__file__).resolve().parents[3]
FLASH_BASE = 0x08000000
FLASH_SIZE = 1024 * 1024
SECTORS = (16384, 16384, 16384, 16384, 65536) + (131072,) * 7

@dataclass(frozen=True)
class Profile:
    key: str
    title: str
    build_dir: str
    target: str
    flags: tuple[str, ...]
    audit: str
    audit_flags: tuple[str, ...]
    host: str
    host_args: tuple[str, ...]
    port: str
    purpose: str

PROFILES = (
    Profile('chassis', 'APP / 实车底盘调试', 'app-chassis-arm-Release', 'sentinel_app',
        ('EC_BUILD_SENTINEL_APP', 'EC_APP_CHASSIS_COMMISSIONING'), 'check_sentinel_app.py', ('--chassis-commissioning',),
        'robots/sentinel/host/sentinel_console.py', ('--mechanism', 'chassis', '--serial'), 'UART2 / USART1 / 921600',
        '可驱动四轮. CAN2 ID1/2/3/4=LF/RF/LB/RB, CAD几何/19:1, 遥控首启; 其它机构撤输出.'),
    Profile('four', 'APP / 四电机台架', 'app-four-motor-arm-Release', 'sentinel_app',
        ('EC_BUILD_SENTINEL_APP', 'EC_APP_FOUR_MOTOR_BENCH'), 'check_sentinel_app.py', ('--four-motor-bench',),
        'robots/sentinel/host/sentinel_console.py', ('--mechanism', 'bench', '--serial'), 'UART2 / USART1 / 921600',
        '可驱动. CAN1: GM6020 ID4, M3508 ID2, M2006 ID3, J4310控制1/反馈0. 人工调参.'),
    Profile('gm', 'APP / 单GM6020台架', 'app-gm6020-manual-arm-Release', 'sentinel_app',
        ('EC_BUILD_SENTINEL_APP', 'EC_APP_GM6020_BENCH'), 'check_sentinel_app.py', ('--gm6020-bench',),
        'robots/sentinel/host/sentinel_console.py', ('--mechanism', 'gimbal', '--serial'), 'UART2 / USART1 / 921600',
        '可驱动. 仅CAN1 GM6020 ID4; 人工建立本次连续位置参考.'),
    Profile('capture', '实车只读采集 / 1 Mbps', 'capture-stream-arm-Release', 'sentinel_capture',
        ('EC_BUILD_SENTINEL_CAPTURE',), 'check_capture.py', (),
        'robots/sentinel/host/sentinel_capture_station.py', ('--port',), 'UART1 / USART6 / 1000000',
        '不驱动. DJI只接收; DM只读查询. 对应CaptureStation, 同步电机与IMU记录.'),
    Profile('s5', 'S5 / GM6020波形调试', 's5-arm-Release', 'sentinel_s1',
        ('EC_BUILD_SENTINEL_S1', 'EC_S5_GM6020_BENCH'), 'check_firmware.py', ('--allow-gm6020-bench',),
        'boards/sentinel_f407/tools/motor_bench_console.py', ('--port',), 'UART1 / USART6 / 115200',
        '可驱动. 单GM6020 ID4, 电流/速度/位置与冻结波形. 不用于实车或四电机台架.'),
    Profile('vehicle', 'APP / 实车配置占位', 'app-arm-Release', 'sentinel_app',
        ('EC_BUILD_SENTINEL_APP',), 'check_sentinel_app.py', (),
        'robots/sentinel/host/sentinel_console.py', ('--mechanism', 'gimbal', '--serial'), 'UART2 / USART1 / 921600',
        '当前零执行槽占位, 尚非实车可执行配置. USART6另提供115200只读观测.'),
    Profile('s1', 'S1 / 基础只读观测', 's1-release', 'sentinel_s1',
        ('EC_BUILD_SENTINEL_S1',), 'check_firmware.py', (),
        'boards/sentinel_f407/tools/board_monitor.py', ('--port',), 'UART1 / USART6 / 115200',
        '无电机发送路径. 基础板卡与设备状态观测, 不等于连续采集固件.'),
)

def default_elf(root: Path, profile: Profile) -> Path:
    return root / 'build' / profile.build_dir / 'boards/sentinel_f407' / (profile.target + '.elf')

def sector_span(size: int) -> tuple[int, int]:
    if not 8 <= size <= FLASH_SIZE:
        raise ValueError('镜像为空、过短或超出1 MiB Flash')
    covered = 0
    for number, length in enumerate(SECTORS):
        covered += length
        if size <= covered:
            return number, covered
    raise ValueError('镜像越界')

@dataclass
class Settings:
    cmake: str = ''
    ninja: str = ''
    arm_root: str = ''
    sdk_root: str = ''
    openocd: str = ''
    scripts: str = ''
    python: str = ''
    pwsh: str = ''

    @classmethod
    def discover(cls, root: Path):
        cache = {}
        for profile in PROFILES:
            path = root / 'build' / profile.build_dir / 'CMakeCache.txt'
            if path.is_file():
                for line in path.read_text(encoding='utf-8').splitlines():
                    if ':' in line and '=' in line and not line.startswith(('#', '//')):
                        key, value = line.split('=', 1)
                        cache.setdefault(key.split(':')[0], value)
        python = root / '.venv/Scripts/python.exe'
        result = cls(shutil.which('cmake') or cache.get('CMAKE_COMMAND', ''),
            shutil.which('ninja') or cache.get('CMAKE_MAKE_PROGRAM', ''),
            cache.get('ARM_GNU_ROOT', ''), cache.get('EC_STM32CUBE_F4_ROOT', ''),
            shutil.which('openocd') or '', '', str(python) if python.is_file() else sys.executable,
            shutil.which('pwsh') or '')
        if result.openocd:
            base = Path(result.openocd).parent.parent
            for relative in ('openocd/scripts', 'share/openocd/scripts', 'scripts'):
                if (base / relative / 'target/stm32f4x.cfg').is_file():
                    result.scripts = str(base / relative)
                    break
        saved = root / 'build/firmware-tool/settings.json'
        if saved.is_file():
            for key, value in json.loads(saved.read_text(encoding='utf-8')).items():
                if key in cls.__dataclass_fields__ and isinstance(value, str):
                    setattr(result, key, value)
        return result

    def save(self, root: Path):
        from dataclasses import asdict
        path = root / 'build/firmware-tool/settings.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), ensure_ascii=False, indent=2), encoding='utf-8')

    def arm(self, name: str) -> str:
        return str(Path(self.arm_root) / 'bin' / ('arm-none-eabi-' + name + ('.exe' if os.name == 'nt' else '')))

def require_files(*paths):
    for value in paths:
        if not value or not Path(value).is_file():
            raise ValueError(f'文件不存在, 请检查工具路径: {value}')

def run_command(args, cwd: Path, log, timeout=180):
    # 参数数组直接传进程, 不经shell解释文件名或外部输入.
    log('> ' + subprocess.list2cmdline([str(a) for a in args]))
    env = dict(os.environ, PYTHONIOENCODING='utf-8')
    process = subprocess.Popen([str(a) for a in args], cwd=cwd, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace',
        creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    lines = []
    def consume():
        for line in process.stdout:
            lines.append(line)
            log(line.rstrip())
    reader = threading.Thread(target=consume, daemon=True)
    reader.start()
    try:
        code = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=10)
        raise RuntimeError(f'命令超时({timeout}s), 已停止本工具进程, 未自动重试') from None
    finally:
        reader.join(timeout=10)
        process.stdout.close()
    if code:
        raise RuntimeError(f'命令失败, 退出码 {code}; 请查看日志')
    return ''.join(lines)

def build_firmware(root: Path, profile: Profile, settings: Settings, log, run=run_command):
    require_files(settings.cmake, settings.ninja, settings.python, settings.arm('gcc'),
        root / 'cmake/arm-none-eabi.cmake',
        Path(settings.sdk_root) / 'Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h')
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    # 采集端对标签作协议兼容检查, 不能换成随机构建日期.
    label = 'cap-stream-1m-v1' if profile.key == 'capture' else stamp
    if profile.key == 'four':
        label += '-id3'
    elif profile.key == 'gm':
        label += '-manual'
    flags = ('EC_BUILD_SENTINEL_APP', 'EC_BUILD_SENTINEL_CAPTURE', 'EC_BUILD_SENTINEL_S1',
        'EC_APP_GM6020_BENCH', 'EC_APP_FOUR_MOTOR_BENCH', 'EC_APP_CHASSIS_COMMISSIONING', 'EC_S5_GM6020_BENCH')
    destination = root / 'build' / profile.build_dir
    args = [settings.cmake, '-S', str(root), '-B', str(destination), '-G', 'Ninja',
        '-DCMAKE_BUILD_TYPE=Release', f'-DCMAKE_TOOLCHAIN_FILE={root / "cmake/arm-none-eabi.cmake"}',
        f'-DCMAKE_MAKE_PROGRAM={settings.ninja}', f'-DARM_GNU_ROOT={settings.arm_root}',
        f'-DEC_STM32CUBE_F4_ROOT={settings.sdk_root}', f'-DPython3_EXECUTABLE={settings.python}',
        '-DEC_BUILD_TESTS=OFF', '-DEC_BUILD_EXAMPLES=OFF', '-DEC_S1_UART6_MONITOR=ON',
        '-DEC_MONITOR_REGISTRY=ON', '-DEC_S5_GM6020_ID=4', f'-DEC_FIRMWARE_ID={label}']
    args += [f'-D{key}=' + ('ON' if key in profile.flags else 'OFF') for key in flags]
    run(args, root, log, 180)
    run([settings.cmake, '--build', str(destination), '--target', profile.target, '--parallel', '6'], root, log, 600)
    return default_elf(root, profile)

@dataclass(frozen=True)
class Image:
    binary: Path
    profile_key: str
    label: str
    sha256: str
    size: int
    last_sector: int
    erase_bytes: int

def prepare_image(root: Path, profile: Profile, elf: Path, settings: Settings, work: Path, log, run=run_command):
    require_files(elf, settings.python, settings.arm('nm'), settings.arm('objcopy'))
    work.mkdir(parents=True, exist_ok=True)
    staged = work / 'image.elf'
    if elf.resolve() == staged.resolve():
        raise ValueError('请选择原始构建产物, 不要选择本次临时快照')
    shutil.copyfile(elf, staged)
    binary = staged.with_suffix('.bin')
    run([settings.arm('objcopy'), '-O', 'binary', str(staged), str(binary)], root, log, 30)
    data = binary.read_bytes()
    last_sector, erase_bytes = sector_span(len(data))
    msp, reset = struct.unpack_from('<II', data)
    if msp != 0x20020000 or not reset & 1 or not FLASH_BASE <= reset - 1 < FLASH_BASE + len(data):
        raise ValueError('向量表不符合当前STM32F407 1MiB镜像')
    audit = root / 'boards/sentinel_f407' / profile.audit
    result = run([settings.python, str(audit), '--elf', str(staged), '--nm', settings.arm('nm'), *profile.audit_flags], root, log, 60)
    if json.loads(result).get('result') != 'pass':
        raise ValueError('固件角色审计未通过')
    symbols = run([settings.arm('nm'), '-g', '--defined-only', str(staged)], root, lambda _: None, 30)
    address = next((int(parts[0], 16) for line in symbols.splitlines()
        if len(parts := line.split()) == 3 and parts[2] == 'g_ec_s1_build_id'), None)
    if address is None or not 0 <= address - FLASH_BASE < len(data):
        raise ValueError('未找到有效固件标识')
    start = address - FLASH_BASE
    end = data.find(bytes([0]), start, min(start + 256, len(data)))
    if end < 0:
        raise ValueError('固件标识未终止')
    label = data[start:end].decode('ascii')
    if profile.key == 'four' and not label.endswith('-id3'):
        raise ValueError('不是当前ID3四电机版本; 旧ID7镜像有0x1FF冲突, 请重新构建')
    if profile.key == 'gm' and not label.endswith('-manual'):
        raise ValueError('不是当前人工连续参考版本; 请重新构建单GM固件')
    if profile.key == 'capture' and label != 'capture-cap-stream-1m-v1':
        raise ValueError('采集站要求capture-cap-stream-1m-v1, 不能混用其它采集协议版本')
    image = Image(binary, profile.key, label, hashlib.sha256(data).hexdigest(), len(data), last_sector, erase_bytes)
    summary = dict(profile=profile.key, label=label, sha256=image.sha256, bytes=image.size,
        flash_base=FLASH_BASE, sectors=[0, last_sector], erase_end=FLASH_BASE + erase_bytes - 1, source_elf=str(elf))
    (work / 'image.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    log(f'已审计: {label}, {len(data)}字节, 仅擦写扇区0-{last_sector}')
    log(f'SHA256: {image.sha256}')
    return image

def attached_probes(settings: Settings, log, run=run_command):
    require_files(settings.pwsh)
    command = "$ErrorActionPreference='Stop'; @(Get-PnpDevice -PresentOnly | Where-Object {$_.InstanceId -like 'USB*VID_0483&PID_374*' -and $_.InstanceId -notlike '*&MI_*'} | ForEach-Object {$_.InstanceId}) | ConvertTo-Json -Compress"
    text = run([settings.pwsh, '-NoLogo', '-NoProfile', '-NonInteractive', '-Command', command], ROOT, lambda _: None, 30)
    found = json.loads(text) if text.strip() else []
    if isinstance(found, str):
        found = [found]
    if not isinstance(found, list) or any(not isinstance(item, str) for item in found):
        raise ValueError('无法确定已连接ST-Link数量')
    log(f'已连接ST-Link数量: {len(found)}')
    return found

def programming_script(last_sector: int) -> str:
    if not isinstance(last_sector, int) or not 0 <= last_sector < len(SECTORS):
        raise ValueError('无效扇区范围')
    # 文件名与目标地址固定, 不把用户路径拼进Tcl; 逐扇区擦除, 不走整片擦除优化.
    return '''proc checked_program {} {
    init
    reset halt
    set id [lindex [stm32f4x.cpu read_memory 0xe0042000 32 1] 0]
    if {($id & 0xfff) != 0x413} {error "Not an STM32F405/407 target"}
    set kib [lindex [stm32f4x.cpu read_memory 0x1fff7a22 16 1] 0]
    if {$kib != 1024} {error "Expected 1024 KiB Flash"}
    echo "FLASH_TOOL_TARGET_OK id=$id flash_kib=$kib"
    for {set sector 0} {$sector <= @LAST@} {incr sector} {
        flash erase_sector 0 $sector $sector
    }
    flash write_image image.bin 0x08000000 bin
    verify_image image.bin 0x08000000 bin
    echo "FLASH_TOOL_VERIFIED"
    reset run
    echo "FLASH_TOOL_SUCCESS"
}
if {[catch {checked_program} failure]} {
    echo "FLASH_TOOL_FAILED: $failure"
    shutdown error
} else {
    shutdown
}
'''.replace('@LAST@', str(last_sector))

def flash_image(image: Image, settings: Settings, log, run=run_command, probes=attached_probes):
    require_files(settings.openocd, Path(settings.scripts) / 'interface/stlink.cfg',
        Path(settings.scripts) / 'target/stm32f4x.cfg', image.binary)
    data = image.binary.read_bytes()
    if image.binary.name != 'image.bin' or len(data) != image.size or hashlib.sha256(data).hexdigest() != image.sha256:
        raise ValueError('审计后的镜像发生变化, 拒绝烧录')
    last_sector, _ = sector_span(image.size)
    if len(probes(settings, log, run)) != 1:
        raise ValueError('必须只连接一个ST-Link; 不按Windows序列号猜测目标')
    script = image.binary.parent / 'program.tcl'
    script.write_text(programming_script(last_sector), encoding='ascii')
    args = [settings.openocd, '-s', settings.scripts, '-f', 'interface/stlink.cfg',
        '-f', 'target/stm32f4x.cfg', '-c', 'gdb port disabled; tcl port disabled; telnet port disabled; adapter speed 4000',
        '-f', str(script)]
    log('开始烧录: 无Flash备份, 无整片擦除, 不改Option Bytes, 不下发电机目标')
    output = run(args, image.binary.parent, log, 180)
    if 'FLASH_TOOL_SUCCESS' not in output or 'FLASH_TOOL_FAILED' in output:
        raise RuntimeError('缺少烧录/校验/复位完成证据; 不报告成功, 不自动重试')
    log('烧录并校验通过, MCU已复位运行. 上位机需重新连接, GM台架需重新建立参考.')

def host_command(root: Path, profile: Profile, settings: Settings, port: str):
    if not port.strip():
        raise ValueError('请输入人工选择的串口, 例如COM3')
    require_files(settings.python, root / profile.host)
    return [settings.python, str(root / profile.host), *profile.host_args, port.strip()]

def new_run(root: Path) -> Path:
    base = root / 'build/firmware-tool/runs'
    base.mkdir(parents=True, exist_ok=True)
    return Path(tempfile.mkdtemp(prefix=datetime.now().strftime('%Y%m%d-%H%M%S-'), dir=base))

"""打包CAN1 ID4的S5波形固件、原始上位机与重建源码, 不访问硬件."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile
from firmware_tool_model import ROOT, PROFILES, Settings, default_elf, prepare_image
from package_capture_handoff import operator_html


def package(destination: Path):
    destination = destination.resolve()
    archive = destination.with_suffix('.zip')
    if destination.exists() or archive.exists():
        raise FileExistsError('输出已存在, 请换名称; 不覆盖旧交接包')
    profile = next(p for p in PROFILES if p.key == 's5')
    settings = Settings.discover(ROOT)
    elf = default_elf(ROOT, profile)
    build = ROOT / 'build' / profile.build_dir
    # 源配置默认值不能证明旧构建目录里的实际ID, 打包前核对本次缓存及编译命令.
    cache = dict(line.split('=', 1) for line in (build / 'CMakeCache.txt').read_text().splitlines()
        if '=' in line and not line.startswith(('#', '//')))
    if cache.get('EC_S5_GM6020_ID:STRING') != '4' or cache.get('EC_S5_GM6020_BENCH:BOOL') != 'ON':
        raise ValueError('须先以EC_S5_GM6020_BENCH=ON、EC_S5_GM6020_ID=4构建')
    if 'EC_S5_GM6020_ID=4' not in (build / 'build.ninja').read_text():
        raise ValueError('编译参数未包含ID4')
    audit = ROOT / 'build/s5-handoff-validation/audit'
    image = prepare_image(ROOT, profile, elf, settings, audit, print)
    if not image.label.startswith('s5-gm6020-id4-'):
        raise ValueError('请选择本次明确标识ID4的S5镜像, 不按同名文件猜角色')
    pins = [line for line in (ROOT / 'requirements-dev.txt').read_text(encoding='utf-8').splitlines()
        if line.startswith(('pyserial==', 'numpy==', 'matplotlib==', 'dearpygui=='))]
    if len(pins) != 4:
        raise ValueError('波形控制台需要串口、numpy、matplotlib与dearpygui固定版本')
    destination.mkdir(parents=True)
    # 沿用原始工程路径, 刷写工具可直接找到S5默认镜像. 不复制外部CAD/日志/虚拟环境.
    candidates = [ROOT / 'CMakeLists.txt', ROOT / 'requirements-dev.txt']
    for name in ('LICENSE', 'LICENSE.txt', 'NOTICE'):
        if (ROOT / name).is_file():
            candidates.append(ROOT / name)
    for directory in ('boards', 'cmake', 'include', 'src', 'platform', 'robots', 'examples', 'tests', 'docs'):
        candidates.extend(p for p in (ROOT / directory).rglob('*') if p.is_file()
            and '__pycache__' not in p.parts and p.suffix not in ('.pyc', '.pyo'))
    source_files = []
    for original in sorted(candidates):
        relative = original.relative_to(ROOT)
        target = destination / 'source' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original, target)
        source_files.append(relative.as_posix())
    packaged_elf = destination / 'source' / elf.relative_to(ROOT)
    packaged_elf.parent.mkdir(parents=True, exist_ok=True)
    for extension in ('.elf', '.bin', '.hex'):
        shutil.copyfile(elf.with_suffix(extension), packaged_elf.with_suffix(extension))
    (destination / 'records/s5').mkdir(parents=True)
    (destination / 'requirements.txt').write_text('\n'.join(pins) + '\n', encoding='ascii')
    document = (ROOT / 'docs/guides/S5波形调试交接.md').read_text(encoding='utf-8')
    (destination / '操作说明.md').write_text(document, encoding='utf-8')
    html = operator_html(document).replace('<title>实车采集操作说明</title>', '<title>S5 ID4波形调试交接</title>')
    (destination / '先读我.html').write_text(html, encoding='utf-8')
    shutil.copyfile(audit / 'image.json', destination / 'IMAGE.json')
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    metadata = dict(profile='s5', role='ACTUATING isolated single GM6020 bench',
        source_base_commit=revision, source_snapshot='working tree; see per-file SHA256SUMS.txt',
        firmware_label=image.label, firmware_sha256=image.sha256, firmware_bytes=image.size,
        firmware=packaged_elf.relative_to(destination).as_posix(), flash_sectors=[0, image.last_sector],
        bus=1, device_id=4, feedback_id=0x208, command_id=0x1fe, command_slot_zero_based=3,
        serial=dict(silkscreen='UART1', peripheral='USART6', baud=115200), protocol='S5/v6',
        console='source/boards/sentinel_f407/tools/motor_bench_console.py',
        source_files=source_files, requirements=pins, hardware_access_during_packaging=False,
        physical_acceptance=False, firmware_default_id_changes_ESC_saved_ID=False)
    (destination / 'PACKAGE.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    manifest = [hashlib.sha256(p.read_bytes()).hexdigest() + '  ' + p.relative_to(destination).as_posix()
        for p in sorted(destination.rglob('*')) if p.is_file()]
    (destination / 'SHA256SUMS.txt').write_text('\n'.join(manifest) + '\n', encoding='utf-8')
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in sorted(destination.rglob('*')):
            if path.is_file():
                bundle.write(path, destination.name + '/' + path.relative_to(destination).as_posix())
    print(json.dumps(dict(archive=str(archive), bytes=archive.stat().st_size,
        sha256=hashlib.sha256(archive.read_bytes()).hexdigest()), ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    package(parser.parse_args().output)


if __name__ == '__main__':
    main()

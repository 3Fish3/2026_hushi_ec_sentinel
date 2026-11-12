"""打包实车机构APP、原始Python/重建源码与操作卡; 默认底盘, 不访问串口或SWD."""
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


def package(destination: Path, diagram: Path, role: str = 'chassis'):
    destination = destination.resolve()
    archive = destination.with_suffix('.zip')
    if destination.exists() or archive.exists():
        raise FileExistsError('目标已存在, 请换输出名称; 不覆盖旧交接包')
    if role == 'chassis' and not diagram.is_file():
        raise FileNotFoundError(diagram)
    profile = next(p for p in PROFILES if p.key == role)
    guide, attachments = {
        'chassis': ('底盘实车调试交接.md', ('底盘参数表.csv', '底盘CAD基线.json')),
        'gimbal': ('云台实车调试交接.md', ('云台参数表.csv',)),
        'shooter': ('发射机构实车调试交接.md', ('发射参数表.csv',)),
    }[role]
    settings = Settings.discover(ROOT)
    elf = default_elf(ROOT, profile)
    audit = ROOT / f'build/{role}-handoff-validation/app-audit'
    image = prepare_image(ROOT, profile, elf, settings, audit, print)
    destination.mkdir(parents=True)
    # 只复制可重建工程与原始脚本, 不携带原始记录、私有运行状态、工具链或虚拟环境.
    source_files = []
    directories = ('boards', 'cmake', 'include', 'src', 'platform', 'robots', 'examples', 'tests', 'docs')
    candidates = [ROOT / 'CMakeLists.txt', ROOT / 'requirements-dev.txt']
    for name in ('LICENSE', 'LICENSE.txt', 'NOTICE'):
        if (ROOT / name).is_file():
            candidates.append(ROOT / name)
    for name in directories:
        candidates.extend(p for p in (ROOT / name).rglob('*') if p.is_file()
            and '__pycache__' not in p.parts and p.suffix not in ('.pyc', '.pyo'))
    for original in sorted(candidates):
        relative = original.relative_to(ROOT)
        target = destination / 'source' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original, target)
        source_files.append(relative.as_posix())
    # 放在刷写工具的角色默认路径, 不修改原始脚本的仓库定位方式.
    packaged_elf = destination / 'source' / elf.relative_to(ROOT)
    packaged_elf.parent.mkdir(parents=True, exist_ok=True)
    for ext in ('.elf', '.bin', '.hex'):
        shutil.copyfile(elf.with_suffix(ext), packaged_elf.with_suffix(ext))
    (destination / 'records').mkdir()
    pins = [line for line in (ROOT / 'requirements-dev.txt').read_text(encoding='utf-8').splitlines()
        if line.startswith(('pyserial==', 'dearpygui=='))]
    if len(pins) != 2:
        raise ValueError('未找到串口和GUI的固定版本依赖')
    (destination / 'requirements.txt').write_text('\n'.join(pins) + '\n', encoding='ascii')
    document = (ROOT / 'docs/guides' / guide).read_text(encoding='utf-8')
    (destination / '操作说明.md').write_text(document, encoding='utf-8')
    html = operator_html(document).replace('<title>实车采集操作说明</title>', f'<title>{profile.title}交接</title>')
    links = ' | '.join(f'<a href="{name}">{name}</a>' for name in attachments)
    if role == 'chassis':
        links += ' | <a href="底盘轮位与ID.png">轮位与ID图</a>'
    html = html.replace('</body>', f'<p>{links}</p></body>')
    (destination / '先读我.html').write_text(html, encoding='utf-8')
    for name in attachments:
        shutil.copyfile(ROOT / 'docs/guides' / name, destination / name)
    if role == 'chassis':
        shutil.copyfile(diagram, destination / '底盘轮位与ID.png')
    # 可独立核对方向和编译参数; GUI只热改已支持的PID/电流参数.
    shutil.copyfile(audit / 'image.json', destination / 'IMAGE.json')
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    metadata = dict(scope='vehicle-'+role+'-commissioning', profile=role, source_base_commit=revision,
        source_snapshot='working tree files; per-file hashes in SHA256SUMS.txt, not claimed committed',
        firmware_label=image.label, firmware_sha256=image.sha256, firmware_bytes=image.size,
        flash_sectors=[0, image.last_sector], firmware=packaged_elf.relative_to(destination).as_posix(),
        actuating_vehicle_firmware_included=True, motor_ratio=19.0 if role == 'chassis' else None,
        feeder_motor_ratio=36.0 if role == 'shooter' else None, source_files=source_files,
        requirements=pins, hardware_access_during_packaging=False, physical_acceptance=False)
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
    parser.add_argument('--diagram', type=Path, default=ROOT / 'build/chassis-cad-20260926/chassis-body-ids.png')
    parser.add_argument('--profile', choices=('chassis', 'gimbal', 'shooter'), default='chassis')
    args = parser.parse_args()
    package(args.output, args.diagram, args.profile)


if __name__ == '__main__':
    main()

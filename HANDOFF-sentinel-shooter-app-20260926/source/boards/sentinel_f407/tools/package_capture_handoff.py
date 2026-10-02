"""组装已构建并验证的实车采集交接包; 只创建新目录, 不覆盖旧包或原始数据."""
import argparse
import hashlib
import html
import json
import importlib.metadata
import sys
from pathlib import Path
import re
import shutil
import subprocess
import zipfile

ROOT=Path(__file__).resolve().parents[3]


def operator_html(text):
    def inline(line):
        line=html.escape(line)
        line=re.sub(r'\*\*(.+?)\*\*',r'<strong>\1</strong>',line)
        return re.sub(r'`(.+?)`',r'<code>\1</code>',line)
    body=[];table=False;first=False
    for line in text.splitlines():
        if line.startswith('|'):
            cells=[x.strip() for x in line.strip('|').split('|')]
            if all(not x.strip('-: ') for x in cells):continue
            if not table:body.append('<table>');table=True;first=True
            tag='th' if first else 'td';first=False
            body.append('<tr>'+''.join(f'<{tag}>{inline(x)}</{tag}>' for x in cells)+'</tr>');continue
        if table:body.append('</table>');table=False
        if line.startswith('#'):
            level=len(line)-len(line.lstrip('#'));body.append(f'<h{level}>{inline(line[level:].strip())}</h{level}>')
        elif line:body.append('<p>'+inline('• '+line[2:] if line.startswith('- ') else line)+'</p>')
    if table:body.append('</table>')
    return '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>实车采集操作说明</title><style>body{max-width:1020px;margin:32px auto;padding:0 24px;font:17px/1.7 "Microsoft YaHei",sans-serif;color:#202a36}h1,h2{color:#14547d}h2{border-bottom:1px solid #ccd4dc;margin-top:32px}p{margin:8px 0}table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccd4dc;padding:6px 10px;text-align:left}code{background:#edf1f5;padding:2px 5px}strong{color:#8b3a10}@media print{body{font-size:11pt;margin:0}}</style>'+''.join(body)+'</html>'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware-dir',type=Path,required=True)
    parser.add_argument('--app-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();dest=args.output.resolve();archive=dest.with_suffix('.zip')
    if dest.exists() or archive.exists():raise FileExistsError('目标交接包已存在, 请使用新目录名')
    firmware=args.firmware_dir.resolve();app=args.app_dir.resolve()
    if dest.is_relative_to(app):raise ValueError('输出目录不能位于程序输入目录内')
    for name in ('sentinel_capture.hex','sentinel_capture.bin','sentinel_capture.elf'):
        if not (firmware/name).is_file():raise FileNotFoundError(firmware/name)
    if not (app/'CaptureStation.exe').is_file():raise FileNotFoundError(app/'CaptureStation.exe')
    for entry in app.rglob('*'):
        if not entry.resolve().is_relative_to(app):raise ValueError('程序目录包含越界链接')
    dest.mkdir(parents=True);(dest/'firmware').mkdir();(dest/'records').mkdir()
    for name in ('sentinel_capture.hex','sentinel_capture.bin','sentinel_capture.elf'):
        shutil.copy2(firmware/name,dest/'firmware'/name)
    shutil.copytree(app,dest/'app')
    (dest/'licenses').mkdir()
    shutil.copy2(Path(sys.base_prefix)/'LICENSE.txt',dest/'licenses/Python-LICENSE.txt')
    distribution=importlib.metadata.distribution('pyinstaller')
    copying=next(p for p in distribution.files if str(p).endswith('licenses/COPYING.txt'))
    shutil.copy2(distribution.locate_file(copying),dest/'licenses/PyInstaller-COPYING.txt')
    shutil.copy2(ROOT/'boards/sentinel_f407/tools/licenses/pyserial-LICENSE.txt',dest/'licenses/pyserial-LICENSE.txt')
    document=(ROOT/'docs/guides/实车只读采集交接.md').read_text(encoding='utf-8')
    (dest/'先读我.html').write_text(operator_html(document),encoding='utf-8')
    (dest/'操作说明.txt').write_text(document,encoding='utf-8-sig')
    (dest/'启动采集.cmd').write_bytes(b'@echo off\r\ncd /d "%~dp0"\r\nstart "" "%~dp0app\\CaptureStation.exe"\r\n')
    # 源码作为可选备用入口, 保持导入目录布局; 操作者正常只使用exe.
    files=['robots/sentinel/host/'+n for n in ('sentinel_capture_station.py','sentinel_stream.py','sentinel_raw.py')]
    files+=['boards/sentinel_f407/tools/'+n for n in ('board_monitor.py','sentinel_capture.py')]
    for name in files:
        target=dest/'source'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,target)
    (dest/'source/requirements.txt').write_text('pyserial==3.5\n',encoding='ascii')
    revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    dirty=bool(subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True))
    identity={'role':'capture-dm-read-only','build_id':'capture-cap-stream-1m-v1','uart_baud':1000000,
              'platform':'Windows x64','source_base_commit':revision,'source_worktree_dirty':dirty,
              'bench_scope':'C-board BMI088 + one GM6020 CAN1 ID4 + existing USB serial; not full vehicle',
              'rotation_rate_limits_hz':{'gyro':250,'accel':100,'each_gimbal_motor':250},
              'firmware_sha256':hashlib.sha256((dest/'firmware/sentinel_capture.bin').read_bytes()).hexdigest()}
    (dest/'PACKAGE.json').write_text(json.dumps(identity,ensure_ascii=False,indent=2),encoding='utf-8')
    entries=[]
    for file in sorted(dest.rglob('*')):
        if file.is_file():entries.append(hashlib.sha256(file.read_bytes()).hexdigest()+'  '+file.relative_to(dest).as_posix())
    (dest/'SHA256SUMS.txt').write_text('\n'.join(entries)+'\n',encoding='utf-8')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for file in sorted(dest.rglob('*')):
            if file.is_file():z.write(file,dest.name+'/'+file.relative_to(dest).as_posix())
    print(json.dumps({'directory':str(dest),'archive':str(archive),'bytes':archive.stat().st_size,
                      'sha256':hashlib.sha256(archive.read_bytes()).hexdigest()},ensure_ascii=False))


if __name__=='__main__':main()

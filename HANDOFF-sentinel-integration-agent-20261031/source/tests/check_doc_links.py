"""文档本地链接检查: 所有被跟踪 Markdown 里的相对 .md 链接都必须指向被跟踪文件.

仓库根由脚本自身位置推导, 不依赖当前工作目录: 契约测试必须能在仓库外的构建目录里运行.
"""
import re
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parent.parent
raw = subprocess.run(['git', '-c', 'core.quotepath=false', '-C', str(root), 'ls-files', '-z', '*.md'],
                     capture_output=True, check=True).stdout
files = [item.decode('utf-8') for item in raw.split(b'\0') if item]
tracked = {name.replace('\\', '/') for name in files}
bad = []
checked = 0
for name in files:
    text = (root / name).read_text(encoding='utf-8')
    for match in re.finditer(r'\[[^\]]*\]\(([^)#\s]+\.md)(#[^)]*)?\)', text):
        target = match.group(1)
        if target.startswith(('http://', 'https://')):
            continue
        checked += 1
        resolved = (root / name).parent / target
        try:
            rel = resolved.resolve().relative_to(root).as_posix()
        except ValueError:
            bad.append(f'{name}: {target} (outside repo)')
            continue
        if rel not in tracked:
            bad.append(f'{name}: {target}')
print(f'checked {len(files)} md files, {checked} local links; broken: {len(bad)}')
for item in bad[:15]:
    print('  ', item)
# 契约测试靠退出码判定: 发现断链必须失败, 否则会变成“打印了断链但测试通过”的假绿.
raise SystemExit(1 if bad else 0)

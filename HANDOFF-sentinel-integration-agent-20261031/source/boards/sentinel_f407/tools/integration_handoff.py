"""底盘云台跨机器交接: 源码/双固件封包、校验和按原ZIP打包回传; 仅使用标准库."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import difflib
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[3]
TREES = ('boards', 'cmake', 'include', 'src', 'platform', 'robots', 'examples', 'tests', 'docs')
ROOT_FILES = ('CMakeLists.txt', 'README.md', '.gitignore', 'requirements-dev.txt')
SKIP_DIRS = {'.git', '.venv', 'venv', 'env', '__pycache__', '.pytest_cache', '.mypy_cache',
    '.cache', '.vs', '.vscode', '.obsidian', 'node_modules', 'site-packages', 'cmakefiles',
    '.ruff_cache'}
SKIP_SUFFIXES = {'.pyc', '.pyo', '.exe', '.dll', '.pyd', '.o', '.obj', '.a', '.lib', '.pdb',
    '.elf', '.bin', '.hex', '.map', '.whl', '.zip', '.7z', '.sraw', '.btrace'}
IMAGE_PATHS = {
    'integration': 'build/app-integration-arm-Release/boards/sentinel_f407/sentinel_app.elf',
    'capture_current': 'build/capture-current-arm-Release/boards/sentinel_f407/sentinel_capture.elf',
}
TASK = 'docs/guides/Agent-底盘云台联调.md'


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as file:
        for block in iter(lambda: file.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + '\n').encode('utf-8')


def safe_name(name):
    path = PurePosixPath(name)
    if not name or '\\' in name or path.is_absolute() or '..' in path.parts or any(':' in p for p in path.parts) or path.as_posix() != name:
        raise ValueError(f'非法包内路径: {name!r}')
    return name


def linked(path):
    return path.is_symlink() or getattr(path, 'is_junction', lambda: False)()


def source_allowed(relative):
    parts = PurePosixPath(relative).parts
    name = parts[-1]
    if any(p.lower() in SKIP_DIRS | {'build', 'dist'} for p in parts[:-1]) or parts[0] == 'records':
        return False
    if name == 'AGENTS.md' or name == 'Plan.local.md' or name == '.env' or name.startswith('.env.'):
        return False
    if Path(name).suffix.lower() in SKIP_SUFFIXES or name.endswith('.wire.jsonl'):
        return False
    if parts[:4] == ('docs', 'boards', 'sentinel_f407', 'references'):
        return False
    if parts[0] == 'references':
        return len(parts) == 4 and parts[1:3] == ('records', '10310221') and name.endswith('.csv')
    return True


def walk_files(root, source_only=False):
    root = Path(root).resolve()
    for parent, dirs, files in os.walk(root, followlinks=False):
        base = Path(parent)
        def allowed_dir(name):
            if name.lower() in SKIP_DIRS or linked(base / name):
                return False
            relative = (base / name).relative_to(root).parts
            if source_only and (name.lower() in ('build', 'dist') or relative[0] == 'records'):
                return False
            if source_only and relative[0] == 'references':
                selected = ('references', 'records', '10310221')
                return relative[:len(selected)] == selected[:len(relative)]
            return True
        dirs[:] = sorted(d for d in dirs if allowed_dir(d))
        for name in sorted(files):
            file = base / name
            if linked(file) or not file.resolve().is_relative_to(root):
                raise ValueError(f'不复制链接或根目录外文件: {file}')
            if file.is_file():
                yield file.relative_to(root).as_posix(), file


def source_inventory(root):
    return {name: {'sha256': digest(path), 'bytes': path.stat().st_size}
        for name, path in walk_files(root, source_only=True) if source_allowed(name)}


def tree_digest(files):
    rows = ''.join(f'{files[name]["sha256"]}  {name}\n' for name in sorted(files))
    return hashlib.sha256(rows.encode('utf-8')).hexdigest()


def verify(archive):
    """校验全部ZIP成员, 不解压或执行包内代码."""
    with zipfile.ZipFile(archive) as bundle:
        members = bundle.infolist()
        names = [item.filename for item in members]
        if len(names) != len(set(names)) or not names:
            raise ValueError('ZIP为空或有重复成员')
        roots = set()
        for item in members:
            safe_name(item.filename)
            if item.is_dir() or stat.S_ISLNK(item.external_attr >> 16):
                raise ValueError('只接受普通文件成员')
            roots.add(item.filename.split('/')[0])
        if len(roots) != 1:
            raise ValueError('ZIP须有唯一顶层目录')
        prefix = next(iter(roots)) + '/'
        manifest_name = prefix + 'SHA256SUMS.txt'
        expected = {}
        for line in bundle.read(manifest_name).decode('utf-8').splitlines():
            sha, name = line.split('  ', 1)
            safe_name(name)
            if len(sha) != 64 or any(c not in '0123456789abcdef' for c in sha) or name in expected:
                raise ValueError('SHA256清单格式错误')
            expected[name] = sha
        if set(names) != {prefix + name for name in expected} | {manifest_name}:
            raise ValueError('ZIP成员和校验清单不一致')
        for name, sha in expected.items():
            value = hashlib.sha256()
            with bundle.open(prefix + name) as file:
                for block in iter(lambda: file.read(1024 * 1024), b''):
                    value.update(block)
            if value.hexdigest() != sha:
                raise ValueError(f'校验失败: {name}')
        metadata = 'PACKAGE.json' if prefix + 'PACKAGE.json' in names else 'RETURN.json'
        return prefix, json.loads(bundle.read(prefix + metadata))


def write_archive(archive, prefix, entries):
    archive = Path(archive).resolve()
    checksum = archive.with_name(archive.name + '.sha256')
    if archive.exists() or checksum.exists():
        raise FileExistsError(archive)
    archive.parent.mkdir(parents=True, exist_ok=True)
    manifest = []
    for name, value in sorted(entries.items()):
        safe_name(name)
        sha = digest(value) if isinstance(value, Path) else hashlib.sha256(value).hexdigest()
        manifest.append(f'{sha}  {name}\n')
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED, compresslevel=6) as bundle:
        for name, value in sorted(entries.items()):
            if isinstance(value, Path):
                bundle.write(value, prefix + '/' + name)
            else:
                bundle.writestr(prefix + '/' + name, value)
        bundle.writestr(prefix + '/SHA256SUMS.txt', ''.join(manifest).encode('utf-8'))
    verify(archive)
    sha = digest(archive)
    with checksum.open('x', encoding='utf-8', newline='\n') as file:
        file.write(sha + '  ' + archive.name + '\n')
    print(json.dumps({'archive': str(archive), 'bytes': archive.stat().st_size,
        'sha256': sha}, ensure_ascii=False))
    return archive


def pack(destination, evidence, task=None):
    destination = Path(destination).resolve()
    archive = destination.with_name(destination.name + '.zip')
    if destination.exists() or archive.exists():
        raise FileExistsError('输出已存在, 请换名称; 不覆盖旧交接包')
    task = Path(task) if task is not None else ROOT / TASK
    if not task.is_file():
        raise FileNotFoundError(f'一次性Agent任务文档不随Git提供, 请用--task指定: {task}')
    evidence = Path(evidence).resolve()
    images = {}
    for key, relative in IMAGE_PATHS.items():
        meta = json.loads((evidence / key / 'image.json').read_text(encoding='utf-8'))
        elf = ROOT / relative
        binary = elf.with_suffix('.bin')
        if digest(binary) != meta['sha256'] or binary.stat().st_size != meta['bytes']:
            raise ValueError(f'现成镜像与已验证元数据不符: {key}')
        if not meta['label'].startswith('app-integration-' if key == 'integration' else 'capture-current-'):
            raise ValueError(f'镜像角色错误: {key}')
        for suffix in ('.elf', '.bin', '.hex', '.map'):
            if not elf.with_suffix(suffix).is_file():
                raise FileNotFoundError(elf.with_suffix(suffix))
        images[key] = {k: meta[k] for k in ('label', 'sha256', 'bytes', 'sectors')}
        images[key]['elf'] = 'source/' + relative
    destination.mkdir(parents=True)
    source = destination / 'source'
    candidates = [(name, ROOT / name) for name in ROOT_FILES if (ROOT / name).is_file()]
    for tree in TREES:
        candidates += [(tree + '/' + name, path) for name, path in walk_files(ROOT / tree, source_only=True)]
    candidates += [(p.relative_to(ROOT).as_posix(), p) for p in (ROOT / 'references/records/10310221').glob('*.csv')]
    for name, original in sorted(candidates):
        if not source_allowed(name):
            continue
        if linked(original) or not original.resolve().is_relative_to(ROOT):
            raise ValueError(f'源码超出范围: {original}')
        target = source / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original, target)
    # 外部任务单统一放在原包约定路径, 校验和回传仍以实际打包内容为基线.
    task_target = source / TASK
    task_target.parent.mkdir(parents=True, exist_ok=True)
    task_target.write_bytes(task.read_bytes())
    files = source_inventory(source)
    for key, relative in IMAGE_PATHS.items():
        elf = ROOT / relative
        for suffix in ('.elf', '.bin', '.hex', '.map'):
            target = source / Path(relative).with_suffix(suffix)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(elf.with_suffix(suffix), target)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    identity = 'sentinel-integration-' + stamp + '-' + tree_digest(files)[:12]
    base = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip()
    baseline = {'schema': 1, 'package_id': identity, 'tree_sha256': tree_digest(files), 'files': files}
    (destination / 'SOURCE_BASELINE.json').write_bytes(encoded(baseline))
    metadata = {'schema': 'sentinel-integration-handoff/1', 'package_id': identity,
        'created_utc': stamp, 'source_base_commit': base, 'source_snapshot': 'working tree, including uncommitted/new source files',
        'source_tree_sha256': baseline['tree_sha256'], 'source_baseline': 'SOURCE_BASELINE.json',
        'task_document': 'source/' + TASK, 'firmware': images, 'dependencies_bundled': False,
        'agent_policy_files_bundled': False, 'hardware_access_during_packaging': False}
    (destination / 'PACKAGE.json').write_bytes(encoded(metadata))
    logs = ('host-tests.log', 'final-targeted-tests.log', 'site-contract-test.log', 'gui-smoke.log',
        'arm-build.log', 'delivery-verification.log', 'build-host.log', 'build-site-contract.log')
    for name in logs:
        target = destination / 'evidence' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(evidence / name, target)
    for key, meta in images.items():
        (destination / 'evidence' / (key + '-image.json')).write_bytes(encoded(meta))
    readme = '''# 底盘＋云台跨机器Agent交接

先读 `source/docs/guides/Agent-底盘云台联调.md`, 工作根目录为 `source/`.
两份固件已放在 `source/build/`对应角色目录, 含ELF/BIN/HEX/MAP; 不含SDK、工具链、环境或依赖安装包.
`PACKAGE.json`给出本包身份, `SOURCE_BASELINE.json`给出精确源码基线. 本机Agent规则未携带, 原工程README中本地参考不属于交付内容.
输入比例和重装IMU几何尚需现场填写, 任务文档包含任务步骤和要回传的配置/证据/优化项.

保留收到的原ZIP. 从本目录校验和回传(仅Python标准库, 不需要Git):

```text
python source/boards/sentinel_f407/tools/integration_handoff.py verify <收到的原ZIP>
python source/boards/sentinel_f407/tools/integration_handoff.py return --baseline <收到的原ZIP> --source source --results results --output RETURN-sentinel-integration-<日期>.zip
```

一轮可准备多项互不依赖的短对照, 共用构建/采集准备, 每项独立结论与回报. 某项受阻不拖住其它独立项目.
有结论或问题即先发短摘要和已有证据, 根因未明/采集失败也不等整轮调参完成.
人工只做必要短操作; 同类失败没有新判别证据时先回卡点, 不继续多轮换参数试错. 工作时段结束仍无结论也给一条简短状态.
完整要求见 `source/docs/guides/固件刷写与上位机对应.md` §5. 首报不必等ZIP、视频或完整验收材料.

正式分批包可将本批报告和结果放在 `results/B01/`:

```text
python source/boards/sentinel_f407/tools/integration_handoff.py return --baseline <收到的原ZIP> --source source --results results/B01 --output RETURN-sentinel-integration-B01.zip
```

`REPORT.md`开头写本批结论、实际镜像/参数、证据时段及卡点; 已发材料引用原批次/文件/SHA256, 不重复上传.
源码改动放在 `source/`, 已实现修改与仅建议分开写明. 每批源码差异都对比同一个原ZIP, 不串联应用多批patch.
回传自动包含新增/修改/删除清单、完整修改文件、文本patch和指定结果目录; 无需重传未改源码与环境.
'''
    (destination / 'README.md').write_text(readme, encoding='utf-8')
    # 外发目录已经筛选过, 这里连同显式加入source/build的两份固件一起封包.
    entries = {name: path for name, path in walk_files(destination)}
    return write_archive(archive, destination.name, entries)


def text_patch(name, before, after, existed, exists):
    try:
        if b'\0' in before or b'\0' in after:
            return b''
        old, new = before.decode('utf-8'), after.decode('utf-8')
    except UnicodeDecodeError:
        return b''
    quoted = lambda path: json.dumps(path, ensure_ascii=False)
    lines = list(difflib.unified_diff(old.splitlines(keepends=True), new.splitlines(keepends=True),
        quoted('a/' + name) if existed else '/dev/null',
        quoted('b/' + name) if exists else '/dev/null'))
    if not lines:
        return b''
    header = f'diff --git {quoted("a/" + name)} {quoted("b/" + name)}\n'
    if not existed:
        header += 'new file mode 100644\n'
    elif not exists:
        header += 'deleted file mode 100644\n'
    return (header + ''.join(line if line.endswith('\n') else line + '\n\\ No newline at end of file\n'
        for line in lines)).encode('utf-8')


def make_return(baseline_archive, source, results, output):
    prefix, package = verify(baseline_archive)
    if package.get('schema') != 'sentinel-integration-handoff/1':
        raise ValueError('基线必须是原始外发ZIP')
    source, results = Path(source).resolve(), Path(results).resolve()
    if not source.is_dir() or not (results / 'REPORT.md').is_file():
        raise ValueError('需要source目录和results/REPORT.md')
    current = source_inventory(source)
    entries = {}
    changes, patches, removed = [], [], []
    with zipfile.ZipFile(baseline_archive) as bundle:
        baseline = json.loads(bundle.read(prefix + package['source_baseline']))
        old = baseline['files']
        if baseline['package_id'] != package['package_id'] or tree_digest(old) != package['source_tree_sha256']:
            raise ValueError('源码基线身份错误')
        for name in sorted(set(old) | set(current)):
            before, after = old.get(name), current.get(name)
            if before == after:
                continue
            status = 'added' if before is None else 'deleted' if after is None else 'modified'
            content = bundle.read(prefix + 'source/' + safe_name(name)) if before else b''
            if before and hashlib.sha256(content).hexdigest() != before['sha256']:
                raise ValueError(f'原ZIP源码与基线不符: {name}')
            new_content = (source / name).read_bytes() if after else b''
            patch = text_patch(name, content, new_content, before is not None, after is not None)
            changes.append({'path': name, 'status': status, 'before': before, 'after': after,
                'in_text_patch': bool(patch)})
            patches.append(patch)
            if after:
                entries['changes/files/' + name] = source / name
            else:
                removed.append(name)
    entries['changes/source.patch'] = b''.join(patches)
    entries['changes/DELETED.txt'] = ''.join(name + '\n' for name in removed).encode('utf-8')
    for name, path in walk_files(results):
        if path.resolve() in (Path(output).resolve(), Path(baseline_archive).resolve()):
            continue
        entries['results/' + name] = path
    metadata = {'schema': 'sentinel-integration-return/1', 'package_id': package['package_id'],
        'baseline_archive_sha256': digest(baseline_archive), 'baseline_source_tree_sha256': baseline['tree_sha256'],
        'returned_source_tree_sha256': tree_digest(current), 'changes': changes,
        'report': 'results/REPORT.md', 'results_files': sorted(n for n in entries if n.startswith('results/'))}
    entries['RETURN.json'] = encoded(metadata)
    return write_archive(output, Path(output).stem, entries)


def apply_increment(archive, source, execute=False):
    """按规范化哈希预检全部增量, 再原位替换; 不依赖Git索引或patch的行尾匹配."""
    prefix,_=verify(archive)
    source=Path(source).resolve(strict=True)
    changes=[];seen=set()
    canonical=lambda b:b.replace(b'\r\n',b'\n')
    sha=lambda b:hashlib.sha256(canonical(b)).hexdigest()
    with zipfile.ZipFile(archive) as bundle:
        for entry in json.loads(bundle.read(prefix+'SOURCE_CHANGES.json')):
            name=safe_name(entry['path']);folded=name.casefold()
            if folded in seen or not source_allowed(name) or not (name in ROOT_FILES or name.split('/')[0] in TREES):
                raise ValueError(f'不允许的增量路径: {name}')
            seen.add(folded);target=source/name
            if not target.resolve().is_relative_to(source) or any(linked(p) for p in (target,*target.parents) if p!=source):
                raise ValueError(f'目标不能经链接离开源码目录: {name}')
            after=bundle.read(prefix+'files/'+name)
            if sha(after)!=entry['after_sha256']: raise ValueError(f'参考文件哈希不符: {name}')
            before=target.read_bytes() if target.exists() else None
            actual=sha(before) if before is not None else None
            if actual==entry['after_sha256']: continue
            if actual!=entry['field_before_sha256']:
                raise ValueError(f'现场基线已改动, 未写入任何文件: {name}, actual={actual}')
            data=canonical(after)
            if before and b'\r\n' in before and before.count(b'\r\n')==before.count(b'\n'):
                data=data.replace(b'\n',b'\r\n')
            changes.append((name,target,data))
    # 到此才写入; 任一基线冲突时整个预检不改文件. 已成功写入的项可在重跑时识别为已应用.
    if execute:
        for name,target,data in changes:
            target.parent.mkdir(parents=True,exist_ok=True)
            with tempfile.NamedTemporaryFile(dir=target.parent,delete=False) as temporary:
                temporary.write(data);temporary_path=Path(temporary.name)
            try: os.replace(temporary_path,target)
            finally:
                if temporary_path.exists(): temporary_path.unlink()
            if sha(target.read_bytes())!=sha(data): raise OSError(f'写后哈希不符: {name}')
    return dict(result='applied' if execute else 'checked',changed=[n for n,_,_ in changes],source=str(source))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    outbound = sub.add_parser('pack', help='从本机工作树及已验证镜像生成外发包')
    outbound.add_argument('--output', type=Path, required=True)
    outbound.add_argument('--task', type=Path, help='本次Agent任务文档; 不随Git提供')
    outbound.add_argument('--evidence', type=Path, default=ROOT / 'build/integration-validation')
    check = sub.add_parser('verify', help='验证外发或回传ZIP, 不解压/运行内容')
    check.add_argument('archive', type=Path)
    inbound = sub.add_parser('return', help='相对原ZIP收集全部源码差异、结果和建议')
    inbound.add_argument('--baseline', type=Path, required=True)
    inbound.add_argument('--source', type=Path, required=True)
    inbound.add_argument('--results', type=Path, required=True)
    inbound.add_argument('--output', type=Path, required=True)
    apply=sub.add_parser('apply',help='按SOURCE_CHANGES哈希预检增量; --execute才写入, 不依赖Git')
    apply.add_argument('archive',type=Path);apply.add_argument('--source',type=Path,required=True)
    apply.add_argument('--execute',action='store_true')
    args = parser.parse_args()
    if args.command == 'apply':
        print(json.dumps(apply_increment(args.archive,args.source,args.execute),ensure_ascii=False))
    elif args.command == 'pack':
        pack(args.output, args.evidence, args.task)
    elif args.command == 'return':
        make_return(args.baseline, args.source, args.results, args.output)
    else:
        prefix, metadata = verify(args.archive)
        print(json.dumps({'result': 'pass', 'root': prefix, 'package_id': metadata.get('package_id',metadata.get('schema')),
            'archive_sha256': digest(args.archive)}, ensure_ascii=False))


if __name__ == '__main__':
    main()

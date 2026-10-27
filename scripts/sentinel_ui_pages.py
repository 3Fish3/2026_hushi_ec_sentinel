"""Native DPG forms for capture, offline reports and existing-build firmware tools."""
import os
import json
import math
import subprocess
from pathlib import Path
import queue
import shlex
import shutil
import time

from sentinel_ui_resources import ProcessJob
from sentinel_ui_model import CaptureReadout, MOTOR_LABELS, CAPTURE_STATES
from sentinel_ui_flash import PROBE_CONFIGS, discover_elfs


class FormPage:
    def __init__(self, kind, root, source, dpg, parent, *, images=None, capture_baud=1000000,
                 actions_parent=None):
        self.kind, self.root, self.source, self.dpg = kind, root, source, dpg
        self.job = None
        self.lines = []
        self.closing = False
        self.fields = {}
        self.serial_ports = {}
        self.capture_baud = capture_baud
        self.readout = CaptureReadout() if kind == 'capture-inspect' else None
        self.capture_modules = None
        self.logging_enabled = False
        self.stream_summary = None
        self.flash_result = None
        if kind == 'capture-inspect':
            from sentinel_ui_adapters import SourceModules
            self.capture_modules = SourceModules([source / 'boards/sentinel_f407/tools'])
            try:
                self.capture = self.capture_modules.load('sentinel_capture')
            except Exception:
                self.capture_modules.close()
                raise
        self.stopping = False
        self.checking_probe = False
        self.start_label = {'capture': '开始记录', 'flash': '备份并烧录',
                            'capture-inspect': '开始检查', 'monitor': '开始观测',
                            'build': '开始构建', 'audit': '检查固件',
                            'identify': '开始辨识', 'tune': '生成调参建议'}.get(kind, '执行')
        with dpg.theme() as self.primary_theme:
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (38, 99, 179))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (31, 85, 156))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (24, 68, 128))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255))
        with dpg.group(parent=actions_parent if actions_parent is not None else parent):
            with dpg.group(horizontal=True):
                if kind == 'flash':
                    dpg.add_button(label='检测探针连接', tag='ui.form.check', callback=self.check_probe,
                                   width=155, height=42)
                start = dpg.add_button(label=self.start_label, tag='ui.form.start', callback=self.start,
                                       width=175, height=42)
                dpg.bind_item_theme(start, self.primary_theme)
                dpg.add_button(label='停止检查' if kind == 'capture-inspect' else '停止 / 取消',
                               tag='ui.form.stop', callback=self.cancel, enabled=False, width=135, height=42)
            dpg.add_text('就绪，请填写参数后点击“' + self.start_label + '”。', tag='ui.form.status', wrap=0)
            dpg.add_separator()
        with dpg.group(parent=parent):
            if kind == 'capture-inspect':
                dpg.add_text('检查电机 / IMU', color=(40, 99, 170))
                dpg.add_text('全车 10 路电机只读检查 · 115200 baud · 不使能、不驱动、不置零')
                self.port_field()
                with dpg.group(tag='ui.capture.settings'):
                    dpg.add_text('采集设置与日志', color=(40, 99, 170))
                    self.field('hz', '总分页查询频率（1–15 Hz）', '10')
                    self.field('log', 'JSONL 记录路径（可选，新文件）', '')
                    self.field('hardware', '机械 / 接线版本标签', 'unconfirmed')
                    self.field('capture_elf', '对应 ELF（可选，仅将哈希记入日志）', '')
                    self.field('duration', '检查时长（秒，0 表示持续检查）', '0')
                    if '--baud' in self.capture.main.__code__.co_consts:
                        self.field('baud', '串口波特率（原版参数）', '115200')
                dpg.add_separator()
                with dpg.group(horizontal=True):
                    dpg.add_button(label='全车轮询', callback=lambda: self.capture_action('focus', page=None))
                    dpg.add_button(label='遥控 / IMU 优先', callback=lambda: self.capture_action('focus', page=6))
                dpg.add_text('全车电机在线状态')
                with dpg.table(header_row=True, resizable=True, policy=dpg.mvTable_SizingStretchProp,
                               borders_innerH=True, borders_outerH=True, borders_innerV=True):
                    for title in ('电机', 'CAN / ID', '状态', '编码器 / P16', 'RPM / V12', '温度 °C', '反馈年龄 ms', '角度 rad / DM 状态'):
                        dpg.add_table_column(label=title)
                    for name, label in MOTOR_LABELS.items():
                        with dpg.table_row():
                            dpg.add_selectable(label=label, span_columns=True,
                                               callback=lambda s, a, u: self.capture_action('focus', page=u),
                                               user_data=1 + self.capture.NAMES.index(name) // 2)
                            for index, value in enumerate(self.readout.row(name, time.monotonic())):
                                dpg.add_text(value, tag=f'ui.motor.{name}.{index}')
                            dpg.add_text('--', tag=f'ui.motor.{name}.value')
                dpg.add_text('未检查', tag='ui.capture.link', wrap=0)
                dpg.add_text('DR16：未检查    IMU：未检查', tag='ui.capture.sensors', wrap=0)
                dpg.add_text('在线状态来自板端反馈；连接中断时保留缓存，分页快照较旧不判为电机失联。',
                             color=(100, 110, 125), wrap=0)
                with dpg.collapsing_header(label='DM 只读参数', default_open=True):
                    with dpg.table(header_row=True, resizable=True, policy=dpg.mvTable_SizingStretchProp):
                        for title in ('参数', 'RID', '原始位值', '参数值', '样本年龄 ms', '质量'):
                            dpg.add_table_column(label=title)
                        for i, (rid, name, _) in enumerate(self.capture.REGISTERS):
                            with dpg.table_row():
                                dpg.add_selectable(label=name, span_columns=True,
                                                   callback=lambda s, a, u: self.capture_action('focus', page=u),
                                                   user_data=7+i//4)
                                dpg.add_text(f'0x{rid:02X}')
                                for key in ('raw', 'value', 'age', 'state'):
                                    dpg.add_text('--', tag=f'ui.register.{name}.{key}')
                dpg.add_text('机械标记（仅记录到 PC，不改变设备零位）')
                self.field('mark_label', '机械标记', '')
                self.field('mark_reference', '独立量角 / 单位 / 备注', '')
                with dpg.group(horizontal=True):
                    dpg.add_button(label='记录标记', tag='ui.capture.mark', enabled=False,
                                   callback=lambda: self.capture_action('mark', label=self.value('mark_label'),
                                                                        reference=self.value('mark_reference')))
                    dpg.add_button(label='暂停记录', tag='ui.capture.pause', enabled=False,
                                   callback=lambda: self.capture_action('pause'))
                with dpg.collapsing_header(label='JSONL 导出 CSV（离线）'):
                    self.field('export_source', '原始 JSONL', '')
                    self.field('export_target', '新 CSV 文件', '')
                    dpg.add_button(label='导出 CSV', callback=self.export_capture)
            elif kind == 'capture':
                baud_label = f'{capture_baud:,} baud' if capture_baud is not None else '波特率沿用所选源码'
                dpg.add_text(f'连续只读采集 · {baud_label} · 需匹配 capture 固件')
                dpg.add_text('停止记录只结束采集，不代表机构停止；不发送电机停控命令。')
                self.port_field()
                self.field('seconds', '记录秒数 (1..1800)', '30')
                dpg.add_text('采集轴：静止 / 主 yaw / 发射 yaw / pitch')
                dpg.add_combo(['stationary', 'main', 'fire', 'pitch'], default_value='stationary',
                              tag='ui.field.axis', width=300)
                self.fields['axis'] = 'ui.field.axis'
                self.field('note', '工况备注', '')
                self.field('output', '输出目录', str(root / 'build/capture-records'))
                with dpg.group(horizontal=True):
                    dpg.add_button(label='检查电机 / IMU', tag='ui.capture.inspect', callback=self.inspect_requested)
                    dpg.add_button(label='打开记录目录', callback=self.open_directory)
            elif kind == 'monitor':
                dpg.add_text('S1 板级观测 · 遥控器 / IMU / CAN')
                self.port_field()
                self.field('hz', '查询频率（1–10 Hz）', '5')
            elif kind == 'identify':
                dpg.add_text('离线台架辨识；文件路径可用引号包裹，多个文件用空格分隔。')
                for key, label in [('train', '训练记录'), ('validate', '验证记录'), ('output', '模型 JSON 输出')]:
                    self.field(key, label, '')
            elif kind == 'tune':
                dpg.add_text('离线调参建议；不连接串口、不下发参数。')
                dpg.add_text('将运行所选原生 C++ 仿真程序；不是选择测速日志。请只选择可信的离线仿真程序。')
                for key, label in [('model', '辨识模型 JSON'), ('native', '原生 C++ 仿真可执行程序'), ('output', '建议 JSON 输出')]:
                    self.field(key, label, '')
            elif kind == 'audit':
                dpg.add_text('仅 S1/S5 ELF 静态审计；不能作为 APP 或 capture 镜像的审计结论。')
                self.field('elf', 'ELF 文件', '')
                self.field('nm', 'arm-none-eabi-nm', shutil.which('arm-none-eabi-nm') or '')
                dpg.add_checkbox(label='明确按 S5 GM6020 台架审计', tag='ui.field.s5')
                self.fields['s5'] = 'ui.field.s5'
                dpg.add_text('第一版不提供烧录。Windows 探针枚举尚未移植。')
            elif kind == 'build':
                dpg.add_text('构建已有 CMake 工程；不自动配置、不更改机构参数、不烧录。')
                self.field('directory', '已配置的构建目录', str(source / 'build'))
                self.field('cmake', 'cmake 程序', shutil.which('cmake') or 'cmake')
                self.field('target', '目标（空=默认）', '')
            elif kind == 'flash':
                dpg.add_text('固件烧录', color=(40, 99, 170))
                dpg.add_text('自动备份 → 擦写 → 校验 → 复位')
                dpg.add_text('仅连接调试探针，请断开动力电源。烧录会覆盖原有固件。', color=(144, 94, 30))
                dpg.add_separator()
                dpg.add_text('固件文件')
                elfs = [str(path) for path in images] if images is not None else discover_elfs(root, source)
                if images is not None and any(not Path(path).is_file() for path in images):
                    dpg.add_text('配套固件文件缺失；请先构建对应版本或手动指定 ELF。', color=(165, 65, 45))
                dpg.add_text('选择 ELF（最新构建在前）')
                dpg.add_combo(elfs, default_value=elfs[0] if elfs else '',
                              tag='ui.field.elf', width=-1, callback=self.elf_chosen)
                self.fields['elf'] = 'ui.field.elf'
                dpg.add_text('手动路径（可选，留空使用上方选择）')
                dpg.add_input_text(tag='ui.field.elf_manual', width=-1, hint='需要时输入 ELF 文件路径',
                                   on_enter=True, callback=self.elf_chosen)
                self.fields['elf_manual'] = 'ui.field.elf_manual'
                dpg.add_separator()
                dpg.add_text('调试连接')
                self.field('openocd', 'OpenOCD 程序', shutil.which('openocd') or 'openocd')
                dpg.add_text('探针类型')
                dpg.add_combo(list(PROBE_CONFIGS), default_value='ST-Link',
                              tag='ui.field.probe', width=300, callback=self.probe_changed)
                self.fields['probe'] = 'ui.field.probe'
                dpg.add_text('SWD · 200 kHz · 仅连接一个所选类型的探针', color=(100, 110, 125))
                dpg.add_text('', tag='ui.probe', wrap=0)
            else:
                raise ValueError('未知页面')
            dpg.add_separator()
            dpg.add_text('运行日志', color=(100, 110, 125))
            dpg.add_input_text(tag='ui.output', multiline=True, readonly=True, width=-1, height=220)

    def field(self, key, label, value):
        tag = 'ui.field.' + key
        self.dpg.add_text(label)
        self.dpg.add_input_text(default_value=value, tag=tag, width=-1)
        self.fields[key] = tag

    def port_field(self):
        self.dpg.add_text('串口设备')
        with self.dpg.table(header_row=False, policy=self.dpg.mvTable_SizingStretchProp,
                            width=-1, no_saved_settings=True):
            self.dpg.add_table_column(width_fixed=True, init_width_or_weight=145)
            self.dpg.add_table_column(width_stretch=True)
            with self.dpg.table_row():
                self.dpg.add_button(label='刷新串口', tag='ui.port.refresh', callback=self.scan, width=130, height=36)
                self.dpg.add_combo([], tag='ui.field.port', width=-1, default_value='', callback=self.port_changed)
        self.fields['port'] = 'ui.field.port'
        self.dpg.add_text('', tag='ui.ports', wrap=0)
        with self.dpg.collapsing_header(label='查看全部串口说明'):
            self.dpg.add_input_text(tag='ui.ports.details', multiline=True, readonly=True, width=-1, height=100)
        self.dpg.add_text('手动串口路径（可选，输入后点击“应用”）')
        with self.dpg.table(header_row=False, policy=self.dpg.mvTable_SizingStretchProp,
                            width=-1, no_saved_settings=True):
            self.dpg.add_table_column(width_stretch=True)
            self.dpg.add_table_column(width_fixed=True, init_width_or_weight=105)
            with self.dpg.table_row():
                self.dpg.add_input_text(tag='ui.port.manual', width=-1, on_enter=True,
                                       hint='/dev/ttyUSB0 或 /dev/serial/by-id/...', callback=self.manual_port)
                self.dpg.add_button(label='应用', tag='ui.port.apply', width=90, height=36, callback=self.manual_port)
        self.fields['port_manual'] = 'ui.port.manual'
        self.scan()

    def port_changed(self):
        current = self.dpg.get_value('ui.field.port')
        detail = self.serial_ports.get(current, '')
        self.dpg.set_value('ui.ports', f'发现 {len(self.serial_ports)} 个串口 · ' +
                           (f'已选：{current}' if current else '请在右侧下拉框选择') +
                           (f' · {detail}' if detail and detail != 'n/a' else ''))

    def manual_port(self):
        self.dpg.set_value('ui.field.port', self.dpg.get_value('ui.port.manual').strip())
        self.port_changed()

    def inspect_requested(self):
        if self.job and not self.job.released():
            return
        if hasattr(self, 'switch_tool'):
            output = Path(self.value('output')).expanduser().resolve()
            output.mkdir(parents=True, exist_ok=True)
            self.switch_tool('capture-inspect', defaults=dict(port=self.value('port'),
                log=str(output / f'mapping-{time.time_ns()}.jsonl')))

    def open_directory(self):
        path = Path(self.value('output')).expanduser().resolve()
        path.mkdir(parents=True, exist_ok=True)
        subprocess.Popen(['xdg-open', str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def capture_action(self, action, **values):
        try:
            if not self.job or self.job.released():
                raise ValueError('请先开始检查')
            self.job.send_line(json.dumps(dict(action=action, **values), ensure_ascii=False))
        except (OSError, ValueError) as error:
            self.dpg.set_value('ui.form.status', str(error))

    def export_capture(self):
        try:
            if not self.value('export_source') or not self.value('export_target'):
                raise ValueError('请选择 JSONL 和新 CSV 路径')
            self.capture.export_csv(Path(self.value('export_source')).expanduser(),
                                    Path(self.value('export_target')).expanduser())
            self.dpg.set_value('ui.form.status', 'CSV 已导出：' + self.value('export_target'))
        except (OSError, ValueError) as error:
            self.dpg.set_value('ui.form.status', '导出失败：' + str(error))

    def value(self, key):
        value = self.dpg.get_value(self.fields[key])
        return value.strip() if isinstance(value, str) else value

    def elf_chosen(self):
        manual = self.value('elf_manual')
        if manual:
            self.dpg.set_value('ui.field.elf', manual)
        path = Path(self.value('elf')).expanduser()
        shown = f'已选: {path}' if path.is_file() else f'路径不存在: {path}'
        self.dpg.set_value('ui.probe', shown)

    def probe_changed(self):
        self.dpg.set_value('ui.probe', '已选择 ' + self.value('probe') + '，请重新检查探针。')

    def check_probe(self):
        command = [str(self.root / '.venv/bin/python'), '-u', str(self.root / 'scripts/sentinel_ui_flash.py'),
                   '--check-probe', '--root', str(self.root), '--openocd', self.value('openocd'),
                   '--probe', self.value('probe')]
        self.launch(command, checking_probe=True)

    def tail(self, text, lines=6):
        return '\n'.join((text.strip().splitlines() or ['(无输出)'])[-lines:])

    def executable(self, value):
        try:
            return os.access(Path(value).expanduser(), os.X_OK)
        except OSError:
            return False

    def scan(self):
        from serial.tools import list_ports
        self.dpg.configure_item('ui.port.refresh', label='刷新中…', enabled=False)
        try:
            ports = sorted(list_ports.comports(), key=lambda p: p.device)
            self.serial_ports = {p.device: p.description for p in ports}
            current = self.dpg.get_value('ui.field.port')
            self.dpg.configure_item('ui.field.port', items=[p.device for p in ports])
            self.dpg.set_value('ui.field.port', current)
            self.dpg.set_value('ui.ports.details', '\n'.join(f'{p.device} · {p.description}' for p in ports))
            if ports:
                self.port_changed()
            else:
                self.dpg.set_value('ui.ports', '未发现串口；连接设备后刷新，或手动输入路径。')
        except OSError as error:
            self.dpg.set_value('ui.ports', '串口刷新失败，可直接手动输入路径：' + str(error))
        finally:
            self.dpg.configure_item('ui.port.refresh', label='刷新串口', enabled=True)

    def command(self):
        python = str(self.root / '.venv/bin/python')
        if self.kind == 'capture-inspect':
            target = self.source / 'boards/sentinel_f407/tools/sentinel_capture.py'
            hz = float(self.value('hz'))
            seconds = float(self.value('duration'))
            if not target.is_file():
                raise ValueError('所选源码缺少只读采集工具: ' + str(target))
            if not self.value('port') or not math.isfinite(hz) or not 1 <= hz <= 15:
                raise ValueError('请指定串口和 1–15 Hz 查询频率')
            if not math.isfinite(seconds) or seconds < 0:
                raise ValueError('检查时长必须是大于等于 0 的有限秒数')
            command = [python, '-u', str(self.root / 'scripts/sentinel_capture_worker.py'),
                       '--mode', 'inspect', '--source', str(self.source), '--port', self.value('port'),
                       '--hz', str(hz), '--hardware', self.value('hardware'), '--seconds', str(seconds)]
            log = self.value('log')
            if log:
                if Path(log).expanduser().exists():
                    raise ValueError('记录文件已存在，请使用新文件名')
                command += ['--log', str(Path(log).expanduser().resolve())]
            if self.value('capture_elf'):
                command += ['--elf', str(Path(self.value('capture_elf')).expanduser().resolve())]
            if 'baud' in self.fields:
                command += ['--baud', str(int(self.value('baud')))]
            return command
        if self.kind == 'monitor':
            hz = float(self.value('hz'))
            if not self.value('port') or not 1 <= hz <= 10:
                raise ValueError('请指定串口和 1–10 Hz 查询频率')
            return [python, '-u', str(self.root / 'common/board_tools/board_monitor.py'),
                    '--port', self.value('port'), '--hz', str(hz), '--headless']
        if self.kind == 'capture':
            if not (self.source / 'robots/sentinel/host/sentinel_stream.py').is_file():
                raise ValueError('所选源码缺少连续采集实现；请返回首页选择正确源码版本')
            seconds = float(self.value('seconds'))
            if not self.value('port') or not 1 <= seconds <= 1800:
                raise ValueError('请指定串口和 1..1800 秒采集时长')
            output = Path(self.value('output')).expanduser().resolve()
            if not self.value('output'):
                raise ValueError('请指定输出目录')
            return [python, '-u', str(self.root / 'scripts/sentinel_capture_worker.py'),
                    '--source', str(self.source), '--port', self.value('port'),
                    '--seconds', str(seconds), '--axis', self.value('axis'), '--note', self.value('note'),
                    *(['--baud', str(self.capture_baud)] if self.capture_baud is not None else []),
                    '--output', str(output / f'{self.value("axis")}-{time.time_ns()}.sraw')]
        if self.kind == 'build':
            directory = Path(self.value('directory')).expanduser().resolve()
            if not (directory / 'CMakeCache.txt').is_file():
                raise ValueError('目录内没有 CMakeCache.txt；请先配置工程，本页仅构建已有工程')
            command = [self.value('cmake'), '--build', str(directory), '--parallel', '2']
            if self.value('target'):
                command += ['--target', self.value('target')]
            return command
        if self.kind == 'flash':
            elf = self.value('elf')
            if not Path(elf).expanduser().is_file():
                raise ValueError('请选择或输入存在的固件 ELF；不存在: ' + elf)
            return [python, '-u', str(self.root / 'scripts/sentinel_ui_flash.py'), '--flash',
                    '--root', str(self.root), '--elf', str(Path(elf).expanduser().resolve()),
                    '--openocd', self.value('openocd'), '--probe', self.value('probe')]
        if self.kind == 'audit':
            if not Path(self.value('elf')).is_file() or not self.value('nm'):
                raise ValueError('请选择现存 ELF 和 nm 程序')
            command = [python, str(self.root / 'common/build_verify/check_firmware.py'),
                       '--elf', str(Path(self.value('elf')).expanduser().resolve()), '--nm', self.value('nm')]
            return command + (['--allow-gm6020-bench'] if self.value('s5') else [])
        script = f'motor_bench_{self.kind}.py'
        command = [python, str(self.root / 'common/motor_bench' / script)]
        for key in self.fields:
            value = self.value(key)
            if not value:
                raise ValueError('请填写全部文件参数')
            values = shlex.split(value) if key in ('train', 'validate') else [value]
            if not values:
                raise ValueError('文件列表不能为空')
            if key != 'output' and any(not Path(path).expanduser().is_file() for path in values):
                raise ValueError('输入文件不存在: ' + value)
            if key == 'native' and not self.executable(value):
                raise ValueError('原生 C++ 仿真程序必须是 Linux 可执行文件，不是测速记录或 Windows .exe')
            if key == 'output' and Path(value).expanduser().exists():
                raise ValueError('输出文件已存在，请使用新文件名，避免覆盖')
            command += ['--' + key, *[str(Path(path).expanduser().resolve()) for path in values]]
        return command

    def start(self):
        if self.closing or (self.job and not self.job.released()):
            return
        try:
            command = self.command()
        except (OSError, ValueError) as error:
            self.dpg.set_value('ui.form.status', '未启动: ' + str(error))
            return
        self.launch(command)

    def launch(self, command, *, checking_probe=False):
        if self.closing:
            return
        try:
            if self.job and not self.job.released():
                raise ValueError('上一任务资源仍未释放')
            env = dict(os.environ, PYTHONUNBUFFERED='1', MPLBACKEND='Agg')
            env.pop('PYTHONHOME', None)
            env.pop('PYTHONPATH', None)
            self.job = ProcessJob(command, cwd=self.root, env=env,
                                  hardware=self.kind in ('capture', 'capture-inspect', 'flash', 'monitor'),
                                  **({'input_pipe': True} if self.kind == 'capture-inspect' else {}))
            self.lines = []
            self.stopping = False
            self.checking_probe = checking_probe
            self.logging_enabled = bool(self.value('log')) if self.kind == 'capture-inspect' else False
            self.stream_summary = None
            self.flash_result = None
            if self.readout is not None:
                self.readout.reset()
            self.dpg.set_value('ui.form.status',
                               '输出线程启动失败，已请求取消；请返回释放资源: ' + self.job.startup_error
                               if self.job.startup_error else
                               ('正在检测探针连接…' if checking_probe else '已启动，正在等待工具反馈…'))
            self.sync_buttons()
        except (OSError, ValueError) as error:
            self.dpg.set_value('ui.form.status', '未启动: ' + str(error))

    def cancel(self):
        if self.job:
            try:
                self.job.request_stop(kill_after=5.0 if self.kind == 'flash' else 1.5)
                self.stopping = True
                self.dpg.set_value('ui.form.status', '已发送停止请求；正在关闭串口和后台进程…')
                self.sync_buttons()
            except OSError as error:
                self.dpg.set_value('ui.form.status', '取消请求失败，资源仍受返回门禁管理: ' + str(error))

    @property
    def needs_confirmation(self):
        return bool(self.job and self.job.needs_confirmation and not self.job.released())

    def tick(self):
        if not self.job or self.closing:
            return
        changed = False
        while True:
            try:
                line = self.job.events.get_nowait()
                if self.kind == 'flash' and line.startswith(('FLASH_STAGE_TIMEOUT ', 'FLASH_STAGE_FAILED ', 'FLASH_STAGE_CANCELLED ')):
                    self.flash_result = line.partition(' ')[2]
                    self.dpg.set_value('ui.form.status', self.flash_result)
                if self.readout is not None or self.kind == 'capture':
                    try:
                        event = json.loads(line)
                    except json.JSONDecodeError:
                        event = None
                    if isinstance(event, dict):
                        if self.readout is not None:
                            self.readout.consume(event, time.monotonic())
                        else:
                            self.stream_event(event)
                        if event.get('event') == 'ui_action':
                            self.logging_enabled = event['logging']
                            self.dpg.set_value('ui.form.status', event['message'])
                self.lines.append(line)
                self.lines = self.lines[-200:]
                changed = True
            except queue.Empty:
                break
        if changed:
            self.dpg.set_value('ui.output', '\n'.join(self.lines))
            if self.checking_probe:
                self.dpg.set_value('ui.probe', self.tail('\n'.join(self.lines)))
        code = self.job.process.poll()
        if code is not None:
            self.dpg.set_value('ui.form.status', (self.stream_summary + '\n' if self.stream_summary else '') +
                               f'进程已结束，退出码 {code}；' +
                               ('资源已释放' if self.job.released() else '仍在等待后台资源释放'))
            if self.readout is not None:
                self.readout.link = 'CLOSED'
            if self.flash_result:
                self.dpg.set_value('ui.form.status', self.flash_result +
                                   ('（后台已退出）' if self.job.released() else ''))
            elif self.stopping:
                self.dpg.set_value('ui.form.status', '本次操作已停止，连接已关闭。' if self.job.released()
                                   else '后台已停止，正在关闭残留连接…')
            if self.checking_probe and self.job.released():
                passed = code == 0 and any('FLASH_TARGET_OK' in line for line in self.lines)
                self.dpg.set_value('ui.form.status', '探针及目标连接正常，可以烧录。' if passed else
                                   '探针检查未通过，请查看下方检测结果。')
        if self.readout is not None:
            self.render_capture()
        self.sync_buttons()

    def sync_buttons(self):
        running = bool(self.job and not self.job.released())
        label = self.start_label
        if running:
            label = ('正在停止…' if self.stopping else '结束任务中…' if self.flash_result else
                     '正在检测…' if self.checking_probe else '运行中…')
        self.dpg.configure_item('ui.form.start', enabled=not running and not self.closing, label=label)
        self.dpg.configure_item('ui.form.stop', enabled=running and not self.stopping,
                                label='正在停止…' if running and self.stopping else
                                ('停止检查' if self.kind == 'capture-inspect' else '停止 / 取消'))
        if self.kind == 'flash':
            self.dpg.configure_item('ui.form.check', enabled=not running and not self.closing,
                                    label='正在检测…' if running and self.checking_probe else '检测探针连接')
        for key, tag in self.fields.items():
            self.dpg.configure_item(tag, enabled=not running or key in (
                'mark_label', 'mark_reference', 'export_source', 'export_target'))
        if 'port' in self.fields:
            self.dpg.configure_item('ui.port.apply', enabled=not running)
        if self.kind == 'capture-inspect':
            self.dpg.configure_item('ui.capture.mark', enabled=running)
            self.dpg.configure_item('ui.capture.pause', enabled=running and bool(self.value('log')),
                                    label='暂停记录' if self.logging_enabled else '恢复记录')
        if self.kind == 'capture':
            self.dpg.configure_item('ui.capture.inspect', enabled=not running)

    def stream_event(self, event):
        kind = event.get('event')
        message = None
        if kind == 'recording':
            message = '正在记录 — 现在开始缓慢手动运动；静止项目请保持不动。\n' + event['path']
        elif kind == 'progress':
            message = (f'正在记录：{event["records"]} 条 | 板端丢弃 {event["dropped"]} | '
                       f'记录缺口 {event["gaps"]} | CRC 错误 {event["crc_errors"]}')
        elif kind == 'finished':
            result = event['summary']
            counts = result['received']
            message = '记录结束 · ' + ('传输完整' if result['complete'] else '有缺口/异常，请保留文件并反馈')
            if result.get('source_issues') or result.get('diagnostics_error'):
                message = '记录结束 · 板端诊断有异常或无法读取，请保留文件并反馈'
            message += (f'\n陀螺仪 {counts.get("gyro:0", 0)} / 加速度 {counts.get("accel:0", 0)} / '
                        f'主 Yaw {counts.get("can:4", 0)} / 发射 Yaw {counts.get("can:5", 0)} / '
                        f'Pitch {counts.get("can:9", 0)}\n{event["path"]}')
        elif kind == 'error':
            message = '采集未完成：' + event['message']
        if message:
            if kind in ('finished', 'error'):
                self.stream_summary = message
            self.dpg.set_value('ui.form.status', message)

    def render_capture(self):
        now = time.monotonic()
        for name in MOTOR_LABELS:
            for index, value in enumerate(self.readout.row(name, now)):
                tag = f'ui.motor.{name}.{index}'
                self.dpg.set_value(tag, value)
                if index == 1:
                    color = (40, 135, 80) if value == '在线' else (144, 94, 30)
                    self.dpg.configure_item(tag, color=color)
            if name in self.readout.motors:
                motor = self.readout.motors[name][0]
                valid = motor['seen'] and motor.get('decoded_valid', True)
                angle = motor.get('rotor_rad')
                value = f'{angle:.5f}; {motor.get("rpm")}' if valid and angle is not None else '--'
                if valid and motor['kind'] == 4:
                    value = f'state={motor.get("status")} v12={motor.get("v12")}'
                self.dpg.set_value(f'ui.motor.{name}.value', value)
            else:
                self.dpg.set_value(f'ui.motor.{name}.value', '--')
        self.dpg.set_value('ui.capture.link', f'连接：{self.readout.link}    固件：{self.readout.build_id or "未读取"}')
        sensors = self.readout.sensors
        states = [CAPTURE_STATES.get(sensors[key]['state'], sensors[key]['state'])
                  if key in sensors else '未检查' for key in ('remote', 'imu')]
        cached = '（连接中断，缓存）' if self.readout.link not in ('ONLINE', 'WAITING') else ''
        self.dpg.set_value('ui.capture.sensors', f'DR16：{states[0]}    IMU：{states[1]}{cached}')
        if 6 in self.readout.pages:
            received, data = self.readout.pages[6]
            self.dpg.set_value('ui.capture.sensors', self.capture.sensor_text(
                data, (now-received)*1000, self.readout.link))
        if 0 in self.readout.pages:
            summary = self.readout.pages[0][1]['summary']
            self.dpg.set_value('ui.capture.link',
                f'{self.readout.link} | {self.readout.build_id} | RX={summary["rx_frames"]} '
                f'drop={summary["queue_dropped"]} max queue={summary["queue_high_water"]} '
                f'| DM reads={summary["queries"]} timeout={summary["query_timeouts"]} | '
                + ('LOG ON' if self.logging_enabled else 'LOG OFF'))
        for _, name, _ in self.capture.REGISTERS:
            if name not in self.readout.registers:
                for key in ('raw', 'value', 'age', 'state'):
                    self.dpg.set_value(f'ui.register.{name}.{key}', '--')
        for page in range(7, 10):
            if page not in self.readout.pages:
                continue
            received, data = self.readout.pages[page]
            for row in data.get('registers', []):
                state = self.capture.display_state(row, (now-received)*1000,
                    500 if row['rid'] >= 0x50 else 5000, self.readout.link)
                values = dict(raw=f'0x{row["bits"]:08X}' if row['seen'] else '--',
                              value=str(row['value']) if row['seen'] and row['decoded_valid'] else '--',
                              age=str(int(row['age_ms']+(now-received)*1000)) if row['seen'] else '--', state=state)
                for key, value in values.items():
                    self.dpg.set_value(f'ui.register.{row["name"]}.{key}', value)

    def prepare_close(self):
        self.closing = True

    def close(self):
        self.prepare_close()
        if self.job:
            self.job.close()

    def released(self):
        return self.job is None or self.job.released()

    def dispose(self):
        if self.capture_modules is not None:
            self.capture_modules.close()

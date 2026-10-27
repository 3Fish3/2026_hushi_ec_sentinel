"""Embed the existing DPG views without changing archived robot source files."""
import importlib
import os
import queue
import re
import subprocess
from concurrent.futures import TimeoutError as FutureTimeout
from pathlib import Path
import sys
import threading
import time

from sentinel_ui_resources import SerialOwner


class SourceModules:
    def __init__(self, paths):
        self.paths = [str(Path(path).resolve()) for path in paths]
        self.previous_path = sys.path[:]
        project = Path(__file__).resolve().parents[1]
        suffixes = ('/robots/sentinel/host', '/boards/sentinel_f407/tools',
                    '/common/motor_bench', '/common/board_tools')
        old_roots = [str(Path(path).resolve()) for path in sys.path
                     if Path(path).resolve().is_relative_to(project)
                     and str(Path(path).resolve()).endswith(suffixes)]
        # Source selection replaces robot search roots; a missing helper must fail,
        # not resolve through the launcher's previously selected PYTHONPATH.
        self._purge(old_roots + self.paths)
        sys.path[:] = self.paths + [path for path in self.previous_path
                                   if str(Path(path).resolve()) not in old_roots]

    def _purge(self, roots=None):
        for name, module in list(sys.modules.items()):
            filename = getattr(module, '__file__', None)
            if filename and any(Path(filename).resolve().is_relative_to(path) for path in (roots or self.paths)):
                del sys.modules[name]

    def load(self, name):
        module = importlib.import_module(name)
        filename = getattr(module, '__file__', None)
        if not filename or not any(Path(filename).resolve().is_relative_to(path) for path in self.paths):
            raise ImportError(f'{name} 不属于所选源码路径，拒绝混用版本')
        return module

    def close(self):
        self._purge()
        sys.path[:] = self.previous_path


class EmbeddedDPG:
    def __init__(self, dpg, parent):
        self.api, self.parent = dpg, parent

    def window(self, **kwargs):
        if kwargs.get('tag') == 'main':
            kwargs.pop('label', None)
            return self.api.child_window(parent=self.parent, width=-1, height=-1,
                                         border=False, **kwargs)
        return self.api.window(**kwargs)

    def add_input_text(self, **kwargs):
        if kwargs.get('tag') == 'port':
            from serial.tools import list_ports
            kwargs.pop('hint', None)
            return self.add_combo([p.device for p in list_ports.comports()], **kwargs)
        return self.api.add_input_text(**kwargs)

    def add_combo(self, items=(), **kwargs):
        if kwargs.get('tag') == 'port':
            kwargs['width'] = max(kwargs.get('width', 0), 300)
        item = self.api.add_combo(items, **kwargs)
        if kwargs.get('tag') == 'port':
            with self.api.popup(item, mousebutton=self.api.mvMouseButton_Right):
                self.api.add_text('手动输入串口路径，回车确认')
                self.api.add_input_text(default_value=kwargs.get('default_value', ''), on_enter=True,
                    callback=lambda s, a: self.api.set_value('port', a.strip()), width=350)
            with self.api.tooltip(item):
                self.api.add_text('下拉选择串口；右键可手动输入路径。')
        return item

    def __getattr__(self, name):
        return getattr(self.api, name)


def light_palette(module):
    for name, color in dict(GREEN=(35, 130, 80), AMBER=(145, 90, 25), RED=(180, 45, 45),
                             MUTED=(100, 110, 125), BLUE=(38, 99, 179)).items():
        if hasattr(module, name):
            setattr(module, name, color)


class FirmwarePage:
    """原 Tk 固件工具的显示适配, 工作流程直接调用原 Window.work."""
    def __init__(self, root, source, dpg, parent):
        from dataclasses import fields
        self.repo, self.dpg = source, dpg
        self.modules = SourceModules([root / 'common/board_tools'])
        self.original = self.modules.load('firmware_tool')
        self.model = self.original.model
        self.settings = self.model.Settings.discover(source)
        self.queue = queue.SimpleQueue()
        self.worker = None
        self.controls = []
        self.lines = []
        self.dialogs = []
        self.action = None
        self.closing = False
        with dpg.group(parent=parent):
            dpg.add_text('原版固件工具 · 构建 / 审计 / 烧录 / 对应上位机')
            dpg.add_text('固件用途')
            self.controls.append(dpg.add_combo([p.title for p in self.model.PROFILES],
                default_value=self.model.PROFILES[0].title, tag='fw.profile', width=-1, callback=self.select_profile))
            with dpg.group(horizontal=True):
                from serial.tools import list_ports
                self.controls.append(dpg.add_combo([p.device for p in list_ports.comports()], tag='fw.port', width=350))
                self.controls.append(dpg.add_button(label='刷新串口', callback=self.scan))
                self.controls.append(dpg.add_button(label='启动对应上位机', callback=self.open_host))
            self.controls.append(dpg.add_input_text(tag='fw.port.manual', hint='或输入串口路径，回车确认', width=480, on_enter=True,
                callback=lambda s, a: dpg.set_value('fw.port', a.strip())))
            dpg.add_text('', tag='fw.detail', wrap=0)
            self.path_field('elf', '固件 ELF', '')
            dpg.add_text('尚未审计', tag='fw.artifact', wrap=0)
            with dpg.group(horizontal=True):
                for label, action in [('检查固件', 'inspect'), ('仅构建', 'build'),
                                      ('烧录已有固件', 'flash'), ('构建并烧录', 'build_flash')]:
                    self.controls.append(dpg.add_button(label=label, height=40,
                        callback=lambda s, a, u: self.start(u), user_data=action))
            dpg.add_text('调试探针')
            self.controls.append(dpg.add_combo(['ST-Link', 'CMSIS-DAP'], default_value='ST-Link', tag='fw.probe', width=300))
            with dpg.collapsing_header(label='工具路径（原版自动发现及保存位置）'):
                labels = dict(cmake='CMake', ninja='Ninja', arm_root='ARM 工具链目录', sdk_root='STM32CubeF4 目录',
                              openocd='OpenOCD', scripts='OpenOCD scripts 目录', python='项目 Python', pwsh='PowerShell 7')
                for field in fields(self.model.Settings):
                    self.path_field(field.name, labels[field.name], getattr(self.settings, field.name),
                                    directory=field.name in ('arm_root', 'sdk_root', 'scripts'))
            dpg.add_text('就绪，未访问硬件。', tag='fw.status', wrap=0)
            dpg.add_input_text(tag='fw.output', multiline=True, readonly=True, width=-1, height=300)
        self.select_profile()

    def path_field(self, key, label, value, directory=False):
        dpg = self.dpg
        dpg.add_text(label)
        with dpg.group(horizontal=True):
            self.controls.append(dpg.add_input_text(default_value=value, tag='fw.'+key, width=-140))
            with dpg.file_dialog(show=False, modal=True, directory_selector=directory, width=800, height=450,
                    callback=lambda s, a: dpg.set_value('fw.'+key, a['file_path_name'])) as dialog:
                if not directory:
                    dpg.add_file_extension('.*')
            self.dialogs.append(dialog)
            self.controls.append(dpg.add_button(label='选择…', width=120,
                callback=lambda s, a, u: dpg.configure_item(u, show=True), user_data=dialog))

    def profile(self):
        return next(p for p in self.model.PROFILES if p.title == self.dpg.get_value('fw.profile'))

    def select_profile(self):
        p = self.profile()
        self.dpg.set_value('fw.elf', str(self.model.default_elf(self.repo, p)))
        self.dpg.set_value('fw.detail', f'{p.purpose}\n连接：{p.port} | 对应：{p.host}')
        self.dpg.set_value('fw.artifact', '尚未审计；烧录前按所选用途重新审计镜像。')

    def current_settings(self):
        return self.model.Settings(**{key: self.dpg.get_value('fw.'+key).strip()
                                     for key in self.model.Settings.__dataclass_fields__})

    def scan(self):
        from serial.tools import list_ports
        selected = self.dpg.get_value('fw.port')
        self.dpg.configure_item('fw.port', items=[p.device for p in list_ports.comports()])
        self.dpg.set_value('fw.port', selected)

    def start(self, action):
        if self.closing or not self.released():
            return
        profile, settings = self.profile(), self.current_settings()
        elf = Path(self.dpg.get_value('fw.elf')).expanduser()
        probe = self.dpg.get_value('fw.probe')
        self.action = action
        for item in self.controls:
            self.dpg.configure_item(item, enabled=False)
        self.dpg.set_value('fw.status', '正在执行：' + action)
        original_flash = self.model.flash_image
        def work():
            try:
                if os.name != 'nt' or probe == 'CMSIS-DAP':
                    def probes(settings, log, run):
                        from sentinel_ui_flash import STLINK_USB_PIDS
                        result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=20, check=True)
                        found = []
                        for line in result.stdout.splitlines():
                            match = re.search(r'\bID\s+([0-9a-f]{4}):([0-9a-f]{4})\b', line, re.I)
                            if match:
                                vid, pid = (v.lower() for v in match.groups())
                                if ((probe == 'ST-Link' and vid == '0483' and pid in STLINK_USB_PIDS) or
                                        (probe == 'CMSIS-DAP' and (vid, pid) == ('c251', 'f001'))):
                                    found.append(line)
                        log(f'{probe} 数量：{len(found)}')
                        return found
                    def run(args, cwd, log, timeout=180):
                        args = list(args)
                        if probe == 'CMSIS-DAP' and 'interface/stlink.cfg' in args:
                            i = args.index('interface/stlink.cfg')
                            args[i] = 'interface/cmsis-dap.cfg'
                            args[i+1:i+1] = ['-c', 'cmsis_dap_vid_pid 0xc251 0xf001; cmsis_dap_backend hid; transport select swd']
                        return self.model.run_command(args, cwd, log, timeout)
                    self.model.flash_image = lambda image, settings, log: original_flash(
                        image, settings, log, run=run, probes=probes)
                self.original.Window.work(self, action, profile, settings, elf)
            finally:
                self.model.flash_image = original_flash
        self.worker = threading.Thread(target=work, name='original-firmware-work', daemon=False)
        self.worker.start()

    def open_host(self):
        if self.closing:
            return
        try:
            args = self.model.host_command(self.repo, self.profile(), self.current_settings(), self.dpg.get_value('fw.port'))
            subprocess.Popen(args, cwd=self.repo)
            self.dpg.set_value('fw.status', '已启动对应原版上位机。')
        except (OSError, ValueError) as error:
            self.dpg.set_value('fw.status', str(error))

    def tick(self):
        while not self.queue.empty():
            kind, value = self.queue.get_nowait()
            if kind == 'elf':
                self.dpg.set_value('fw.elf', value)
            elif kind == 'image':
                self.dpg.set_value('fw.artifact', value)
            elif kind == 'log':
                self.lines = (self.lines + [value])[-600:]
                self.dpg.set_value('fw.output', '\n'.join(self.lines))
            else:
                self.dpg.set_value('fw.status', ('失败：' if kind == 'error' else '') + value)
        if self.released():
            for item in self.controls:
                self.dpg.configure_item(item, enabled=True)

    @property
    def needs_confirmation(self):
        return not self.released() and self.action in ('flash', 'build_flash')

    def released(self):
        return self.worker is None or not self.worker.is_alive()

    def prepare_close(self):
        self.closing = True

    def close(self):
        if self.worker:
            self.worker.join(timeout=3)
        if not self.released():
            raise RuntimeError('原版固件任务仍在运行，请等待结束后返回；不在烧录中途取消')

    def dispose(self):
        self.modules.close()


class BenchPage:
    def __init__(self, root, dpg, parent, leave, *, offline=False, serial_factory=None):
        self.closing = False
        self.next_refresh = 0
        self.modules = SourceModules([root / 'common/motor_bench', root / 'common/board_tools'])
        try:
            module = self.modules.load('motor_bench_console')
            light_palette(module)
            setattr(module, 'dpg', EmbeddedDPG(dpg, parent))
            if serial_factory is None:
                import serial
                serial_factory = serial.Serial
            self.serials = SerialOwner(serial_factory)
            page = self

            class EmbeddedConsole(module.Console):
                def connect(self):
                    if not page.released():
                        raise ValueError('之前的串口或线程尚未释放，请先返回并完成资源清理')
                    super().connect()

                def disconnect(self):
                    if not page.closing:
                        super().disconnect()

                def stop(self):
                    if not page.closing:
                        super().stop()

                def invoke(self, action):
                    if not page.closing:
                        super().invoke(action)

            def factory(*args, **kwargs):
                if offline:
                    raise ValueError('离线回放页面不允许连接串口')
                return module.bench.Session(*args, serial_factory=self.serials.open, **kwargs)

            self.view = EmbeddedConsole('', root / 'build/motor-bench-logs', session_factory=factory)
            self.view.scan_ports()
            if offline:
                for tag in ('port', 'connect', 'disconnect', 'controls'):
                    dpg.configure_item(tag, show=False)
        except Exception:
            self.modules.close()
            raise

    @property
    def needs_confirmation(self):
        return bool(self.view.session) or not self.serials.released()

    def tick(self):
        if not self.closing and time.monotonic() >= self.next_refresh:
            self.view.refresh()
            self.next_refresh = time.monotonic() + .1

    def prepare_close(self):
        self.closing = True
        self.serials.request_freeze()

    def close(self):
        self.prepare_close()
        # Freeze first: the legacy Session finally block attempts a STOP frame.
        # It must be rejected, not acknowledged as a successful write.
        self.serials.freeze()
        if self.view.session:
            self.view.session.quit_event.set()
            if self.view.session.ident is not None:
                self.view.session.join(timeout=3)
        self.serials.close()
        self.view.worker.shutdown(wait=True, cancel_futures=True)
        if not self.released():
            raise RuntimeError('台架线程或串口尚未释放')

    def released(self):
        session = self.view.session
        return self.serials.released() and (session is None or not session.is_alive())

    def dispose(self):
        self.modules.close()


class TransportGate:
    def __init__(self, raw, closing, lock):
        self.raw, self.closing, self.lock = raw, closing, lock

    @property
    def observer(self):
        return self.raw.observer

    @observer.setter
    def observer(self, value):
        self.raw.observer = value

    def send(self, data):
        with self.lock:
            if self.closing.is_set():
                raise OSError('返回清理中，禁止发送报文')
            return self.raw.send(data)

    def __getattr__(self, name):
        return getattr(self.raw, name)


class AppPage:
    def __init__(self, root, source, dpg, parent, leave, *, transport_factory=None, mechanism='gimbal',
                 fixed_mechanism=False):
        self.closing = threading.Event()
        self.lock = threading.RLock()
        self.transports = []
        self.sessions = []
        self.pending = None
        self.next_refresh = 0
        self.modules = SourceModules([source / 'robots/sentinel/host',
                                      source / 'boards/sentinel_f407/tools'])
        try:
            module = self.modules.load('sentinel_console')
            light_palette(module)
            setattr(module, 'dpg', EmbeddedDPG(dpg, parent))
            page = self

            def tracked(factory):
                def create(*args, **kwargs):
                    with page.lock:
                        if page.closing.is_set():
                            raise OSError('页面正在释放资源')
                        raw = factory(*args, **kwargs)
                        page.transports.append(raw)
                        return TransportGate(raw, page.closing, page.lock)
                return create
            module.wire.SerialTransport = tracked(transport_factory or module.wire.SerialTransport)
            module.wire.ProcessTransport = tracked(module.wire.ProcessTransport)

            class TrackedSession(module.MaintenanceSession):
                def __init__(self, peer, mechanism):
                    # Retain the peer even when the handshake or constructor fails.
                    self.peer = peer
                    page.sessions.append(self)
                    super().__init__(peer, mechanism)
            setattr(module, 'MaintenanceSession', TrackedSession)

            class EmbeddedConsole(module.Console):
                def scan_ports(self):
                    from serial.tools import list_ports
                    ports = list(list_ports.comports())
                    selected = dpg.get_value('port')
                    dpg.configure_item('port', items=[p.device for p in ports])
                    dpg.set_value('port', selected)
                    self.message('可用串口: ' + ', '.join(f'{p.device} {p.description}' for p in ports))

                def connect(self):
                    if not page.released():
                        raise ValueError('之前的 APP 连接资源尚未释放，请先返回并完成清理')
                    super().connect()

                def disconnect(self):
                    if not page.closing.is_set():
                        super().disconnect()

                def invoke(self, action):
                    if not page.closing.is_set():
                        super().invoke(action)

            self.view = EmbeddedConsole(backend=source / 'build/sentinel-app-host/ec_sentinel_app_host',
                                        log_dir=root / 'build/sentinel-console-logs', mechanism=mechanism)
            if fixed_mechanism:
                dpg.configure_item('part', enabled=False)
            self.view.scan_ports()
        except Exception:
            self.modules.close()
            raise

    @property
    def needs_confirmation(self):
        return bool(self.view.session or self.view.job or self.transports)

    def tick(self):
        if not self.closing.is_set() and time.monotonic() >= self.next_refresh:
            self.view.refresh()
            self.next_refresh = time.monotonic() + .1

    def prepare_close(self):
        self.closing.set()

    def close(self):
        self.prepare_close()
        # Wait for a currently executing send/open. Never call the legacy stop,
        # disconnect, MaintenanceSession.close, or release methods here.
        with self.lock:
            pass
        if self.view.session and self.view.session not in self.sessions:
            self.sessions.append(self.view.session)
        if self.view.job:
            self.pending = self.view.job[1]
            self.view.job = None
        if self.pending and not self.pending.cancel():
            try:
                result = self.pending.result(timeout=3)
            except FutureTimeout as error:
                if not self.pending.done():
                    raise RuntimeError('连接/后台操作尚未结束，禁止离开；等待后重试') from error
                result = None  # A completed handshake may itself have raised TimeoutError.
            except Exception:
                result = None  # Transport handles are tracked even on failed connection.
            if hasattr(result, 'peer') and result not in self.sessions:
                self.sessions.append(result)
        errors = []
        for session in self.sessions:
            try:
                session.peer.close()  # Thread/transport close only; no AUTHORIZE/PERMIT.
            except Exception as error:
                errors.append(str(error))
        for transport in self.transports:
            try:
                transport.close()
            except Exception as error:
                errors.append(str(error))
        self.view.worker.shutdown(wait=True, cancel_futures=True)
        if errors or not self.released():
            raise RuntimeError('; '.join(errors) or 'APP 串口、管道或后台线程仍未释放')

    def released(self):
        if self.pending and not self.pending.done():
            return False
        if any(session.peer._thread.is_alive() for session in self.sessions):
            return False
        for transport in self.transports:
            if hasattr(transport, '_serial') and transport._serial.is_open:
                return False
            if hasattr(transport, '_process') and (
                    transport._process.poll() is None or transport._reader.is_alive()):
                return False
        return True

    def dispose(self):
        self.modules.close()

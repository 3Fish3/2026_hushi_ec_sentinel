"""C板快速刷写工具. 启动界面不访问串口、SWD或电机."""
import argparse
from dataclasses import fields
from pathlib import Path
import queue
import subprocess
import threading
import tkinter as tk
from tkinter import filedialog, ttk
from tkinter.scrolledtext import ScrolledText
import firmware_tool_model as model

class Window:
    def __init__(self, root_path=model.ROOT):
        self.repo = Path(root_path).resolve()
        self.root = tk.Tk()
        self.root.title('C板固件工具 / 构建 · 刷写 · 上位机对应')
        self.root.geometry('1080x780')
        self.root.minsize(880, 650)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('Title.TLabel', font=('Microsoft YaHei UI', 14, 'bold'))
        style.configure('Note.TLabel', foreground='#596778')
        self.queue = queue.SimpleQueue()
        self.busy = False
        self.controls = []
        self.settings = model.Settings.discover(self.repo)
        self.selected = tk.StringVar(value=model.PROFILES[0].title)
        self.elf = tk.StringVar()
        self.port = tk.StringVar(value='COM3')
        self.status = tk.StringVar(value='就绪. 选择用途后构建或检查已有固件; 本工具不会自动试动电机.')
        self.detail = tk.StringVar()
        self.artifact = tk.StringVar()
        page = ttk.Frame(self.root, padding=14)
        page.pack(fill='both', expand=True)
        page.columnconfigure(0, weight=1)
        page.rowconfigure(6, weight=1)
        ttk.Label(page, text='C板快速刷写', style='Title.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Label(page, text=str(self.repo), style='Note.TLabel').grid(row=1, column=0, sticky='w', pady=(2, 10))
        choose = ttk.LabelFrame(page, text='1. 固件用途与对应上位机', padding=10)
        choose.grid(row=2, column=0, sticky='ew')
        choose.columnconfigure(1, weight=1)
        self.selector = ttk.Combobox(choose, textvariable=self.selected, values=[p.title for p in model.PROFILES], state='readonly', width=28)
        self.selector.grid(row=0, column=0, sticky='w')
        self.selector.bind('<<ComboboxSelected>>', lambda _: self.select_profile())
        self.controls.append((self.selector, 'readonly'))
        link = ttk.Frame(choose)
        link.grid(row=0, column=1, sticky='e')
        ttk.Label(link, text='串口').pack(side='left', padx=(10, 4))
        entry = ttk.Entry(link, textvariable=self.port, width=9)
        entry.pack(side='left')
        self.controls.append((entry, 'normal'))
        self.button(link, '启动对应上位机', self.open_host).pack(side='left', padx=(8, 0))
        ttk.Label(choose, textvariable=self.detail, wraplength=940, justify='left').grid(row=1, column=0, columnspan=2, sticky='ew', pady=(10, 0))
        image = ttk.LabelFrame(page, text='2. 固件文件 (ELF, 自动生成并审计BIN)', padding=10)
        image.grid(row=3, column=0, sticky='ew', pady=10)
        image.columnconfigure(0, weight=1)
        entry = ttk.Entry(image, textvariable=self.elf)
        entry.grid(row=0, column=0, sticky='ew')
        self.controls.append((entry, 'normal'))
        self.button(image, '选择ELF', self.choose_elf).grid(row=0, column=1, padx=(8, 0))
        ttk.Label(image, textvariable=self.artifact, style='Note.TLabel', wraplength=930).grid(row=1, column=0, columnspan=2, sticky='w', pady=(6, 0))
        actions = ttk.Frame(page)
        actions.grid(row=4, column=0, sticky='ew')
        for title, action in [('检查固件', 'inspect'), ('仅构建', 'build'), ('烧录已有固件', 'flash'), ('构建并烧录', 'build_flash')]:
            self.button(actions, title, lambda a=action: self.start(a)).pack(side='left', padx=(0, 8))
        self.button(actions, '工具路径', self.toggle_settings).pack(side='right')
        self.progress = ttk.Progressbar(page, mode='indeterminate')
        self.progress.grid(row=5, column=0, sticky='ew', pady=8)
        self.log = ScrolledText(page, height=14, state='disabled', font=('Consolas', 9), wrap='word')
        self.log.grid(row=6, column=0, sticky='nsew')
        self.options = ttk.LabelFrame(page, text='工具路径 / 自动发现后可修改, 保存于build/', padding=8)
        self.variables = {}
        self.options.columnconfigure(1, weight=1)
        names = dict(cmake='CMake', ninja='Ninja', arm_root='ARM工具链目录', sdk_root='STM32CubeF4目录', openocd='OpenOCD', scripts='OpenOCD scripts目录', python='项目Python', pwsh='PowerShell 7')
        for row, field in enumerate(fields(model.Settings)):
            variable = tk.StringVar(value=getattr(self.settings, field.name))
            self.variables[field.name] = variable
            ttk.Label(self.options, text=names[field.name]).grid(row=row, column=0, sticky='w', padx=(0, 8))
            entry = ttk.Entry(self.options, textvariable=variable)
            entry.grid(row=row, column=1, sticky='ew', pady=2)
            self.controls.append((entry, 'normal'))
            self.button(self.options, '...', lambda key=field.name: self.choose_tool(key)).grid(row=row, column=2, padx=4)
        ttk.Label(page, textvariable=self.status, wraplength=980).grid(row=8, column=0, sticky='ew', pady=(10, 0))
        ttk.Label(page, text='烧录前撤销目标并断开控制会话. 只接一个ST-Link; 无Flash备份/整片擦除/Option Bytes修改. 物理断电手段仍保留.',
            wraplength=980, style='Note.TLabel').grid(row=9, column=0, sticky='ew', pady=(6, 0))
        self.elf.trace_add('write', self.file_changed)
        self.select_profile()
        self.root.after(80, self.poll)

    def button(self, parent, title, callback):
        button = ttk.Button(parent, text=title, command=callback)
        self.controls.append((button, 'normal'))
        return button

    def profile(self):
        return next(p for p in model.PROFILES if p.title == self.selected.get())

    def select_profile(self):
        p = self.profile()
        self.elf.set(str(model.default_elf(self.repo, p)))
        self.detail.set(f'{p.purpose}\n连接: C板丝印{p.port}  |  对应: {p.host}')
        self.artifact.set('尚未审计; 文件不存在时请先构建. 烧录总是重新审计所选文件.')

    def file_changed(self, *_):
        self.artifact.set('文件已变更, 尚未审计; 烧录前会重新核对所选镜像.')
        if not self.busy:
            self.status.set('请选择检查、构建或烧录动作. 尚未访问硬件.')

    def choose_elf(self):
        path = filedialog.askopenfilename(parent=self.root, title='选择与用途匹配的ELF', filetypes=[('ARM ELF', '*.elf')], initialdir=self.repo / 'build')
        if path:
            self.elf.set(path)
            self.artifact.set('已选择文件, 尚未审计. 旧ID7四电机镜像不可用于当前台架.')

    def toggle_settings(self):
        if self.options.winfo_manager():
            self.options.grid_remove()
        else:
            self.options.grid(row=7, column=0, sticky='ew', pady=6)

    def choose_tool(self, key):
        folder = key in ('arm_root', 'sdk_root', 'scripts')
        value = filedialog.askdirectory(parent=self.root) if folder else filedialog.askopenfilename(parent=self.root)
        if value:
            self.variables[key].set(value)

    def current_settings(self):
        return model.Settings(**{key: value.get().strip() for key, value in self.variables.items()})

    def start(self, action):
        if self.busy:
            return
        profile, settings = self.profile(), self.current_settings()
        elf = Path(self.elf.get()).expanduser()
        self.busy = True
        for widget, _ in self.controls:
            widget.configure(state='disabled')
        self.progress.start(12)
        self.status.set({'inspect': '检查固件中, 不访问硬件', 'build': '构建中, 不访问硬件',
            'flash': '检查并烧录所选固件...', 'build_flash': '构建并烧录中...'}[action])
        self.worker = threading.Thread(target=self.work, args=(action, profile, settings, elf), daemon=False)
        self.worker.start()

    def work(self, action, profile, settings, elf):
        work = None
        try:
            work = model.new_run(self.repo)
            settings.save(self.repo)
            with (work / 'run.log').open('x', encoding='utf-8') as stream:
                def log(line):
                    stream.write(line + '\n')
                    stream.flush()
                    self.queue.put(('log', line))
                log(f'任务: {action} / {profile.title}')
                log(f'日志: {work}')
                try:
                    if action in ('build', 'build_flash'):
                        elf = model.build_firmware(self.repo, profile, settings, log)
                        self.queue.put(('elf', str(elf)))
                    image = model.prepare_image(self.repo, profile, elf, settings, work, log)
                    self.queue.put(('image', f'{image.label} | {image.size:,} B | 扇区0-{image.last_sector} | SHA256 {image.sha256}'))
                    if action in ('flash', 'build_flash'):
                        model.flash_image(image, settings, log)
                        result = '烧录、校验和复位完成; 尚未做电机动作验收. 请重新连接对应上位机.'
                    else:
                        result = '构建/审计完成, 未访问硬件.' if action == 'build' else '固件审计通过, 未访问硬件.'
                    log(result)
                    self.queue.put(('done', result))
                except Exception as error:
                    log(f'失败: {error}')
                    raise
        except Exception as error:
            location = f' 日志: {work}' if work else ''
            self.queue.put(('error', str(error) + location))

    def poll(self):
        for _ in range(200):
            try:
                kind, value = self.queue.get_nowait()
            except queue.Empty:
                break
            if kind == 'log':
                self.log.configure(state='normal')
                self.log.insert('end', value + '\n')
                if int(self.log.index('end-1c').split('.')[0]) > 600:
                    self.log.delete('1.0', 'end-500lines')
                self.log.see('end')
                self.log.configure(state='disabled')
            elif kind == 'elf':
                self.elf.set(value)
            elif kind == 'image':
                self.artifact.set(value)
            elif kind in ('done', 'error'):
                self.busy = False
                self.progress.stop()
                for widget, state in self.controls:
                    widget.configure(state=state)
                prefix = '失败, 不自动重试: ' if kind == 'error' else ''
                self.status.set(prefix + value)
        self.root.after(80, self.poll)

    def open_host(self):
        try:
            settings = self.current_settings()
            args = model.host_command(self.repo, self.profile(), settings, self.port.get())
            subprocess.Popen(args, cwd=self.repo)
            self.status.set('已启动对应上位机, 不自动申请控制权或下发目标.')
        except Exception as error:
            self.status.set(f'启动失败: {error}')

    def close(self):
        if self.busy:
            self.status.set('任务仍在运行, 请等待结束后关闭; 不在烧录中途强制中断.')
            return
        self.root.destroy()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=model.ROOT, help='仓库目录, 默认从脚本位置解析')
    args = parser.parse_args()
    window = Window(args.root)
    window.root.mainloop()

if __name__ == '__main__':
    main()

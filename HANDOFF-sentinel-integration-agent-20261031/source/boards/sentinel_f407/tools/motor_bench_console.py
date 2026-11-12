"""Dear PyGui GM6020 S5/v6台架控制台; 启动/打开文件均不自动连接或输出."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import queue
import time

import dearpygui.dearpygui as dpg
from serial.tools import list_ports
import motor_bench as bench
import motor_bench_trace as trace
from motor_bench_console_model import FIELDS, LiveModel, Recording, same_settings, status_values, save_profile, load_profile, velocity_shortcut, position_shortcut

BLUE = (88, 178, 255)
GREEN = (105, 213, 166)
AMBER = (242, 191, 99)
RED = (245, 112, 119)
MUTED = (150, 165, 186)
MODES = ['电流', '速度', '位置']
UNITS = ['A', 'rad/s', 'rad']


class Console:
    def __init__(self, port='COM3', log_dir=Path('build/motor-bench-logs'), session_factory=bench.Session, metric_font=None):
        self.log_dir, self.session_factory = Path(log_dir), session_factory
        self.session = None
        self.events = queue.Queue(maxsize=64)
        self.live = LiveModel()
        self.pending = False
        self.stopping = False
        self.recordings = {}
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='bench-files')
        self.jobs = []
        self.next_refresh = 0
        self.last_plot_time = None
        self.last_size = None
        self.port = port
        self.metric_font = metric_font
        self.build()

    def message(self, text, error=False):
        dpg.set_value('notice', str(text))
        dpg.configure_item('notice', color=RED if error else MUTED)

    def invoke(self, action):
        try:
            action()
        except (ValueError, TypeError, KeyError, OSError, OverflowError, queue.Full) as exc:
            self.message(exc, True)

    def button(self, label, action, **kwargs):
        return dpg.add_button(label=label, callback=lambda: self.invoke(action), **kwargs)

    def build(self):
        with dpg.theme() as theme:
            with dpg.theme_component(dpg.mvAll):
                for key, color in ((dpg.mvThemeCol_WindowBg, (19, 24, 32)),
                                   (dpg.mvThemeCol_ChildBg, (25, 32, 43)),
                                   (dpg.mvThemeCol_FrameBg, (36, 46, 61)),
                                   (dpg.mvThemeCol_Button, (40, 66, 91)),
                                   (dpg.mvThemeCol_ButtonHovered, (49, 95, 129)),
                                   (dpg.mvThemeCol_Header, (36, 61, 83)),
                                   (dpg.mvThemeCol_Text, (226, 233, 242))):
                    dpg.add_theme_color(key, color)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
                dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 6)
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 8, 5)
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 9, 8)
        dpg.bind_theme(theme)
        with dpg.theme() as stop_theme:
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (139, 48, 57))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (181, 57, 67))
        with dpg.window(tag='main'):
            with dpg.group(horizontal=True):
                dpg.add_text('GM6020 / 台架控制台', color=BLUE)
                dpg.add_combo([self.port], default_value=self.port, tag='port', width=100)
                self.button('刷新串口', self.scan_ports)
                self.button('连接 · 仅查询', self.connect, tag='connect')
                self.button('断开', self.disconnect, tag='disconnect')
            dpg.add_text('离线 · 输出状态未知', tag='link', color=AMBER)
            dpg.add_text('零电流不等于刹停 · 紧急处置使用台架物理断电 · 仅S5/v6, 不适用整车', color=MUTED)
            with dpg.group(horizontal=True):
                for key, label in [('target', 'MCU当前目标'), ('measured', '实际运动'), ('error', '目标 − 测量'), ('current', '命令电流')]:
                    with dpg.child_window(tag='card_'+key, width=270, height=86, no_scrollbar=True):
                        dpg.add_text(label, tag='title_'+key, color=MUTED)
                        dpg.add_text('—', tag='value_'+key, color=BLUE)
                        if self.metric_font: dpg.bind_item_font('value_'+key,self.metric_font)
            dpg.add_text('1s平均速度: — · 原生RPM状态 / 接收时间加权, 需完整1s有效窗口',tag='average_speed',color=GREEN)
            dpg.add_text('反馈源: — · 实际速度卡显示原生RPM, 不伪称滤波后的内环测量',tag='feedback_source',color=MUTED)
            with dpg.group(horizontal=True):
                with dpg.child_window(tag='controls', width=365, height=540):
                    with dpg.group(horizontal=True):
                        dpg.add_text('操作 / 参数草稿', color=BLUE)
                        self.button('停止输出 [F8]', self.stop, tag='stop')
                        dpg.bind_item_theme('stop', stop_theme)
                    dpg.add_combo(MODES, default_value='速度', tag='mode', width=140, callback=lambda: self.mode_changed())
                    for key, label, unit in FIELDS[:2]:
                        dpg.add_input_text(label=f'{label} {unit}', tag='edit_'+key, default_value='0', width=135)
                    dpg.add_text('目标单位: rad/s', tag='target_unit', color=MUTED)
                    with dpg.group(horizontal=True):
                        self.button('开始本次测试', self.start, tag='start')
                        self.button('建立位置参考', self.reference, tag='reference')
                    dpg.add_input_text(label='目标斜坡 s · 0=阶跃', tag='ramp', default_value='0', width=135)
                    self.button('更新当前模式目标', self.update_target, tag='update_target')
                    with dpg.group(horizontal=True,tag='speed_shortcuts'):
                        for label, delta in [('-2',-2),('-0.5',-.5),('+0.5',.5),('+2',2)]:
                            self.button(label,lambda delta=delta:self.quick_speed(delta),tag='quick_'+label)
                        self.button('反转',lambda:self.quick_speed(reverse=True),tag='quick_reverse')
                    dpg.add_text('速度快捷: 按输入框目标运算, 立即阶跃更新',tag='quick_hint',color=MUTED,wrap=330)
                    with dpg.group(horizontal=True,tag='position_shortcuts'):
                        for label,delta in [('-0.1',-.1),('-0.01',-.01),('+0.01',.01),('+0.1',.1)]:
                            self.button(label,lambda delta=delta:self.quick_position(delta),tag='position_quick_'+label)
                    dpg.add_text('位置快捷(rad): 按目标增减, 立即更新; 不重建参考',tag='position_quick_hint',color=MUTED,wrap=330)
                    dpg.add_separator()
                    self.button('用MCU回读值填入草稿', self.copy_actual, tag='copy_actual')
                    dpg.add_text('参数未载入', tag='dirty', color=AMBER, wrap=320)
                    dpg.add_text('草稿在下次启动采用; 运行中只更新目标.', color=MUTED, wrap=325)
                    for key, label, unit in FIELDS[4:7]:
                        dpg.add_input_text(label=f'{label} {unit}', tag='edit_'+key, width=125)
                    dpg.add_checkbox(label='编码器5点因果测速', tag='encoder')
                    with dpg.collapsing_header(label='电流 / 速度目标上限'):
                        for key, label, unit in FIELDS[2:4]:
                            dpg.add_input_text(label=f'{label} {unit}', tag='edit_'+key, width=125)
                    with dpg.collapsing_header(label='D项 / 反馈滤波'):
                        for key, label, unit in FIELDS[7:]:
                            dpg.add_input_text(label=f'{label} {unit}', tag='edit_'+key, width=135)
                    dpg.add_input_text(label='工况备注', tag='note', width=220)
                    with dpg.collapsing_header(label='候选参数文件'):
                        dpg.add_input_text(tag='profile_path', width=-1, default_value=str(self.log_dir/'candidate.json'))
                        self.button('保存草稿候选', self.save_draft)
                        self.button('打开候选 · 不下发', lambda: self.pick('profile'))
                        dpg.add_text('候选不代表调稳; 不覆盖已有同名文件.', color=MUTED, wrap=310)
                    with dpg.collapsing_header(label='原有诊断操作'):
                        self.button('停发心跳 · 时效测试', self.silence)
                with dpg.child_window(tag='workspace', width=750, height=540):
                    with dpg.tab_bar(tag='tabs'):
                        with dpg.tab(label='实时观测', tag='live_tab'):
                            dpg.add_text('低频状态快照, 非1 kHz记录; 横轴为主机接收时间, 断样不补点.', color=MUTED, wrap=700)
                            dpg.add_checkbox(label='自动适配坐标 · 取消后可自由缩放查看历史',tag='live_autofit',default_value=True)
                            self.plot('live_speed', '速度 [rad/s]', [('target', '目标'), ('rpm', '原生RPM'), ('estimate', '编码器估计 · 滤前')], 210)
                            self.plot('live_current', '命令电流 [A] · 非实测力矩', [('command', 'MCU命令')], 145)
                            self.plot('live_position', '位置 [rad]', [('absolute', '单圈绝对角'), ('relative', '有参考的相对角'), ('target', '位置目标')], 145)
                        with dpg.tab(label='高频记录 / 回放', tag='replay_tab'):
                            with dpg.group(horizontal=True):
                                dpg.add_combo(['完整PID · 1024周期', '精简长窗 · 2730周期'], default_value='完整PID · 1024周期', tag='layout', width=205)
                                dpg.add_checkbox(label='等待启动/更新触发', tag='trigger')
                            with dpg.group(horizontal=True):
                                self.button('采集波形', self.capture, tag='capture')
                                self.button('结束采样', self.freeze, tag='freeze')
                                self.button('打开A', lambda: self.pick('A'))
                                self.button('打开B对照', lambda: self.pick('B'))
                                self.button('清除B', self.clear_b)
                            dpg.add_text('采集空闲 · 采集/下载不改变控制目标',tag='capture_status',color=AMBER)
                            dpg.add_text('尚未打开记录', tag='record_info', wrap=700, color=MUTED)
                            with dpg.group(horizontal=True):
                                self.button('适配全窗', self.fit_replay)
                                self.button('导出A CSV / PNG', self.export_a)
                            with dpg.group(horizontal=True):
                                dpg.add_input_float(label='选区始 s', tag='range_start', width=90, step=0, callback=lambda: self.statistics())
                                dpg.add_input_float(label='末 s', tag='range_end', width=90, step=0, default_value=1, callback=lambda: self.statistics())
                            dpg.add_text('—', tag='statistics', wrap=700, color=GREEN)
                            speed = [('rpm', '原生RPM'), ('used', '内环测量'), ('target', '速度目标')]
                            self.plot('replay_speed', '速度 [rad/s] · 每个文件从自身窗口起点对齐', [(slot+'_'+key, slot+' '+label) for slot in 'AB' for key,label in speed], 220)
                            for key, color in [('range_start', GREEN), ('range_end', AMBER)]:
                                dpg.add_drag_line(tag='drag_'+key, label='选区'+('始' if key.endswith('start') else '末'), parent='replay_speed', color=color,
                                                  callback=lambda s,a,u: self.drag_range(u,a), user_data=key)
                            self.plot('replay_current', '命令 / PID [A] · 长窗无PID分项', [(slot+'_'+key, slot+' '+label) for slot in 'AB' for key,label in [('command','命令'),('p','P'),('i','I'),('d','D')]], 180)
                            self.plot('replay_position', '位置 [rad]', [(slot+'_'+key, slot+' '+label) for slot in 'AB' for key,label in [('encoder','单圈角'),('position','相对角'),('target','位置目标')]], 180)
                        with dpg.tab(label='MCU回读 / 诊断'):
                            dpg.add_text('实际值来自MCU, 不从输入框推断生效状态.', color=MUTED)
                            with dpg.table(header_row=True, resizable=True):
                                dpg.add_table_column(label='参数 / 单位');dpg.add_table_column(label='MCU实际值')
                                for key,label,unit in FIELDS:
                                    with dpg.table_row():
                                        dpg.add_text(f'{label} {unit}');dpg.add_text('—',tag='actual_'+key)
                            dpg.add_text('—', tag='diagnostics', wrap=700)
            dpg.add_text('就绪 · 不自动连接. 可直接打开 .btrace 离线查看.', tag='notice', color=MUTED, wrap=1100)
        with dpg.file_dialog(tag='file_dialog', show=False, width=850, height=500, callback=self.file_selected):
            dpg.add_file_extension('.btrace');dpg.add_file_extension('.json')
        with dpg.handler_registry():
            dpg.add_key_press_handler(key=dpg.mvKey_F8, callback=lambda: self.stop())

    def plot(self, tag, title, series, height):
        with dpg.plot(tag=tag, label=title, width=-1, height=height):
            dpg.add_plot_legend(horizontal=True, outside=False, location=dpg.mvPlot_Location_NorthEast)
            dpg.add_plot_axis(dpg.mvXAxis, tag=tag+'_x', label='s')
            with dpg.plot_axis(dpg.mvYAxis, tag=tag+'_y'):
                for key,label in series:
                    dpg.add_line_series([], [], tag=tag+'_'+key, label=label, show=not key.startswith('B_'))

    def scan_ports(self):
        ports = [p.device for p in list_ports.comports()]
        dpg.configure_item('port', items=ports or [dpg.get_value('port')])
        self.message('串口列表已刷新; 不自动连接')

    def connect(self):
        if self.session is not None and self.session.is_alive():
            return
        self.log_dir.mkdir(parents=True, exist_ok=True)
        path = self.log_dir/f'gm6020-{time.time_ns()}.jsonl'
        bench.Session.drain(self.events)
        self.live = LiveModel();self.pending = False;self.stopping = False
        self.session = self.session_factory(dpg.get_value('port'), path, self.events)
        self.session.start()
        self.message(f'连接仅查询 · 日志: {path}')

    def disconnect(self):
        if self.session:
            self.session.stop_motion();self.session.quit_event.set()
            self.stopping = True
        self.message('已请求停止并断开; 请求不等于机械已停止')

    def stop(self):
        if self.session and self.session.is_alive():
            self.session.stop_motion();self.stopping = True
            self.message('停止请求已排队; 等待MCU回读, 不代表刹停')

    def available(self):
        if not self.session or not self.session.is_alive() or not self.live.fresh(time.monotonic()):
            raise ValueError('没有新鲜的S5/v6连接')
        return self.live.last

    def values(self):
        return [float(dpg.get_value('edit_'+key)) for key,_,_ in FIELDS]+[int(dpg.get_value('encoder'))]

    def mode(self):
        return MODES.index(dpg.get_value('mode'))+1

    def mode_changed(self):
        # 模式只编辑草稿, 不发停止或启动; 当前运行模式以MCU回读为准.
        dpg.set_value('target_unit', '目标单位: '+UNITS[self.mode()-1])
        self.message('模式仅修改草稿; 运行中不能跨模式更新目标')

    def start(self):
        d = self.available()
        if self.pending or self.stopping or d['active']:
            raise ValueError('已有运行/待确认请求, 不重复启动')
        if not d['feedback_valid'] or self.mode()==3 and not d['position_valid']:
            raise ValueError('反馈或位置参考无效')
        self.session.start_motion(bench.settings_payload(self.mode(), self.values()))
        self.pending = True
        self.message('启动已发送; 是否采用以MCU状态为准')

    def update_target(self):
        d = self.available()
        if not d['active'] or self.mode()!=d['mode'] or self.stopping:
            raise ValueError('只能更新当前正在运行模式的目标')
        target = float(dpg.get_value('edit_target'))
        ramp = float(dpg.get_value('ramp'))
        self.session.update_target(target, round(ramp*1e6))
        self.message('仅更新目标; 草稿中的PID/反馈源不会随目标更新采用')

    def quick_speed(self, delta=0, reverse=False):
        d = self.available()
        if not d['active'] or d['mode']!=2 or self.mode()!=2 or self.stopping:
            raise ValueError('速度快捷操作仅用于当前正在运行的速度模式')
        value = velocity_shortcut(float(dpg.get_value('edit_target')),delta,reverse,d['speed_limit'])
        self.session.update_target(value,0)
        # 请求入队成功后才修改草稿, 连点累加而不被滞后的MCU回读覆盖.
        dpg.set_value('edit_target',format(value,'.7g'))
        self.message(f'速度目标 {value:.3f} rad/s 已请求立即更新; 不改变PID/运行截止时间')

    def quick_position(self, delta):
        d = self.available()
        if not d['active'] or d['mode']!=3 or self.mode()!=3 or not d['position_valid'] or self.stopping:
            raise ValueError('位置快捷操作需要运行中的位置模式和有效位置参考')
        value = position_shortcut(float(dpg.get_value('edit_target')),delta)
        self.session.update_target(value,0)
        dpg.set_value('edit_target',format(value,'.7g'))
        self.message(f'位置目标 {value:.5f} rad 已请求立即更新; 不重置控制器/参考/运行截止时间')

    def reference(self):
        self.available();self.session.set_reference()
        self.message('请求以当前角度建立参考, 不使能、不写编码器零点')

    def silence(self):
        self.available();self.session.silence_event.set()
        self.message('已停发心跳, 仍查询; 原有命令时效生效, 不自动恢复')

    def fill(self, mode, values):
        dpg.set_value('mode', MODES[mode-1])
        for (key,_,_),value in zip(FIELDS,values):
            dpg.set_value('edit_'+key, format(value,'.7g'))
        dpg.set_value('encoder', bool(values[-1]));self.mode_changed()

    def copy_actual(self):
        d = self.available()
        self.fill(d['mode'], status_values(dict(d, target=d['target_final'])))
        self.message('回读值已填入草稿; 没有下发参数')

    def save_draft(self):
        path = Path(dpg.get_value('profile_path'))
        path.parent.mkdir(parents=True, exist_ok=True)
        save_profile(path, self.mode(), self.values(), dpg.get_value('note'))
        self.message(f'已保存未验收候选: {path}')

    def pick(self, slot):
        self.file_slot = slot
        if self.log_dir.is_dir(): dpg.configure_item('file_dialog',default_path=str(self.log_dir.resolve()))
        dpg.show_item('file_dialog')

    def file_selected(self, sender, data):
        self.invoke(lambda: self.open_file(data['file_path_name'], self.file_slot))

    def open_file(self, path, slot='A'):
        if slot == 'profile':
            data = load_profile(path)
            self.fill(data['mode'], data['values']);dpg.set_value('note',data.get('note',''))
            self.message('候选已载入草稿; 未下发, 不代表实测通过')
        else:
            self.submit_job(lambda: Recording(path), lambda record: self.show_record(record, slot))

    def submit_job(self, work, done):
        if len(self.jobs)>=2:
            raise ValueError('文件任务正在处理, 请稍后再试')
        self.jobs.append((self.worker.submit(work),done))

    def show_record(self, record, slot):
        self.recordings[slot] = record
        for plot, mapping in [('speed',{'rpm':'raw_velocity_rad_s','used':'velocity_used','target':'speed_target'}),
                              ('current',{'command':'command_a','p':'p_a','i':'i_a','d':'d_a'}),
                              ('position',{'encoder':'encoder_rad','position':'position','target':'position_target'})]:
            for key,field in mapping.items():
                series=record.series(field)
                dpg.set_value(f'replay_{plot}_{slot}_{key}', series)
                dpg.configure_item(f'replay_{plot}_{slot}_{key}',show=any(math.isfinite(v) for v in series[1]))
        lines=[]
        for name,r in self.recordings.items():
            s=r.metadata['start']
            lines.append(f'{name}: {r.path.name}\n  Kp/Ki/Kd={s["kp"]:.5g}/{s["ki"]:.5g}/{s.get("kd",0):.5g} · 上限{s["current_limit"]:.4g} A · '
                         f'{"编码器" if s.get("encoder_velocity") else "原生RPM"} · {len(r.rows)}周期 · {r.metadata.get("build_id", "未知固件")}')
        dpg.set_value('record_info','\n'.join(lines))
        dpg.set_value('range_start',0);dpg.set_value('range_end',max(r.rows[-1]['time_s'] for r in self.recordings.values()))
        self.statistics();self.fit_replay();dpg.set_value('tabs',dpg.get_alias_id('replay_tab'))
        self.message('记录已打开 · 仅离线分析, 不改实时控制')

    def clear_b(self):
        self.recordings.pop('B',None)
        for plot in ('speed','current','position'):
            for item in dpg.get_item_children('replay_'+plot+'_y',1):
                if dpg.get_item_alias(item).startswith('replay_'+plot+'_B_'):
                    dpg.set_value(item,[[],[]]);dpg.configure_item(item,show=False)
        if 'A' in self.recordings:
            self.show_record(self.recordings['A'],'A')

    def statistics(self):
        lo,hi = dpg.get_value('range_start'),dpg.get_value('range_end')
        for key in ('range_start','range_end'):
            dpg.set_value('drag_'+key,dpg.get_value(key))
        lines=[]
        for slot,r in self.recordings.items():
            m=r.summary(lo,hi)
            text=f'{slot}: 选区 {m["samples"]}点 / {m["duration_s"]:.3f}s'
            for key,label in [('velocity_error_rad_s','内环误差 rad/s'),('position_error_rad','位置误差 rad')]:
                if key in m:
                    e=m[key];text+=f' | {label}: RMS {e["rms"]:.4f}, p-p {e["peak_to_peak"]:.4f}'
            if m['samples']:
                text+=f' | 缺口{m["record_gaps"]}, CAN未接受{m["tx_not_accepted"]}'
            lines.append(text)
        dpg.set_value('statistics','\n'.join(lines) or '无记录; 精简长窗不提供内环PID误差')

    def drag_range(self, key, value):
        dpg.set_value(key,value);self.statistics()

    def fit_replay(self):
        for plot in ('speed','current','position'):
            for axis in ('x','y'):
                dpg.fit_axis_data(f'replay_{plot}_{axis}')

    def export_a(self):
        if 'A' not in self.recordings:
            raise ValueError('先打开A记录')
        path=self.recordings['A'].path
        self.submit_job(lambda: trace.export(path),lambda outputs:self.message('已导出: '+', '.join(outputs)))

    def capture(self):
        self.available()
        compact=dpg.get_value('layout').startswith('精简')
        self.session.start_capture(trace.IDENT_CAPACITY if compact else trace.CAPACITY,
                                   dpg.get_value('trigger'),dpg.get_value('note'),compact)
        self.message('已请求采集; 满窗冻结并下载, 不改变电机目标')

    def freeze(self):
        self.available();self.session.freeze_capture();self.message('已请求结束采样并下载; 不停止电机')

    def refresh(self):
        while True:
            try: kind,value=self.events.get_nowait()
            except queue.Empty: break
            if kind=='status':
                self.live.accept(value['data'],value['received_at'])
                if value['request'] in (0x21,0x23): self.pending=False
                if value['request']==0x23: self.stopping=False
            elif kind=='trace_saved':
                self.message('原始记录已保存: '+value)
                self.invoke(lambda: self.open_file(value))
            elif kind in ('error','note'):
                self.message(value,kind=='error')
            elif kind=='closed':
                self.pending=False;self.live.received_at=float('-inf')
                self.message('连接已关闭; 最后测量不是当前运动状态')
        remaining=[]
        for future,done in self.jobs:
            if future.done():
                try: done(future.result())
                except Exception as exc: self.message(f'文件处理失败: {exc}',True)
            else: remaining.append((future,done))
        self.jobs=remaining
        now=time.monotonic()
        alive=bool(self.session and self.session.is_alive())
        fresh=alive and self.live.fresh(now)
        d=self.live.last
        capture=self.session.capture if alive else None
        if capture:
            stage={'arm':'预置采集','wait':'采样 / 等待触发','freeze':'结束采样',
                   'start_meta':'读取起点元数据','end_meta':'读取终点元数据','data':'下载样本','done':'写入文件'}[capture.phase]
            count=len(capture.data)//capture.record.size
            dpg.set_value('capture_status',f'{stage} · 已下载{count}/{capture.count}周期; 控制心跳独立调度')
        else:
            dpg.set_value('capture_status','采集空闲 · 原始记录下载后自动打开, CSV/PNG按需导出')
        for key in ('kp','ki'):
            dpg.configure_item('edit_'+key,show=self.mode()!=1)
        dpg.configure_item('edit_position_kp',show=self.mode()==3)
        dpg.configure_item('encoder',show=self.mode()!=1)
        dpg.configure_item('connect',enabled=not alive)
        dpg.configure_item('disconnect',enabled=alive)
        for tag in ('reference','copy_actual','capture','freeze'):
            dpg.configure_item(tag,enabled=fresh)
        dpg.configure_item('start',enabled=bool(fresh and not d['active'] and d['feedback_valid'] and not self.pending and not self.stopping))
        dpg.configure_item('update_target',enabled=bool(fresh and d['active'] and self.mode()==d['mode'] and not self.stopping))
        quick=bool(fresh and d['active'] and d['mode']==2 and self.mode()==2 and not self.stopping)
        dpg.configure_item('speed_shortcuts',show=self.mode()==2)
        dpg.configure_item('quick_hint',show=self.mode()==2)
        for tag in ('quick_-2','quick_-0.5','quick_+0.5','quick_+2','quick_reverse'):
            dpg.configure_item(tag,enabled=quick)
        position_quick=bool(fresh and d['active'] and d['mode']==3 and self.mode()==3 and d['position_valid'] and not self.stopping)
        dpg.configure_item('position_shortcuts',show=self.mode()==3)
        dpg.configure_item('position_quick_hint',show=self.mode()==3)
        for tag in ('position_quick_-0.1','position_quick_-0.01','position_quick_+0.01','position_quick_+0.1'):
            dpg.configure_item(tag,enabled=position_quick)
        average=self.live.average_speed(now) if fresh else None
        dpg.set_value('average_speed',f'1s平均速度: {average:+.3f} rad/s · 原生RPM状态 / 接收时间加权' if average is not None else
                      '1s平均速度: — · 等待完整1s有效窗口, 断样/反馈失效后重新累计')
        for key in ('target','measured','error','current'):
            dpg.set_value('value_'+key,'—')
        if fresh:
            reason=bench.REASONS[d['reason']] if d['reason']<len(bench.REASONS) else str(d['reason'])
            result=bench.REASONS[d['result']] if d['result']<len(bench.REASONS) else str(d['result'])
            dpg.set_value('link',f'S5/v6 · CAN{d["bus"]} ID{d["device_id"]} · {MODES[d["mode"]-1]} · '
                          f'{"输出启用" if d["active"] else "输出禁用"} / {reason} · 请求结果: {result}')
            dpg.configure_item('link',color=GREEN if d['feedback_valid'] else AMBER)
            dpg.set_value('feedback_source',f'控制反馈源: {"编码器5点" if d["encoder_velocity"] else "原生RPM"} · alpha={d["feedback_alpha"]:.4g} · D τ={d["derivative_tau_s"]:.4g}s · '
                          '速度卡显示原生RPM, 实际滤后内环值见完整波形')
            dpg.set_value('value_target',f'{d["target"]:.3f} {UNITS[d["mode"]-1]}')
            dpg.set_value('value_current',f'{d["output_a"]:.3f} A')
            measured=d['relative_rad'] if d['mode']==3 else d['speed_rad_s']
            valid=d['position_valid'] if d['mode']==3 else d['feedback_valid']
            dpg.set_value('title_measured','相对位置' if d['mode']==3 else '实际速度 · 原生RPM')
            if valid:
                dpg.set_value('value_measured',f'{measured:.3f} '+('rad' if d['mode']==3 else 'rad/s'))
                if d['mode']!=1:
                    dpg.set_value('value_error',f'{d["target"]-measured:+.3f} '+UNITS[d['mode']-1])
            for key,_,_ in FIELDS:
                dpg.set_value('actual_'+key,format(d[key],'.7g'))
            dpg.set_value('diagnostics',f'反馈源: {"编码器5点" if d["encoder_velocity"] else "原生RPM"} (滤波后内环值请看完整波形)\n'
                          f'反馈年龄 {d["feedback_age_ms"]} ms · 编码器估计中心年龄 {d["estimated_velocity_age_ms"]} ms\n'
                          f'位置参考 {"有效" if d["position_valid"] else "未建立/失效"} · 参考代次 {d["reference_epoch"]} · 参数代次 {d["settings_epoch"]}\n'
                          f'上报电流 raw {d["effort_raw"]} · 温度 {d["temperature"]} °C · CAN未接受累计 {d["tx_errors"]}\n'
                          f'固件MCU时间 {d["ms"]} ms · 当前→最终目标 {d["target"]:.5g} → {d["target_final"]:.5g}')
            try: matching=same_settings(self.mode(),self.values(),d)
            except (ValueError,OverflowError): matching=False
            dpg.set_value('dirty','草稿与MCU回读一致' if matching else '草稿尚未应用 / 与MCU不同; 运行中仅更新目标')
        else:
            dpg.set_value('link','离线 / 状态过期 · 输出状态未知, 旧测量不作实时显示')
            dpg.configure_item('link',color=AMBER)
            dpg.set_value('feedback_source','反馈源: 未知 · 实时曲线仅保留历史, 不代表当前运动状态')
            dpg.set_value('dirty','无新鲜回读; 草稿不代表实际生效参数')
            dpg.set_value('diagnostics','没有新鲜的MCU状态')
            for key,_,_ in FIELDS: dpg.set_value('actual_'+key,'—')
        if self.live.received_at!=self.last_plot_time:
            self.last_plot_time=self.live.received_at
            for tag,key,options in [('live_speed_target','target',{'mode':2}),('live_speed_rpm','speed_rad_s',{'feedback':True}),
                                    ('live_speed_estimate','estimated_velocity_rad_s',{'feedback':True}),('live_current_command','output_a',{}),
                                    ('live_position_absolute','absolute_rad',{'feedback':True}),('live_position_relative','relative_rad',{'position':True}),
                                    ('live_position_target','target',{'mode':3})]:
                dpg.set_value(tag,self.live.series(key,**options))
            if dpg.get_value('live_autofit'):
                for plot in ('live_speed','live_current','live_position'):
                    dpg.fit_axis_data(plot+'_x');dpg.fit_axis_data(plot+'_y')
        size=(dpg.get_viewport_client_width(),dpg.get_viewport_client_height())
        if size!=self.last_size:
            self.last_size=size
            for key in ('target','measured','error','current'):
                dpg.configure_item('card_'+key,width=max(200,(size[0]-60)//4))
            height=max(280,size[1]-325)
            dpg.configure_item('controls',height=height)
            width=max(500,size[0]-405)
            dpg.configure_item('workspace',width=width,height=height)
            for tag in ('record_info','statistics','diagnostics'):
                dpg.configure_item(tag,wrap=width-35)
            for tag in ('notice','link','feedback_source','average_speed'):
                dpg.configure_item(tag,wrap=max(700,size[0]-40))
            # 图表总高随工作区扩展; 小窗口以滚动保住绘图区, 不压扁坐标轴.
            live_height=max(220,(height-160)//3)
            replay_height=max(240,(height-330)//3)
            for plot in ('speed','current','position'):
                dpg.configure_item('live_'+plot,height=live_height)
                dpg.configure_item('replay_'+plot,height=replay_height)

    def close(self):
        self.disconnect()
        if self.session:
            self.session.join(timeout=3)
        self.worker.shutdown(wait=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default='COM3')
    parser.add_argument('--log-dir',type=Path,default=Path('build/motor-bench-logs'))
    parser.add_argument('--trace',type=Path,help='离线打开A记录; 不连接串口')
    parser.add_argument('--font',type=Path,default=Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts/msyh.ttc')
    parser.add_argument('--font-size',type=int,default=18)
    args=parser.parse_args()
    dpg.create_context()
    try:
        # UI回调在主线程执行; 串口拥有者线程不接触DPG, 文件任务不阻塞心跳.
        dpg.configure_app(manual_callback_management=True)
        with dpg.font_registry():
            font=dpg.add_font(str(args.font),args.font_size)
            metric_font=dpg.add_font(str(args.font),round(args.font_size*1.65))
        dpg.bind_font(font)
        dpg.create_viewport(title='GM6020 Bench Console',width=1240,height=880,min_width=940,min_height=650)
        console=Console(args.port,args.log_dir,metric_font=metric_font)
        dpg.setup_dearpygui();dpg.show_viewport();dpg.set_primary_window('main',True)
        if args.trace: console.open_file(args.trace)
        try:
            while dpg.is_dearpygui_running():
                jobs=dpg.get_callback_queue()
                if jobs: dpg.run_callbacks(jobs)
                if time.monotonic()>=console.next_refresh:
                    console.refresh();console.next_refresh=time.monotonic()+.1
                dpg.render_dearpygui_frame()
        finally: console.close()
    finally: dpg.destroy_context()


if __name__=='__main__':
    main()

"""正式 APP v3 分机构维护 GUI; 不兼容 S5/capture, 不自动连接或申请控制."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
import math
import os
from pathlib import Path
import time

import dearpygui.dearpygui as dpg
import sentinel_client as wire
from sentinel_pitch_tuning import capture_view, plan_from_fields
from sentinel_console_model import (MaintenanceSession, LiveHistory, PARTS, PART_LABELS,
    DOMAIN_LABELS, parameter_label, physical_parameter, page_fresh)

ROOT = Path(__file__).resolve().parents[3]
GREEN, AMBER, RED, MUTED = (100,220,170), (245,192,98), (245,106,106), (150,165,186)


def plot_values(times, values):
    """Draw valid samples even when every sample is separated by a stale gap."""
    line_x, line_y, point_x, point_y = [], [], [], []
    previous = None
    for stamp, value in zip(times, values, strict=True):
        if not (math.isfinite(stamp) and math.isfinite(value)):
            previous = None
            continue
        point_x.append(stamp)
        point_y.append(value)
        if previous is not None:
            line_x.extend((previous[0], stamp))
            line_y.extend((previous[1], value))
        previous = (stamp, value)
    return [line_x, line_y], [point_x, point_y]


def visible_limits(times, series, low, high):
    values = [value for row in series for stamp, value in zip(times, row, strict=True)
              if low <= stamp <= high and math.isfinite(value)]
    if not values:
        return None
    bottom, top = min(values), max(values)
    middle = (bottom + top) / 2
    radius = max(top - bottom, .02) * .6
    return middle - radius, middle + radius


class Console:
    def __init__(self, *, backend=None, serial='', mechanism='gimbal', log_dir=None):
        candidates = (ROOT/'build/host-tests/ec_sentinel_app_host',
                      ROOT/'build/sentinel-app-host/Release/ec_sentinel_app_host.exe')
        self.backend = Path(backend or next((p for p in candidates if p.is_file()), candidates[-1]))
        self.log_dir = Path(log_dir or ROOT/'build/sentinel-console-logs')
        self.session = None
        self.history = LiveHistory()
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='app-console-io')
        self.job = None
        self.poll = None
        self.next_poll = 0
        self.plot_following = True
        self.notes = []
        self.param_ids = []
        self.assist_seen_report = None
        self.assist_fields = {}
        self.focus = PARTS[mechanism][0]
        self.build(serial, mechanism)

    def message(self, text, error=False):
        self.notes.append(f'{time.strftime("%H:%M:%S")} {text}')
        self.notes = self.notes[-40:]
        dpg.set_value('message', str(text)); dpg.configure_item('message', color=RED if error else AMBER)
        dpg.set_value('events', chr(10).join(self.notes))

    def invoke(self, action):
        try:
            action()
        except Exception as error:
            self.message(str(error), True)

    def button(self, text, action, **kwargs):
        return dpg.add_button(label=text, callback=lambda:self.invoke(action), **kwargs)

    def build(self, serial, mechanism):
        with dpg.theme(tag='draft_theme'):
            with dpg.theme_component(dpg.mvInputText):
                dpg.add_theme_color(dpg.mvThemeCol_Text,(115,183,255))
        with dpg.theme(tag='cancel_theme'):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button,(119,49,49))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,(160,65,65))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,(190,75,75))
        with dpg.window(tag='main', label='正式 APP 分机构维护'):
            dpg.add_text('SENTINEL / APP v3 维护控制台', color=GREEN)
            with dpg.group(horizontal=True):
                dpg.add_combo(['离线仿真', 'Pitch辨识仿真', 'APP串口'], default_value='APP串口' if serial else '离线仿真', tag='link', width=120)
                dpg.add_input_text(default_value=serial, hint='USART1 / 丝印UART2', tag='port', width=185)
                dpg.add_combo(list(PART_LABELS.values()), default_value=PART_LABELS[mechanism], tag='part',
                              callback=lambda:self.scope_changed(), width=125)
                self.button('连接 / HELLO', self.connect, tag='connect')
                self.button('断开', self.disconnect, tag='disconnect', enabled=False)
                self.button('扫描串口', self.scan_ports)
            dpg.add_text('未连接 · 参数和目标均不自动发送', tag='connection', color=MUTED)
            dpg.add_text('—', tag='authority')
            dpg.add_text('—', tag='ack', color=AMBER)
            dpg.add_text('当前关节角: —    当前速度: —', tag='joint_readout', color=GREEN)
            dpg.add_separator()
            with dpg.group(horizontal=True):
                with dpg.child_window(tag='operations', width=375, height=-66):
                    dpg.add_text('1  控制权', color=GREEN)
                    with dpg.group(horizontal=True):
                        self.button('申请维护', lambda:self.io('申请维护', lambda:self.require().authorize()), tag='manual_authorize')
                        self.button('释放维护', self.release)
                    dpg.add_text('原来源先释放; MCU 保留仲裁决定权', color=MUTED, wrap=345)
                    self.button('撤销目标  [F8]', self.stop, width=340,tag='cancel_targets')
                    dpg.bind_item_theme('cancel_targets','cancel_theme')
                    dpg.add_text('撤销不是硬件急停. 现场物理硬切保持可用.', color=AMBER, wrap=345)
                    dpg.add_separator()
                    dpg.add_text('2  原子目标草稿', color=GREEN)
                    dpg.add_text('角度是当前参考下的绝对角, 速度用rad/s. 未勾选域撤销旧目标; 输入不发送.', color=MUTED, wrap=345)
                    with dpg.table(header_row=False,policy=dpg.mvTable_SizingFixedFit):
                        for width in (24,85,135,50):
                            dpg.add_table_column(width_fixed=True,init_width_or_weight=width)
                        for domain in wire.DOMAINS:
                            with dpg.table_row(tag='row_'+domain):
                                dpg.add_checkbox(tag='enable_'+domain)
                                dpg.add_text(DOMAIN_LABELS[domain], tag='label_'+domain)
                                dpg.add_input_text(tag='target_'+domain, hint='目标', width=125)
                                dpg.bind_item_theme('target_'+domain,'draft_theme')
                                dpg.add_text('rad' if domain in PARTS['gimbal'] else 'rad/s')
                    with dpg.group(tag='yaw_frame_row', horizontal=True):
                        dpg.add_text('主 yaw 参考')
                        dpg.add_combo(list(wire.YAW_FRAMES), default_value='joint', tag='frame', width=170)
                    with dpg.group(tag='twist_row'):
                        dpg.add_checkbox(label='改用 base_link twist 草稿', tag='use_twist')
                        for key, unit in (('vx','m/s'), ('vy','m/s'), ('wz','rad/s')):
                            dpg.add_input_text(label=f'{key} [{unit}]', tag=key, width=145)
                            dpg.bind_item_theme(key,'draft_theme')
                    dpg.add_checkbox(label='此包允许正向拨弹 (草稿)', tag='permit')
                    self.button('下发 / 更新目标', self.submit, width=340, tag='manual_submit')
                    dpg.add_text('尚未提交', tag='intent', wrap=345)
                    dpg.add_separator()
                    dpg.add_text('3  观察域 / 关节参考', color=GREEN)
                    dpg.add_combo([], tag='focus', callback=lambda:self.focus_changed(), width=220)
                    dpg.add_text('—', tag='domain_state', wrap=345)
                    with dpg.group(tag='reference_controls'):
                        self.button('取当前角到目标草稿 (不发送)', self.copy_current_angle)
                        with dpg.group(tag='reference_align_controls'):
                            dpg.add_input_text(label='已知关节角 / 台架零点 [rad]', tag='known_angle', width=125)
                            dpg.bind_item_theme('known_angle','draft_theme')
                            self.button('确认已知关节参考', self.align, tag='manual_align')
                        dpg.add_text('', tag='reference_help', color=MUTED, wrap=345)
                with dpg.child_window(tag='workspace', width=-1, height=-66):
                    with dpg.tab_bar(tag='workspace_tabs'):
                        with dpg.tab(label='实时状态',tag='live_tab'):
                            dpg.add_text('采用目标 / 实测反馈 · 域状态约10Hz, 非1kHz原始波形', color=MUTED)
                            with dpg.group(horizontal=True):
                                self.button('恢复跟随 / 适配', self.fit_plots)
                                dpg.add_checkbox(label='实时跟随最近20秒', tag='follow', default_value=True,
                                                 callback=lambda:self.toggle_follow())
                                self.button('导出当前状态CSV', self.export_csv)
                            dpg.add_text('实时跟随 · 滚轮缩放或拖动后进入手动查看', tag='view_mode', color=MUTED)
                            dpg.add_text('等待域状态; 蓝线仅表示MCU实际采用目标', tag='plot_readout', color=AMBER, wrap=700)
                            for tag, title, series in (('tracking','目标与实测',('adopted','measured')),
                                                       ('error_plot','同参考系跟踪误差',('error_series',))):
                                with dpg.plot(label=title, tag=tag, height=240, width=-1):
                                    dpg.add_plot_legend()
                                    dpg.add_plot_axis(dpg.mvXAxis, label='主机接收时间 [s]', tag=tag+'_x')
                                    with dpg.plot_axis(dpg.mvYAxis, label='rad / rad/s', tag=tag+'_y'):
                                        for item in series:
                                            dpg.add_line_series([],[],label={'adopted':'MCU采用目标','measured':'反馈','error_series':'误差'}[item],tag=item,segments=True)
                                            dpg.add_scatter_series([],[],tag=item+'_points')
                                            color=(100,170,255) if item!='measured' else (255,170,95)
                                            with dpg.theme() as theme:
                                                for component in (dpg.mvLineSeries,dpg.mvScatterSeries):
                                                    with dpg.theme_component(component):
                                                        for key in (dpg.mvPlotCol_Line,dpg.mvPlotCol_MarkerFill,dpg.mvPlotCol_MarkerOutline):
                                                            dpg.add_theme_color(key,color,category=dpg.mvThemeCat_Plots)
                                            dpg.bind_item_theme(item,theme)
                                            dpg.bind_item_theme(item+'_points',theme)
                        with dpg.tab(label='参数草稿 / 读回',tag='parameters_tab'):
                            dpg.add_text('DJI速度PID使用S5电流口径: 界面自动按实际电流上限换算. 蓝色是草稿, 每次只应用一项.', color=MUTED, wrap=710)
                            with dpg.group(horizontal=True):
                                self.button('刷新实际参数', lambda:self.io('读取参数', lambda:self.require().refresh_parameters(), self.render_parameters))
                                self.button('保存 MCU 读回候选', self.save_profile)
                            dpg.add_input_text(label='候选 JSON 路径', tag='profile', width=470)
                            self.button('载入为草稿 (不发送)', self.load_profile)
                            dpg.add_text('先设置电流上限, 再设置速度PID. 改电流上限会改变现有增益的电流等效值; 参考类参数需退出执行.', color=AMBER, wrap=710)
                            with dpg.table(tag='parameters', header_row=True, resizable=True,
                                           borders_innerH=True, borders_outerH=True, policy=dpg.mvTable_SizingStretchProp):
                                for label in ('域 / 参数 / 单位','类别与显示范围','实际值 / MCU原生','草稿(所列单位)','动作'):
                                    dpg.add_table_column(label=label)
                        with dpg.tab(label='辅助调参', tag='assist_tab'):
                            self.build_assistance()
                        with dpg.tab(label='状态 / 日志',tag='logs_tab'):
                            dpg.add_text('状态页与通信统计, 不隐藏未配置/失效原因', color=MUTED)
                            dpg.add_input_text(tag='details', multiline=True, readonly=True, width=-1, height=310)
                            dpg.add_input_text(tag='events', multiline=True, readonly=True, width=-1, height=250)
            dpg.add_text('就绪', tag='message', color=AMBER, wrap=1200)
        with dpg.handler_registry():
            dpg.add_key_press_handler(dpg.mvKey_F8, callback=lambda:self.invoke(self.stop))
            dpg.add_mouse_wheel_handler(callback=lambda:self.plot_interaction())
            for button in (dpg.mvMouseButton_Left,dpg.mvMouseButton_Right):
                dpg.add_mouse_down_handler(button=button,callback=lambda:self.plot_interaction())
        self.scope_changed()

    def build_assistance(self):
        dpg.add_text('Pitch · 手动调参保留在“参数草稿 / 读回”；本页不自动写入增益。', wrap=700)
        dpg.add_text('水平检测复用已标定关节角和角速度，0 rad 是否为水平请先核对。', wrap=700, color=AMBER)
        dpg.add_checkbox(label='采集前自动调整到水平并等待稳定', tag='assist_lift', default_value=True)
        dpg.add_checkbox(label='采集完成后保持目标，等待拨杆切入遥控验证', tag='assist_remote', default_value=True)
        for title, fields in (
            ('水平位置与抬升设置', (
                ('level', '水平目标 [rad]', '0'), ('lift_speed', '抬升目标速度 [rad/s]', '0.05'),
                ('lift_timeout', '抬升最长时长 [s]', '12'), ('settle_error', '到位允许误差 [rad]', '0.02'),
                ('settle_speed', '稳定速度阈值 [rad/s]', '0.02'), ('settle_seconds', '连续稳定时长 [s]', '0.5'),
                ('wait_seconds', '到位后等待采集上限 [s]', '30'),
                ('remote_wait', '采集后等待遥控上限 [s]', '30'))),
            ('方波与采集配方（初始草稿，需按实验确认）', (
                ('amplitude', '叠加力矩幅度 [N·m]', '0.03'), ('half_period', '半周期 [s]', '0.1'),
                ('cycles', '正负方波周期数', '10'), ('baseline', '无扰动基线 [s]', '0.2'),
                ('final_hold', '方波后保持段 [s]', '0.2'), ('capture_duration', '采集时长上限 [s]', '5'),
                ('post_hold', '试验完成后保持 [s]', '0.5'), ('hold_timeout', '启用握手时限 [s]', '1'))),
            ('行程与监控边界', (
                ('physical_low', '已确认关节下界 [rad]', ''), ('physical_high', '已确认关节上界 [rad]', ''),
                ('excursion', '采集时目标两侧范围 [rad]', '0.02'),
                ('max_temperature', '温度上限 [°C]', '55'), ('max_torque', '反馈力矩上限 [N·m]', '2'),
                ('max_speed', '反馈速度上限 [rad/s]', '0.6'), ('max_error', '跟踪误差上限 [rad]', '0.03')))):
            with dpg.collapsing_header(label=title, default_open=title.startswith('水平')):
                with dpg.table(header_row=False):
                    dpg.add_table_column(); dpg.add_table_column(width_fixed=True, init_width_or_weight=150)
                    for key, label, default in fields:
                        self.assist_fields[key] = 'assist_' + key
                        with dpg.table_row():
                            dpg.add_text(label)
                            dpg.add_input_text(tag='assist_' + key, default_value=default, width=140)
        with dpg.group(horizontal=True):
            self.button('调整到水平并保持', self.assist_prepare, tag='assist_prepare')
            self.button('开始采集并叠加方波', self.assist_capture, tag='assist_capture')
            self.button('停止辅助调参', self.stop, tag='assist_stop')
        dpg.add_text('等“方波已结束，等待切遥控”后再拨档；试验中切档按中止处理。停止或等待超时会释放保持力矩。', color=AMBER, wrap=700)
        dpg.add_text('调参：左中＋右上；遥控：左上（右中主Yaw / 右上发射Yaw，右杆上下为Pitch）；右下停止。'
                     '需启用同固件遥控验证选项。所有阶段共用当前kp/kd。', wrap=700)
        dpg.add_text('尚未开始', tag='assist_state', wrap=700)
        dpg.add_input_text(label='原始记录路径', tag='assist_path', width=-1)
        self.button('读取采集结果', self.assist_load, tag='assist_load')
        dpg.add_text('采集后显示数据质量与响应统计；当前不自动计算可部署 kp/kd。', tag='assist_summary', wrap=700)
        for prefix, label, unit, series in (
            ('assist_position', '目标与角度', 'rad', [('assist_target', '采用目标'), ('assist_measured', '实测角度')]),
            ('assist_velocity', '角速度响应', 'rad/s', [('assist_speed', '实测角速度')]),
            ('assist_torque', '固件叠加信号', 'N·m', [('assist_effort', '编码前馈力矩')])):
            with dpg.plot(label=label, height=190, width=-1):
                dpg.add_plot_axis(dpg.mvXAxis, label='采样时间 [s]', tag=prefix + '_x')
                axis = dpg.add_plot_axis(dpg.mvYAxis, label=unit, tag=prefix + '_y')
                for tag, name in series:
                    dpg.add_line_series([], [], label=name, tag=tag, parent=axis)

    def assist_start(self, *, capture, lift):
        if self.job:
            raise ValueError('请等待当前参数或连接操作完成')
        session = self.require()
        values = {key: dpg.get_value(tag) for key, tag in self.assist_fields.items()}
        plan, wait = plan_from_fields(values, lift=lift, remote_handoff=bool(dpg.get_value('assist_remote')))
        directory = self.log_dir.parent / 'identification/pitch' / f'assisted-{time.time_ns()}'
        session.start_assisted(plan, directory, capture=capture, wait_s=wait)
        self.assist_seen_report = None
        dpg.set_value('assist_path', str(directory / 'pitch.sraw'))
        self.message('辅助调参已开始：先保持/抬升，实际到位后才启动叠加信号。')

    def assist_prepare(self):
        self.assist_start(capture=False, lift=True)

    def assist_capture(self):
        session = self.require()
        if session.assistance.active:
            session.assistance.capture()
            self.message('已请求采集；到位稳定后启动固件方波。')
        else:
            self.assist_start(capture=True, lift=bool(dpg.get_value('assist_lift')))

    def assist_load(self):
        session = self.session
        if session is not None and session.assistance.active:
            raise ValueError('请等待采集收尾后读取完整原始记录')
        path = Path(dpg.get_value('assist_path').strip())
        self.io('读取 Pitch 采集结果', lambda: capture_view(path), self.assist_render_capture)

    def assist_render_capture(self, view):
        fields = {'assist_target': 'pitch_adopted_rad', 'assist_measured': 'pitch_measured_rad',
                  'assist_speed': 'pitch_measured_velocity_rad_s', 'assist_effort': 'pitch_feedforward_torque_nm'}
        for tag, field in fields.items():
            dpg.set_value(tag, [view['times'], view['series'][field]])
        for prefix in ('assist_position', 'assist_velocity', 'assist_torque'):
            dpg.fit_axis_data(prefix + '_x'); dpg.fit_axis_data(prefix + '_y')
        dpg.set_value('assist_summary', f'来源：{view["source"]}；总记录 {view["records"]}；'
                      f'控制记录 {view["control_records"]}；序号缺口 {view["gaps"]}；最大 lost {view["lost_max"]}\n'
                      f'跟踪误差 RMS={view["rms_error_rad"]} rad，最大={view["max_error_rad"]} rad。'
                      '这是响应统计，不是自动 PID 推荐。')

    def mechanism(self):
        return next(k for k,v in PART_LABELS.items() if v == dpg.get_value('part'))

    def domain_label(self, domain):
        if self.mechanism() == 'bench':
            return {'main_yaw':'GM6020', 'pitch':'J4310', 'friction_left':'M3508', 'trigger':'M2006'}.get(domain,DOMAIN_LABELS[domain])
        return DOMAIN_LABELS[domain]

    def scope_changed(self):
        mechanism = self.mechanism()
        dpg.configure_item('assist_tab', show=mechanism == 'gimbal')
        for domain in wire.DOMAINS:
            dpg.configure_item('row_'+domain, show=domain in PARTS[mechanism])
            dpg.set_value('label_'+domain, self.domain_label(domain))
        for tag in ('yaw_frame_row', 'reference_controls'):
            dpg.configure_item(tag, show=mechanism in ('gimbal','bench'))
        dpg.configure_item('frame', items=['joint'] if mechanism=='bench' else list(wire.YAW_FRAMES))
        if mechanism=='bench':dpg.set_value('frame','joint')
        dpg.configure_item('twist_row', show=mechanism=='chassis')
        dpg.configure_item('permit', show=mechanism=='shooter')
        domains = list(PARTS[mechanism])
        dpg.configure_item('focus', items=domains);dpg.set_value('focus', domains[0])
        self.focus_changed()

    def focus_changed(self):
        self.fit_plots()
        self.focus = dpg.get_value('focus')
        dpg.configure_item('reference_controls', show=self.focus in PARTS['gimbal'])
        native_dm = self.mechanism() == 'bench' and self.focus == 'pitch'
        dpg.configure_item('reference_align_controls', show=not native_dm)
        dpg.set_value('reference_help', 'J4310显示已减配置零偏的关节角. 先取当前角到草稿; 0是配置零位, 不是当前位置. 不需人工设零.' if native_dm else '自由GM台架才用align建立零点. 标记零位调试APP自动恢复参考, 不填0覆盖当前位置; 先取当前角到草稿. 此操作不驱动回零.')
        for tag in ('adopted','measured','error_series'):
            dpg.set_value(tag,[[],[]])
            dpg.set_value(tag+'_points',[[],[]])

    def copy_current_angle(self):
        self.require()
        joint = self.history.joint(self.focus, time.monotonic())
        if not joint or not joint['position_valid']:
            raise ValueError('当前关节参考/反馈无效; 自由台架请先显式建立参考')
        dpg.set_value('target_'+self.focus, f'{joint["angle_rad"]:.8g}')
        self.message('当前角已复制到目标草稿, 未发送')

    def scan_ports(self):
        from serial.tools import list_ports
        self.message('可用串口: '+', '.join(f'{p.device} {p.description}' for p in list_ports.comports()))

    def require(self):
        if not self.session or not self.session.peer.snapshot()['connected']:
            raise ValueError('尚未连接有效 APP 会话')
        return self.session

    def io(self, label, action, done=None):
        if self.job:
            raise ValueError('上一项操作尚未完成; 目标发送不受影响, F8仍可撤销')
        self.job = (label, self.worker.submit(action), done)
        self.message(label+'…')

    def connect(self):
        if self.session:
            raise ValueError('请先断开当前会话')
        if self.job:
            raise ValueError('上一项操作尚未完成')
        mechanism = self.mechanism()
        mode, port = dpg.get_value('link'), dpg.get_value('port').strip()
        if mode=='APP串口' and not port:
            raise ValueError('请指定 USART1 业务串口, 921600 baud')
        path = self.log_dir/f'app-{time.time_ns()}.wire.jsonl'
        def connect():
            self.log_dir.mkdir(parents=True,exist_ok=True)
            extra = ['--plant', '--fixture', mechanism]
            if mode == 'Pitch辨识仿真':
                if mechanism != 'gimbal':
                    raise ValueError('Pitch 辨识仿真仅适用于云台')
                extra.append('--pitch-identification')
            transport = wire.SerialTransport(port,921600) if mode=='APP串口' else wire.ProcessTransport(
                self.backend,extra_args=extra)
            return MaintenanceSession.connect(transport,mechanism,path)
        def connected(session):
            self.session=session;self.history=LiveHistory();self.poll=None;self.next_poll=0
            # 新会话不继承仿真或其它参考下的运动草稿及参数草稿.
            self.param_ids=[]
            for domain in wire.DOMAINS:
                dpg.set_value('enable_'+domain,False);dpg.set_value('target_'+domain,'')
                installed = domain in session.installed_domains
                dpg.configure_item('enable_'+domain, enabled=installed)
                dpg.configure_item('target_'+domain, enabled=installed, hint='目标' if installed else '未装配')
            for tag in ('vx','vy','wz','known_angle'):dpg.set_value(tag,'')
            dpg.set_value('permit',False);dpg.set_value('use_twist',False);dpg.set_value('frame','joint')
            dpg.set_value('intent','尚未提交, 无持续目标')
            for tag in ('link','part','port','connect'):dpg.configure_item(tag,enabled=False)
            dpg.configure_item('disconnect',enabled=True)
            self.render_parameters(session.parameters)
            for entry in session.parameters:
                if entry['domain'] == 'pitch' and entry['id'] & 255 in (21, 22):
                    key = 'physical_low' if entry['id'] & 255 == 21 else 'physical_high'
                    dpg.set_value(self.assist_fields[key], str(entry['value']))
            available = [d for d in session.domains if d in session.installed_domains]
            if available:
                dpg.set_value('focus', available[0]);self.focus_changed()
            self.message(f'已连接, 未申请控制; 已装配: {available}; 字节日志: {path}')
        for tag in ('link','part','port','connect'):dpg.configure_item(tag,enabled=False)
        self.io('连接 APP',connect,connected)

    def disconnect(self):
        if not self.session:
            return
        session=self.session
        self.stop()
        def disconnected(_):
            self.session=None;self.poll=None;self.history.accept({'connected':False})
            for tag in ('link','part','port','connect'):dpg.configure_item(tag,enabled=True)
            dpg.configure_item('disconnect',enabled=False)
            self.message('已断开; 不把失联视为已经停机')
        def close():
            try:session.close()
            except Exception as error:return str(error)
            return None
        def done(error):
            disconnected(None)
            if error:self.message('释放未确认: '+error,True)
        self.io('释放并断开',close,done)

    def release(self):
        self.stop()
        self.io('释放维护',lambda:self.require().release())

    def submit(self):
        session=self.require()
        twist = tuple(dpg.get_value(key) for key in ('vx','vy','wz')) if session.mechanism=='chassis' and dpg.get_value('use_twist') else None
        drafts = {} if twist is not None else {domain:dpg.get_value('target_'+domain)
            for domain in session.domains if dpg.get_value('enable_'+domain)}
        session.submit(drafts,twist=twist,frame=dpg.get_value('frame'),
                       permit=dpg.get_value('permit') if session.mechanism=='shooter' else False)
        dpg.set_value('intent','最近提交: '+json.dumps(session.last_request,ensure_ascii=False))
        self.message(session.note)

    def stop(self):
        if self.session and self.session.peer.snapshot()['connected']:
            self.session.stop();self.message(self.session.note)

    def align(self):
        domain,value=self.focus,dpg.get_value('known_angle')
        self.io('确认关节参考',lambda:self.require().align(domain,value))

    def render_parameters(self, entries):
        drafts={pid:dpg.get_value(f'draft_{pid}') for pid in self.param_ids if dpg.does_item_exist(f'draft_{pid}')}
        dpg.delete_item('parameters',children_only=True,slot=1)
        self.param_ids=[]
        for entry in entries:
            pid=entry['id'];name,unit=parameter_label(entry);self.param_ids.append(pid)
            shown=physical_parameter(entry,entries)
            editable=bool(shown and entry['writable'])
            with dpg.table_row(parent='parameters'):
                dpg.add_text(f'{self.domain_label(entry["domain"])} / {name} [{unit}]'+chr(10)+f'0x{pid:04x}')
                limits=f'{shown["min"]:g} .. {shown["max"]:g}' if shown else '缺少有效电流上限'
                dpg.add_text(f'{entry["kind"]}'+chr(10)+limits)
                actual=f'{shown["value"]:.8g}' if shown else '无法换算'
                dpg.add_text(actual+chr(10)+f'原生 {entry["value"]:.8g}',tag=f'actual_{pid}',color=GREEN)
                dpg.add_input_text(default_value=drafts.get(pid,actual if shown else ''),tag=f'draft_{pid}',width=-1,enabled=editable)
                dpg.bind_item_theme(f'draft_{pid}','draft_theme')
                self.button('应用并读回' if editable else '只读',lambda p=pid:self.apply(p),enabled=editable,
                            tag=f'apply_{pid}', user_data=editable)
        if not entries:self.message('该机构未注册参数; 未配置不等于增益为0')

    def apply(self, pid):
        value=dpg.get_value(f'draft_{pid}')
        def done(result):
            if result['readback']:self.render_parameters(self.require().parameters)
            else:dpg.set_value(f'actual_{pid}','读回未确认')
            if result['unknown']:
                current=f'{result["readback"]["value"]:.8g}' if result['readback'] else '未确认'
                self.message(f'写入结果未知; MCU原生读回={current}; 不自动重新写入. {result["unknown"]}',True)
            elif result['read_error']:
                self.message(f'写入已 ACK 确认; 额外读回失败: {result["read_error"]}',True)
            else:
                shown=physical_parameter(result['readback'],self.require().parameters)
                actual=f'{shown["value"]:.8g}' if shown else '无法换算'
                self.message(f'ACK={result["ack"]["result_name"]}; 显示口径读回={actual}; MCU原生={result["readback"]["value"]:.8g}')
        self.io('应用参数',lambda:self.require().write_physical_parameter(pid,value),done)

    def save_profile(self):
        text=dpg.get_value('profile').strip()
        path=Path(text) if text else self.log_dir/f'{self.mechanism()}-{time.time_ns()}.json'
        self.io('保存 MCU 读回',lambda:self.require().save_readback(path),lambda _:self.message('已保存未验收候选: '+str(path)))

    def load_profile(self):
        path=Path(dpg.get_value('profile').strip())
        def done(drafts):
            entries=[{**e,'value':drafts.get(e['id'],e['value'])} for e in self.require().parameters]
            for entry in entries:
                if entry['id'] not in drafts:continue
                shown=physical_parameter(entry,entries)
                if shown:dpg.set_value(f'draft_{entry["id"]}',f'{shown["value"]:.8g}')
            self.message('已按候选电流上限换算为草稿, 未发送; 先应用电流上限, 再应用速度PID')
        self.io('载入候选草稿',lambda:self.require().load_drafts(path),done)

    def export_csv(self):
        rows=[(t,p.copy()) for t,p in self.history.samples]
        path=self.log_dir/f'status-{time.time_ns()}.csv'
        def export():
            path.parent.mkdir(parents=True,exist_ok=True)
            fields=('host_relative_s','clock_ms','config_epoch','reference_epoch','domain','mode','target','measurement',
                    'measurement_valid','measurement_unit','adopted_frame','measured_frame','execute','reason')
            with path.open('x',newline='',encoding='utf-8-sig') as file:
                writer=csv.DictWriter(file,fieldnames=fields);writer.writeheader()
                for t,page in rows:
                    for entry in page['domains']:
                        writer.writerow(dict(host_relative_s=t,**{k:page[k] for k in fields[1:4]},
                                             **{k:entry[k] for k in fields[4:]}))
        self.io('导出低频状态',export,lambda _:self.message('已导出: '+str(path)))

    def fit_plots(self):
        self.plot_following = True
        dpg.set_value('follow',True)
        dpg.set_value('view_mode','实时跟随 · 按当前20秒数据适配; 滚轮/拖动进入手动查看')

    def toggle_follow(self):
        if dpg.get_value('follow'):
            self.fit_plots()
        else:
            self.manual_plots()

    def manual_plots(self):
        if not self.plot_following:
            return
        self.plot_following = False
        dpg.set_value('follow',False)
        # Programmed limits lock native pan/zoom until explicitly released.
        for plot in ('tracking','error_plot'):
            for axis in ('x','y'):
                dpg.set_axis_limits_auto(plot+'_'+axis)
        dpg.set_value('view_mode','手动查看 · 视窗保持不变; 点“恢复跟随 / 适配”返回实时')

    def plot_interaction(self):
        if any(dpg.is_item_hovered(plot) for plot in ('tracking','error_plot')):
            self.manual_plots()

    def refresh(self):
        now=time.monotonic()
        if self.job and self.job[1].done():
            label,future,done=self.job;self.job=None
            try:
                value=future.result()
                if done:done(value)
                else:self.message(f'{label}: {value}')
            except Exception as error:
                self.message(f'{label}: {error}',True)
                if label=='连接 APP' and self.session is None:
                    for tag in ('link','part','port','connect'):dpg.configure_item(tag,enabled=True)
        snapshot=self.session.peer.snapshot() if self.session else {'connected':False}
        assisting = self.session is not None and self.session.assistance.active
        for tag in ('manual_authorize', 'manual_submit', 'manual_align', 'assist_prepare'):
            dpg.configure_item(tag, enabled=not assisting)
        for tag in (*self.assist_fields.values(), 'assist_lift', 'assist_remote'):
            dpg.configure_item(tag, enabled=not assisting)
        for pid in self.param_ids:
            editable = dpg.get_item_user_data(f'apply_{pid}')
            dpg.configure_item(f'apply_{pid}', enabled=not assisting and bool(editable))
        if self.session is not None:
            assist = self.session.assistance.snapshot()
            labels = {'IDLE':'尚未开始', 'PREFLIGHT':'读取配置与反馈', 'HOLDING':'确认保持',
                      'LIFTING':'缓慢调整到水平', 'SETTLING':'等待实际位置与速度稳定',
                      'READY':'保持中，等待采集', 'CAPTURING':'正在采集并叠加方波',
                      'WAIT_REMOTE':'方波已结束，保持中，等待拨杆切遥控验证',
                      'REMOTE_CONTROL':'遥控验证中，沿用本次kp/kd',
                      'STOPPING':'停止目标、撤权并保存数据', 'COMPLETE':'采集完成，保持已释放',
                      'FAILED':'流程已停止/失败，保持已释放或撤销待确认'}
            status = labels.get(assist['state'], assist['state'])
            if assist['state'] in ('READY', 'WAIT_REMOTE') and 'remaining_s' in assist:
                status += f'；剩余等待 {assist["remaining_s"]:.1f}s'
            if assist.get('error'):
                status += '\n' + str(assist['error'])
            stream = snapshot.get('pitch_stream', {})
            if assisting:
                status += f'；已收 {stream.get("records", 0)} 条'
            dpg.set_value('assist_state', status)
            report = assist.get('report')
            if report is not None and not assisting and not self.job:
                if report.get('revoked'):
                    self.session.requested_authority = False
                # 快照会深拷贝, 用目录身份防止每帧重复导出/读取.
                identity = assist['directory']
                if identity != self.assist_seen_report:
                    self.assist_seen_report = identity
                    dpg.set_value('assist_path', report.get('files', {}).get('sraw', ''))
                    if report.get('records'):
                        self.assist_load()
        self.history.accept(snapshot)
        if self.session and snapshot['connected']:
            if self.poll and self.poll.done():
                try:self.poll.result()
                except Exception as error:self.message('状态查询: '+str(error),True)
                self.poll=None
            if self.poll is None and now>=self.next_poll and not assisting:
                self.poll=self.session.peer.status(1);self.next_poll=now+.1
            fast=snapshot['status'].get(0)
            kind='物理设备配置' if snapshot.get('config_kind')==2 else '仿真/非部署配置'
            dpg.set_value('connection',f'{snapshot.get("identity","?")} · {kind} · 会话 {snapshot["session"]} · 配置 {snapshot["config_epoch"]} / 参考 {snapshot["reference_epoch"]}')
            if page_fresh(fast,now):
                dpg.set_value('authority',f'来源={fast["source"]}  执行资格={fast["authority"]}  人工停止={fast["manual_stop"]}  启动就绪={fast["locally_ready"]}')
            else:dpg.set_value('authority','快页过期 / 未收到, 不展示旧执行资格')
            ack=snapshot.get('last_target_ack')
            dpg.set_value('ack','目标ACK: '+(f'{ack["result_name"]} / {ack["reason_name"]} · ACK不代表运动执行' if ack else '尚无回执'))
            dpg.set_value('details',json.dumps(snapshot,ensure_ascii=False,indent=2))
            if self.session.note:dpg.set_value('intent',self.session.note+chr(10)+str(self.session.last_request or '无持续目标'))
        else:
            dpg.set_value('connection','未连接 / 会话失效; 历史曲线仅供回看')
            dpg.set_value('authority','执行资格未知');dpg.set_value('ack','目标ACK: —')
            for pid in self.param_ids:dpg.set_value(f'actual_{pid}','— / 已断开')
        joint=self.history.joint(self.focus,now)
        angle=f'{joint["angle_rad"]:.5f} rad' if joint and joint['position_valid'] else '未知 / 参考或反馈无效'
        rate=f'{joint["rate_rad_s"]:.4f} rad/s' if joint and joint['velocity_valid'] else '未知'
        dpg.set_value('joint_readout',f'{self.domain_label(self.focus)} 当前关节角: {angle}    当前速度: {rate}' if self.focus in PARTS['gimbal'] else '当前域为速度控制, 请看速度曲线')
        entry=self.history.domain(self.focus,now)
        if self.focus not in PARTS['gimbal']:
            actual=f'{entry["measurement"]:.4f} rad/s' if entry and entry['measurement_valid'] else '未知 / 反馈无效'
            dpg.set_value('joint_readout',f'{self.domain_label(self.focus)} 当前速度: {actual}')
        reason_names={'none':'正常','standby':'待机','no_target':'无目标','no_reference':'未建立参考',
            'fault_dependency':'反馈/参考/限位故障','no_authority':'无执行授权','degraded':'惯性不可用, 机械模式可用',
            'not_configured':'未装配','limit_violation':'超过配置范围','stale_feedback':'反馈过期'}
        text='域状态过期 / 未连接'
        if entry:
            text=('正在执行' if entry['execute'] else '未执行')+' / '+reason_names.get(entry['reason'],entry['reason'])
            text+=chr(10)+f'采用={entry["adopted_frame"]}  反馈={entry["measured_frame"]}  测量有效={entry["measurement_valid"]}'
            if self.session and joint and joint['position_valid']:
                limits={e['id']&255:e['value'] for e in self.session.parameters if e['domain']==self.focus and (e['id']&255) in (21,22)}
                if 21 in limits and 22 in limits:
                    text+=chr(10)+f'软件工作区 [{limits[21]:.5g}, {limits[22]:.5g}] rad'
                    if not limits[21]<=joint['angle_rad']<=limits[22]:text+=chr(10)+'当前角已越界, 新目标ACK也不代表会执行'
        dpg.set_value('domain_state',text)
        dpg.configure_item('domain_state',color=GREEN if entry and entry['execute'] else AMBER)
        unit='rad' if self.focus in PARTS['gimbal'] else 'rad/s'
        for plot in ('tracking','error_plot'):dpg.configure_item(plot+'_y',label=unit)
        t,target,measured,error=self.history.series(self.focus)
        for tag,values in (('adopted',target),('measured',measured),('error_series',error)):
            lines,points=plot_values(t,values)
            dpg.set_value(tag,lines)
            dpg.set_value(tag+'_points',points)
        if t and self.plot_following:
            low,high=max(0,t[-1]-20),max(1,t[-1]+.3)
            for plot,series in (('tracking',(target,measured)),('error_plot',(error,))):
                limits=visible_limits(t,series,low,high)
                if limits is not None:
                    dpg.set_axis_limits(plot+'_x',low,high)
                    dpg.set_axis_limits(plot+'_y',*limits)
        adopted=f'{entry["target"]:.5f} {unit}' if entry and entry['execute'] else '无 (MCU未执行)'
        actual=f'{entry["measurement"]:.5f} {unit}' if entry and entry['measurement_valid'] else '无有效反馈'
        dpg.set_value('plot_readout',f'MCU采用目标: {adopted}    反馈: {actual}'+chr(10)+
                      '点表示实际收到的样本; 断点表示状态间隔过长或参考改变, 不跨断点连线')
        height=max(165,int((dpg.get_item_rect_size('workspace')[1]-125)/2))
        for plot in ('tracking','error_plot'):dpg.configure_item(plot,height=height)

    def close(self):
        self.stop()
        if self.job:
            try:
                value=self.job[1].result(timeout=8)
                if isinstance(value,MaintenanceSession) and value is not self.session:value.close()
            except Exception as error:print('关闭前操作未确认:',error)
        if self.session:
            try:self.session.close()
            except Exception as error:print('关闭时释放未确认:',error)
        self.worker.shutdown(wait=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host-backend',type=Path)
    parser.add_argument('--serial',default='')
    parser.add_argument('--mechanism',choices=PARTS,default='gimbal')
    parser.add_argument('--log-dir',type=Path)
    parser.add_argument('--font',type=Path,default=Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts/msyh.ttc')
    args=parser.parse_args()
    dpg.create_context()
    try:
        dpg.configure_app(manual_callback_management=True)
        with dpg.font_registry():font=dpg.add_font(str(args.font),18)
        dpg.bind_font(font)
        dpg.create_viewport(title='Sentinel APP Maintenance',width=1320,height=900,min_width=1080,min_height=720)
        console=Console(backend=args.host_backend,serial=args.serial,mechanism=args.mechanism,log_dir=args.log_dir)
        dpg.setup_dearpygui();dpg.show_viewport();dpg.set_primary_window('main',True)
        next_refresh=0
        try:
            while dpg.is_dearpygui_running():
                jobs=dpg.get_callback_queue()
                if jobs:dpg.run_callbacks(jobs)
                if time.monotonic()>=next_refresh:
                    console.refresh();next_refresh=time.monotonic()+.1
                dpg.render_dearpygui_frame()
        finally:console.close()
    finally:dpg.destroy_context()


if __name__=='__main__':main()

"""正式 APP v3 分机构维护 GUI; 不兼容 S5/capture, 不自动连接或申请控制."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
import os
from pathlib import Path
import time

import dearpygui.dearpygui as dpg
import sentinel_client as wire
from sentinel_console_model import (MaintenanceSession, LiveHistory, PARTS, PART_LABELS,
    DOMAIN_LABELS, parameter_label, physical_parameter, page_fresh)

ROOT = Path(__file__).resolve().parents[3]
GREEN, AMBER, RED, MUTED = (100,220,170), (245,192,98), (245,106,106), (150,165,186)


class Console:
    def __init__(self, *, backend=None, serial='', mechanism='gimbal', log_dir=None):
        self.backend = Path(backend or ROOT/'build/sentinel-app-host/Release/ec_sentinel_app_host.exe')
        self.log_dir = Path(log_dir or ROOT/'build/sentinel-console-logs')
        self.session = None
        self.history = LiveHistory()
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='app-console-io')
        self.job = None
        self.poll = None
        self.next_poll = 0
        self.notes = []
        self.param_ids = []
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
                dpg.add_combo(['离线仿真', 'APP串口'], default_value='APP串口' if serial else '离线仿真', tag='link', width=120)
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
                        self.button('申请维护', lambda:self.io('申请维护', lambda:self.require().authorize()))
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
                    self.button('下发 / 更新目标', self.submit, width=340)
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
                            self.button('确认已知关节参考', self.align)
                        dpg.add_text('', tag='reference_help', color=MUTED, wrap=345)
                with dpg.child_window(tag='workspace', width=-1, height=-66):
                    with dpg.tab_bar(tag='workspace_tabs'):
                        with dpg.tab(label='实时状态',tag='live_tab'):
                            dpg.add_text('采用目标 / 实测反馈 · 域状态约10Hz, 非1kHz原始波形', color=MUTED)
                            with dpg.group(horizontal=True):
                                self.button('适配曲线', self.fit_plots)
                                dpg.add_checkbox(label='跟随最近20秒', tag='follow', default_value=True)
                                self.button('导出当前状态CSV', self.export_csv)
                            for tag, title, series in (('tracking','目标与实测',('adopted','measured')),
                                                       ('error_plot','同参考系跟踪误差',('error_series',))):
                                with dpg.plot(label=title, tag=tag, height=240, width=-1):
                                    dpg.add_plot_legend()
                                    dpg.add_plot_axis(dpg.mvXAxis, label='主机接收时间 [s]', tag=tag+'_x')
                                    with dpg.plot_axis(dpg.mvYAxis, label='rad / rad/s', tag=tag+'_y'):
                                        for item in series:
                                            dpg.add_line_series([],[],label={'adopted':'采用目标','measured':'反馈','error_series':'误差'}[item],tag=item)
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
                        with dpg.tab(label='状态 / 日志',tag='logs_tab'):
                            dpg.add_text('状态页与通信统计, 不隐藏未配置/失效原因', color=MUTED)
                            dpg.add_input_text(tag='details', multiline=True, readonly=True, width=-1, height=310)
                            dpg.add_input_text(tag='events', multiline=True, readonly=True, width=-1, height=250)
            dpg.add_text('就绪', tag='message', color=AMBER, wrap=1200)
        with dpg.handler_registry():
            dpg.add_key_press_handler(dpg.mvKey_F8, callback=lambda:self.invoke(self.stop))
        self.scope_changed()

    def mechanism(self):
        return next(k for k,v in PART_LABELS.items() if v == dpg.get_value('part'))

    def domain_label(self, domain):
        if self.mechanism() == 'bench':
            return {'main_yaw':'GM6020', 'pitch':'J4310', 'friction_left':'M3508', 'trigger':'M2006'}.get(domain,DOMAIN_LABELS[domain])
        return DOMAIN_LABELS[domain]

    def scope_changed(self):
        mechanism = self.mechanism()
        for domain in wire.DOMAINS:
            dpg.configure_item('row_'+domain, show=domain in PARTS[mechanism])
            dpg.set_value('label_'+domain, self.domain_label(domain))
        for tag in ('yaw_frame_row', 'reference_controls'):
            dpg.configure_item(tag, show=mechanism in ('gimbal','bench','integration'))
        dpg.configure_item('frame', items=['joint'] if mechanism=='bench' else list(wire.YAW_FRAMES))
        if mechanism=='bench':dpg.set_value('frame','joint')
        dpg.configure_item('twist_row', show=mechanism in ('chassis','integration'))
        dpg.configure_item('permit', show=mechanism=='shooter')
        domains = list(PARTS[mechanism])
        dpg.configure_item('focus', items=domains);dpg.set_value('focus', domains[0])
        self.focus_changed()

    def focus_changed(self):
        self.focus = dpg.get_value('focus')
        dpg.configure_item('reference_controls', show=self.focus in PARTS['gimbal'])
        native_dm = self.mechanism() == 'bench' and self.focus == 'pitch'
        dpg.configure_item('reference_align_controls', show=not native_dm)
        dpg.set_value('reference_help', 'J4310显示已减配置零偏的关节角. 先取当前角到草稿; 0是配置零位, 不是当前位置. 不需人工设零.' if native_dm else '自由GM台架才用align建立零点. 标记零位调试APP自动恢复参考, 不填0覆盖当前位置; 先取当前角到草稿. 此操作不驱动回零.')
        for tag in ('adopted','measured','error_series'):
            dpg.set_value(tag,[[],[]])

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
            transport = wire.SerialTransport(port,921600) if mode=='APP串口' else wire.ProcessTransport(
                self.backend,extra_args=['--plant','--fixture','all' if mechanism=='integration' else mechanism])
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
        twist = tuple(dpg.get_value(key) for key in ('vx','vy','wz')) if session.mechanism in ('chassis','integration') and dpg.get_value('use_twist') else None
        drafts = {domain:dpg.get_value('target_'+domain) for domain in session.domains
            if dpg.get_value('enable_'+domain) and (twist is None or domain not in PARTS['chassis'])}
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
                self.button('应用并读回' if editable else '只读',lambda p=pid:self.apply(p),enabled=editable)
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
        for plot in ('tracking','error_plot'):
            dpg.fit_axis_data(plot+'_x');dpg.fit_axis_data(plot+'_y')

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
        self.history.accept(snapshot)
        if self.session and snapshot['connected']:
            if self.poll and self.poll.done():
                try:self.poll.result()
                except Exception as error:self.message('状态查询: '+str(error),True)
                self.poll=None
            if self.poll is None and now>=self.next_poll:
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
            dpg.set_value(tag,[t,values])
        if t and dpg.get_value('follow'):
            for plot in ('tracking','error_plot'):
                dpg.set_axis_limits(plot+'_x',max(0,t[-1]-20),max(1,t[-1]));dpg.fit_axis_data(plot+'_y')
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

"""Sentinel single-window tool centre; resources must be released before navigation."""
import argparse
from pathlib import Path
import signal
import sys
import threading

from sentinel_ui_model import Navigator

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE_FEATURES = {
    '电机在线检查': dict(source='Hushi_EC_2026_Refactor', tool='capture-inspect',
                         images='build/capture-arm-Release/boards/sentinel_f407/sentinel_capture.elf',
                         description='Capture · 全车 10 路电机在线状态 / IMU / 遥控器 · 只读'),
    '板级观察': dict(source='Hushi_EC_2026_Refactor', tool='monitor',
                     images='build/arm-final-release/boards/sentinel_f407/sentinel_s1.elf',
                     description='S1 · 遥控器、IMU 与 CAN 状态观测'),
    '单电机台架': dict(source='test/HANDOFF-sentinel-s5-gm6020-id4-20261003/source', tool='bench',
                       images='build/s5-local-arm-Release/boards/sentinel_f407/sentinel_s1.elf',
                       description='S5 · GM6020 / CAN1 ID4 · 参数、曲线与记录'),
    '数据采集': dict(source='HANDOFF-sentinel-capture-20260924/source', tool='capture',
                     images='../firmware/sentinel_capture.elf',
                     description='Capture · 原版连续记录与电机 / IMU 检查'),
    '底盘控制': dict(source='HANDOFF-sentinel-chassis-app-20261003/source', tool='app', mechanism='chassis',
                     images='build/app-chassis-arm-Release/boards/sentinel_f407/sentinel_app.elf',
                     description='底盘 APP · 2026-10-03 · 参数与维护调试'),
    '云台控制': dict(source='HANDOFF-sentinel-gimbal-app-20260926/source', tool='app', mechanism='gimbal',
                     images='build/app-gimbal-arm-Release/boards/sentinel_f407/sentinel_app.elf',
                     description='云台 APP · 2026-09-26 · Yaw / Pitch 调试'),
    '发射控制': dict(source='HANDOFF-sentinel-shooter-app-20260926/source', tool='app', mechanism='shooter',
                     images='build/app-shooter-arm-Release/boards/sentinel_f407/sentinel_app.elf',
                     description='发射 APP · 2026-09-26 · 参数与维护调试'),
}
CATEGORIES = {
    **{name: [('flash', '固件烧录', profile['description']),
              (profile['tool'], '上位机调试', profile['description'])] +
             ([('capture-inspect', '检查电机 / IMU', profile['description'])] if profile['tool'] == 'capture' else [])
       for name, profile in FIRMWARE_FEATURES.items()},
    '离线分析': [('replay', '记录回放与曲线', '打开台架 A/B 记录；本页禁止连接串口。'),
               ('identify', '模型辨识', '使用训练与验证记录生成模型 JSON。'),
               ('tune', '调参建议', '读取模型与原生测速记录，生成参数建议，不下发。')],
    '构建与审计': [('build', '构建已有工程', '构建已配置的 CMake 目录。'),
                   ('audit', 'S1/S5 ELF 审计', '仅对应 S1/S5 镜像。'),
                   ('firmware-original', '原版完整固件工具', '保留全部用途、工具路径、构建、审计及烧录流程。')],
    '通用工具': [('app', 'APP 分机构维护', '使用首页所选源码，保留云台、底盘、发射与四电机台架选择。'),
                 ('capture', '连续只读采集', '使用首页所选源码的采集实现。'),
                 ('flash', '自选固件烧录', '查看所选工程构建镜像，也可手动输入 ELF 路径。')],
}
LABELS = dict(flash='固件烧录', monitor='板级观测', bench='台架调试', app='机构调试', capture='采集调试',
              **{'capture-inspect': '检查电机 / IMU', 'firmware-original': '完整固件工具'},
              replay='记录回放', identify='模型辨识', tune='调参建议', build='构建', audit='审计')


class Dashboard:
    def __init__(self, dpg, source=ROOT / 'Hushi_EC_2026_Refactor'):
        self.dpg = dpg
        self.source = Path(source).resolve()
        self.navigation = Navigator()
        self.page = None
        self.page_items = set()
        self.rendered = None
        self.modes_state = None
        self.form_drafts = {}
        self.pending_defaults = None
        with dpg.theme() as self.theme:
            with dpg.theme_component(dpg.mvAll):
                for key, color in (
                    (dpg.mvThemeCol_WindowBg, (255, 255, 255)),
                    (dpg.mvThemeCol_ChildBg, (255, 255, 255)),
                    (dpg.mvThemeCol_PopupBg, (255, 255, 255)),
                    (dpg.mvThemeCol_Text, (35, 42, 52)),
                    (dpg.mvThemeCol_TextDisabled, (125, 133, 145)),
                    (dpg.mvThemeCol_Border, (220, 225, 232)),
                    (dpg.mvThemeCol_FrameBg, (244, 246, 249)),
                    (dpg.mvThemeCol_FrameBgHovered, (234, 239, 245)),
                    (dpg.mvThemeCol_FrameBgActive, (224, 233, 244)),
                    (dpg.mvThemeCol_Button, (232, 238, 246)),
                    (dpg.mvThemeCol_ButtonHovered, (213, 226, 243)),
                    (dpg.mvThemeCol_ButtonActive, (192, 213, 239)),
                    (dpg.mvThemeCol_Header, (232, 238, 246)),
                    (dpg.mvThemeCol_HeaderHovered, (213, 226, 243)),
                    (dpg.mvThemeCol_HeaderActive, (192, 213, 239)),
                    (dpg.mvThemeCol_CheckMark, (40, 99, 170)),
                    (dpg.mvThemeCol_SliderGrab, (76, 123, 181)),
                    (dpg.mvThemeCol_SliderGrabActive, (40, 99, 170)),
                    (dpg.mvThemeCol_Separator, (220, 225, 232)),
                    (dpg.mvThemeCol_TableHeaderBg, (237, 242, 248)),
                    (dpg.mvThemeCol_TableBorderStrong, (207, 216, 228)),
                    (dpg.mvThemeCol_TableBorderLight, (227, 232, 239)),
                    (dpg.mvThemeCol_ScrollbarBg, (249, 250, 252)),
                    (dpg.mvThemeCol_ScrollbarGrab, (199, 207, 218)),
                    (dpg.mvThemeCol_ScrollbarGrabHovered, (174, 185, 201)),
                    (dpg.mvThemeCol_ScrollbarGrabActive, (149, 165, 186)),
                    (dpg.mvThemeCol_TextSelectedBg, (194, 218, 248)),
                ):
                    dpg.add_theme_color(key, color)
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 20, 16)
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 10, 7)
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 10, 10)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
                dpg.add_theme_style(dpg.mvStyleVar_FrameBorderSize, 1)
        dpg.bind_theme(self.theme)
        with dpg.theme() as self.active_mode_theme:
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (38, 99, 179))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (31, 85, 156))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (24, 68, 128))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255))
        with dpg.window(tag='ui.root'):
            with dpg.group(horizontal=True):
                dpg.add_button(label='返回首页', tag='ui.back', callback=self.back, width=120, height=36)
                dpg.add_text('Sentinel · 工具中心', color=(40, 99, 170))
                dpg.add_button(label='安全退出', callback=self.exit, width=120, height=36)
            dpg.add_text('首页', tag='ui.breadcrumb')
            options = [str(ROOT / 'Hushi_EC_2026_Refactor')]
            options += [str(p) for p in sorted(ROOT.glob('HANDOFF-*/source')) if p.is_dir()]
            if str(self.source) not in options:
                options.append(str(self.source))
            with dpg.group(tag='ui.source.options'):
                dpg.add_text('辅助工具源码（固件功能使用各自配套版本）')
                dpg.add_combo(options, default_value=str(self.source),
                              tag='ui.source', width=-1, callback=self.change_source)
            dpg.add_text('不会自动连接设备；串口释放 ≠ 电机停止。', color=(144, 94, 30))
            dpg.add_text('', tag='ui.notice', wrap=1100)
            dpg.add_separator()
            dpg.add_group(tag='ui.modes', horizontal=True)
            dpg.add_text('', tag='ui.firmware.source', wrap=0)
            dpg.add_group(tag='ui.actions')
            dpg.add_child_window(tag='ui.scroll', width=-1, height=-1, border=False)
            dpg.add_group(tag='ui.content', parent='ui.scroll')
        with dpg.window(tag='ui.gate', label='资源释放门禁', modal=True, show=False,
                        no_close=True, width=650, height=260):
            dpg.add_text('', tag='ui.gate.message', wrap=610)
            with dpg.group(horizontal=True):
                dpg.add_button(label='取消，留在当前页', tag='ui.cancel', callback=self.cancel)
                dpg.add_button(label='我已确认机构安全，开始释放资源', tag='ui.confirm', callback=self.confirm)
                dpg.add_button(label='重试释放', tag='ui.retry', show=False, callback=self.confirm)
        self.refresh_route()

    def change_source(self, sender=None, value=None):
        if self.navigation.route or not value:
            return
        source = Path(value).expanduser().resolve()
        if not source.is_dir():
            self.dpg.set_value('ui.notice', '源码目录不存在；保留原选择')
            self.dpg.set_value('ui.source', str(self.source))
            return
        self.source = source
        self.dpg.set_value('ui.notice', '源码已明确切换；不会自动选择其它交接版本。')

    def category(self, name):
        if self.navigation.route or self.navigation.state != 'ready':
            return
        self.navigation.open_category(name)
        self.refresh_route()
        if name in FIRMWARE_FEATURES:
            self.open_tool('flash')

    def _delete_page_items(self):
        dpg = self.dpg
        dpg.bind_theme(self.theme)
        roots = []
        for item in self.page_items:
            if not dpg.does_item_exist(item):
                continue
            parent = dpg.get_item_info(item)['parent']
            parent = dpg.get_alias_id(parent) if isinstance(parent, str) else parent
            if parent not in self.page_items:
                roots.append(item)
        for item in roots:
            if dpg.does_item_exist(item):
                dpg.delete_item(item)
        self.page_items.clear()

    def open_tool(self, kind, *, defaults=None):
        if self.navigation.state != 'ready':
            return
        if len(self.navigation.route) == 2:
            if kind != self.navigation.route[-1]:
                self.pending_defaults = defaults
                self.navigation.switch_tool(kind)
            return
        if len(self.navigation.route) != 1:
            return
        self._create_page(kind)
        if self.page:
            self.navigation.open_tool(kind, self.page)
            self.rendered = self.navigation.route
            self.update_header()

    def _create_page(self, kind):
        dpg = self.dpg
        dpg.delete_item('ui.content', children_only=True)
        before = set(dpg.get_all_items())
        try:
            profile = FIRMWARE_FEATURES.get(self.navigation.route[0])
            source = (ROOT / profile['source']).resolve() if profile else self.source
            required = {'app': 'robots/sentinel/host/sentinel_console.py',
                        'capture': 'robots/sentinel/host/sentinel_stream.py',
                        'capture-inspect': 'boards/sentinel_f407/tools/sentinel_capture.py'}.get(kind)
            if required and not (source / required).is_file():
                raise ValueError(f'所选源码缺少 {required}；请返回首页选择包含此工具的交接源码，或进入对应固件功能页')
            if kind == 'firmware-original':
                from sentinel_ui_adapters import FirmwarePage
                self.page = FirmwarePage(ROOT, source, dpg, 'ui.content')
            elif kind in ('bench', 'replay'):
                from sentinel_ui_adapters import BenchPage
                self.page = BenchPage(ROOT, dpg, 'ui.content', self.back, offline=kind == 'replay')
            elif kind == 'app':
                from sentinel_ui_adapters import AppPage
                self.page = AppPage(ROOT, source, dpg, 'ui.content', self.back,
                                    mechanism=profile['mechanism'] if profile else 'gimbal',
                                    fixed_mechanism=False)
            else:
                from sentinel_ui_pages import FormPage
                images = [(source / profile['images']).resolve()] if profile and kind == 'flash' else None
                self.page = FormPage(kind, ROOT, source, dpg, 'ui.content', images=images,
                                     capture_baud=None,
                                     actions_parent='ui.actions')
                self.page.switch_tool = self.open_tool
                draft_key = (self.navigation.route[0], kind, str(source))
                values = dict(self.form_drafts.get(draft_key, {}))
                values.update(self.pending_defaults or {})
                self.pending_defaults = None
                for key, value in values.items():
                    if key in self.page.fields:
                        dpg.set_value(self.page.fields[key], value)
                self.page.draft_key = draft_key
            dpg.bind_theme(self.theme)
            self.page_items = set(dpg.get_all_items()) - before
            dpg.set_value('ui.notice', '')
        except Exception as error:
            self.page_items = set(dpg.get_all_items()) - before
            self._delete_page_items()
            self.page = None
            dpg.set_value('ui.notice', '页面未打开: ' + str(error))

    def update_header(self):
        route = self.navigation.route
        self.dpg.set_value('ui.breadcrumb', ' / '.join(['首页'] + [LABELS.get(x, x) for x in route]))
        self.dpg.configure_item('ui.back', enabled=bool(route) and self.navigation.state == 'ready',
                                label='返回首页' if route and route[0] in FIRMWARE_FEATURES else '返回上级')
        self.dpg.configure_item('ui.source', enabled=not route)
        self.dpg.configure_item('ui.source.options', show=not route)
        self.dpg.configure_item('ui.notice', show=bool(self.dpg.get_value('ui.notice')))
        profile = FIRMWARE_FEATURES.get(route[0]) if route else None
        modes_state = (route, self.navigation.state)
        if self.modes_state != modes_state:
            self.dpg.delete_item('ui.modes', children_only=True)
            if profile:
                for key, title, _ in CATEGORIES[route[0]]:
                    label = '电机在线检查' if key == 'capture-inspect' else title
                    button = self.dpg.add_button(parent='ui.modes', label=label, width=175, height=40,
                                                enabled=self.navigation.state == 'ready',
                                                callback=lambda s, a, u: self.open_tool(u), user_data=key)
                    if route[-1] == key:
                        self.dpg.bind_item_theme(button, self.active_mode_theme)
            self.modes_state = modes_state
        self.dpg.set_value('ui.firmware.source', '配套版本：' + profile['source'] if profile else '')
        self.dpg.configure_item('ui.firmware.source', show=bool(profile))

    def refresh_route(self):
        route = self.navigation.route
        if self.rendered == route:
            return
        if self.page:
            # Called only after Navigator has verified release successfully.
            if hasattr(self.page, 'draft_key'):
                self.form_drafts[self.page.draft_key] = {
                    key: self.dpg.get_value(tag) for key, tag in self.page.fields.items()}
            self.page.dispose()
            self.page = None
        self._delete_page_items()
        self.dpg.delete_item('ui.content', children_only=True)
        if len(route) == 2:
            self._create_page(route[-1])
            self.navigation.resource = self.page
            self.rendered = route
            self.update_header()
            return
        with self.dpg.group(parent='ui.content'):
            if not route:
                self.dpg.add_text('选择用途', color=(40, 99, 170))
                for name in CATEGORIES:
                    self.dpg.add_button(label=name, width=450, height=55,
                                        callback=lambda s, a, u: self.category(u), user_data=name)
                    if name in FIRMWARE_FEATURES:
                        self.dpg.add_text(FIRMWARE_FEATURES[name]['description'], color=(100, 110, 125))
            elif len(route) == 1:
                for key, title, description in CATEGORIES[route[0]]:
                    self.dpg.add_button(label=title, width=470, height=48,
                                        callback=lambda s, a, u: self.open_tool(u), user_data=key)
                    self.dpg.add_text(description, wrap=1000)
                    self.dpg.add_separator()
        self.rendered = route
        self.update_header()

    def back(self):
        if self.navigation.route and self.navigation.route[0] in FIRMWARE_FEATURES:
            self.navigation.home()
        else:
            self.navigation.back()

    def exit(self):
        self.navigation.exit()

    def confirm(self):
        self.navigation.confirm()

    def cancel(self):
        self.pending_defaults = None
        self.navigation.cancel()

    def frame(self):
        dpg, nav = self.dpg, self.navigation
        nav.poll()
        if nav.state in ('ready', 'confirm') and self.page:
            try:
                self.page.tick()
            except Exception as error:
                dpg.set_value('ui.notice', '页面更新失败（请使用安全返回清理）: ' + str(error))
        self.refresh_route()
        self.update_header()
        gated = nav.state != 'ready'
        for group in ('ui.content', 'ui.actions'):
            if dpg.get_item_configuration(group)['enabled'] != (not gated):
                dpg.configure_item(group, enabled=not gated)
        dpg.configure_item('ui.gate', show=gated)
        dpg.configure_item('ui.confirm', show=nav.state == 'confirm')
        dpg.configure_item('ui.cancel', show=nav.state == 'confirm')
        dpg.configure_item('ui.retry', show=nav.state == 'blocked')
        messages = {
            'ready': '',
            'confirm': '请先人工将机构置于安全状态，再确认。\n确认后仅释放资源，不额外发送停止、失能或撤销控制权命令。\n取消会保留当前连接；断开通信可能触发下位机原有超时策略。',
            'releasing': '正在结束读写、等待后台任务并关闭串口。\n释放未确认前不能返回或退出；没有强制跳过选项。',
            'blocked': '资源释放失败，已禁止返回或退出。\n原因: ' + nav.error + '\n请排查并重试；不会把失败当作已释放。',
        }
        dpg.set_value('ui.gate.message', messages[nav.state])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT / 'Hushi_EC_2026_Refactor')
    parser.add_argument('--font', type=Path)
    args = parser.parse_args()
    font = args.font or next((Path(path) for path in (
        '/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc') if Path(path).is_file()), None)
    if font is None or not font.is_file():
        parser.error('缺少中文字体，请传 --font /path/to/font.ttf（或 .ttc）')
    import dearpygui.dearpygui as dpg
    dpg.create_context()
    dpg.configure_app(manual_callback_management=True)
    registry = dpg.add_font_registry()
    ui_font = dpg.add_font(str(font), 18, parent=registry)
    dpg.bind_font(ui_font)
    # The native close button cannot run a pre-close veto in DPG. Disable it;
    # the visible Safety Exit button and signals all go through the same gate.
    dpg.create_viewport(title='Sentinel Tool Centre', width=1360, height=940,
                        min_width=1100, min_height=760, disable_close=True)
    dashboard = Dashboard(dpg, args.source)
    requested_exit = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: requested_exit.set())
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window('ui.root', True)
    while not dashboard.navigation.exit_ready:
        callbacks = dpg.get_callback_queue()
        if callbacks:
            try:
                dpg.run_callbacks(callbacks)
            except Exception as error:
                dpg.set_value('ui.notice', '操作失败，连接仍受资源门禁管理: ' + str(error))
        if requested_exit.is_set() and dashboard.navigation.state == 'ready':
            requested_exit.clear()
            dashboard.exit()
        dashboard.frame()
        if not dashboard.navigation.exit_ready:
            dpg.render_dearpygui_frame()
    # Do not destroy the context until the model has verified the release gate.
    if dashboard.page:
        dashboard.page.dispose()
    dpg.destroy_context()
    return 0


if __name__ == '__main__':
    sys.exit(main())

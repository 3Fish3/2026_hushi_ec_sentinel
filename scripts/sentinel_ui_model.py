"""Single-window navigation. A route changes only after verified resource release."""
import queue
import threading


MOTOR_LABELS = {
    'LF': '左前轮', 'RF': '右前轮', 'LB': '左后轮', 'RB': '右后轮',
    'main_yaw': '主 Yaw', 'fire_yaw': '发射 Yaw',
    'friction_left': '左摩擦轮', 'friction_right': '右摩擦轮',
    'feeder': '拨弹', 'pitch_status': 'Pitch',
}
CAPTURE_STATES = {'ONLINE': '在线', 'NO_FEEDBACK': '未收到反馈', 'LOST': '失联', 'INVALID': '反馈无效'}


class CaptureReadout:
    """保留 MCU 在线判定, 页面缓存老化不冒充电机失联."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.motors = {}
        self.sensors = {}
        self.registers = {}
        self.link = 'WAITING'
        self.build_id = ''
        self.pages = {}

    def consume(self, event, now):
        kind = event.get('event')
        if kind == 'mcu_time_discontinuity':
            self.reset()
        elif kind == 'link_state':
            self.link = event['state']
        elif kind in ('protocol_error', 'disconnected'):
            self.link = kind.upper()
        elif kind == 'page':
            self.link = 'ONLINE'
            data = event['data']
            if 'page' in data:
                self.pages[data['page']] = (now, data)
            if 'build_id' in data:
                self.build_id = data['build_id']
            for motor in data.get('motors', []):
                if motor['name'] in MOTOR_LABELS:
                    self.motors[motor['name']] = (motor, now)
            for key in ('remote', 'imu'):
                if key in data:
                    self.sensors[key] = data[key]
            for register in data.get('registers', []):
                self.registers[register['name']] = register

    def row(self, name, now):
        if name not in self.motors:
            return ('--', '未检查', '--', '--', '--', '--')
        motor, received = self.motors[name]
        extra_ms = max(0, now - received) * 1000
        age = motor['age_ms'] + extra_ms
        state = CAPTURE_STATES.get(motor['state'], motor['state'])
        if self.link != 'ONLINE':
            state = '连接中断（缓存）'
        elif motor['state'] == 'ONLINE' and age >= (500 if motor['kind'] == 4 else 100):
            state = '在线（快照较旧）'
        raw = motor.get('p16') if motor['kind'] == 4 else motor.get('encoder')
        speed = motor.get('v12') if motor['kind'] == 4 else motor.get('rpm')
        def value(number):
            return '--' if number is None else str(number)
        return (f'CAN{motor["bus"]} / 0x{motor["can_id"]:03X}', state,
                value(raw), value(speed), value(motor.get('temperature_c')),
                str(int(age)) if motor['seen'] else '--')


class Navigator:
    def __init__(self):
        self.route: tuple[str, ...] = ()
        self.resource = None
        self.state = 'ready'
        self.error = ''
        self.exit_ready = False
        self._target: tuple[str, ...] | None = None
        self._results = queue.Queue()
        self._thread = None

    def open_category(self, name):
        if self.route or self.state != 'ready':
            raise RuntimeError('请先返回首页')
        self.route = (name,)

    def open_tool(self, name, resource):
        if len(self.route) != 1 or self.state != 'ready':
            raise RuntimeError('请先选择分类')
        self.route += (name,)
        self.resource = resource

    def back(self):
        self._leave(self.route[:-1])

    def home(self):
        self._leave(())

    def switch_tool(self, name):
        if len(self.route) != 2:
            raise RuntimeError('请先打开功能页面')
        self._leave(self.route[:1] + (name,))

    def exit(self):
        self._leave(None)

    def _leave(self, target):
        if self.state != 'ready':
            return
        self._target = target
        if self.resource is None:
            self._finish()
        elif self.resource.needs_confirmation:
            self.state = 'confirm'
        else:
            self.state = 'confirm'
            self.confirm()

    def cancel(self):
        if self.state == 'confirm':
            self.state = 'ready'
            self._target = None

    def confirm(self):
        if self.state not in ('confirm', 'blocked') or self.resource is None:
            return
        self.state = 'releasing'
        self.error = ''
        resource = self.resource
        try:
            if hasattr(resource, 'prepare_close'):
                resource.prepare_close()
        except Exception as error:
            self.state, self.error = 'blocked', str(error)
            return

        def work():
            try:
                resource.close()
                if not resource.released():
                    raise RuntimeError('串口、后台任务或线程仍未释放')
                self._results.put(None)
            except Exception as error:
                self._results.put(str(error))
        self._thread = threading.Thread(target=work, name='sentinel-release', daemon=False)
        try:
            self._thread.start()
        except RuntimeError as error:
            self._thread = None
            self.state, self.error = 'blocked', str(error)

    def poll(self):
        if self.state != 'releasing':
            return
        try:
            error = self._results.get_nowait()
        except queue.Empty:
            return
        if self._thread and self._thread.is_alive():
            self._results.put(error)
            return
        if error is not None:
            self.error = error
            self.state = 'blocked'
        else:
            self._finish()

    def wait(self, timeout):
        """Wait for cleanup; primarily useful to headless callers, never forces release."""
        if self._thread:
            self._thread.join(timeout)
        self.poll()

    def _finish(self):
        self.resource = None
        self.state = 'ready'
        if self._target is None:
            self.exit_ready = True
        else:
            self.route = self._target

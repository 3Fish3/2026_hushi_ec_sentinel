"""O5 主机客户端契约: C++/Python 双向 golden vectors 与端到端会话.

端到端路径是: Python 编码 → 0x7e 分帧 → 原生 APP 处理 → 真实协议回包 → Python 解码,
不是两个 Python 函数互测. 设备反馈通过后端 --inject 文件注入 (独立测试入口),
业务协议本身没有任意注入/CAN/内存写能力.
"""

import importlib.util
import json
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "sentinel_client", ROOT / "robots/sentinel/host/sentinel_client.py")
client_mod = importlib.util.module_from_spec(spec)
# dataclass 需要模块已登记, 动态加载时必须先注册.
sys.modules["sentinel_client"] = client_mod
spec.loader.exec_module(client_mod)
sys.path.insert(0, str(ROOT / 'robots/sentinel/host'))
import sentinel_async as async_mod
import sentinel_test as test_entry

BACKEND = Path(sys.argv.pop(1)) if len(sys.argv) > 1 else None


def bad_crc_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    """CRC 故意写错但转义正确, 使帧层能完整收到再判 CRC 失败."""
    raw = bytes([1, message_type, sequence & 0xFF, (sequence >> 8) & 0xFF,
                 len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    raw += bytes([0x12, 0x34])
    out = bytearray([client_mod.FRAME_START])
    for byte in raw:
        if byte in (client_mod.FRAME_START, client_mod.FRAME_ESCAPE):
            out += bytes([client_mod.FRAME_ESCAPE, byte ^ client_mod.ESCAPE_XOR])
        else:
            out.append(byte)
    out.append(client_mod.FRAME_START)
    return bytes(out)


def feedback_file(directory: Path, frames: int = 60) -> Path:
    """主 yaw 反馈注入 (测试入口): 覆盖前若干毫秒, 使位置/速度样本保持新鲜."""
    path = directory / "inject.txt"
    lines = ["# 测试注入: 主 yaw 编码器反馈, 与业务协议无关"]
    for ms in range(1, frames + 1):
        lines.append(f"dji {ms} main_yaw 0.0 0.0")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


class GoldenVectorTests(unittest.TestCase):
    """C++ 编码 == Python 编码; C++ 回包能被 Python 解码."""

    def test_golden_vectors(self):
        if BACKEND is None:
            self.skipTest("host backend not supplied")
        output = subprocess.check_output([str(BACKEND), "--golden"], text=True)
        lines = [line.split() for line in output.strip().splitlines()]
        pairs = {}
        for entry in lines:
            self.assertEqual(len(entry), 3)
            pairs[(entry[0], entry[1])] = bytes.fromhex(entry[2])
        # 请求方向: 同一语义消息必须逐字节一致.
        self.assertEqual(pairs[("REQ", "hello")], client_mod.encode_hello(0x11223344))
        session, config_epoch, reference_epoch = self._decode_hello(pairs[("RES", "hello")])
        self.assertEqual(session, 1)
        self.assertEqual(config_epoch, 1)
        self.assertEqual(
            pairs[("REQ", "authorize")],
            client_mod.encode_authorize(session, 1, True, "host"))
        self.assertEqual(
            pairs[("REQ", "targets")],
            client_mod.encode_targets(session, 2, config_epoch, reference_epoch,
                                      [("pitch", "position", 0.1, 0)], (0.2, 0.0, 0.0)))
        self.assertEqual(
            pairs[("REQ", "align")],
            client_mod.encode_align(session, 3, "pitch", 0.0, reference_epoch))
        self.assertEqual(
            pairs[("REQ", "param_write")],
            client_mod.encode_param_write(session, 4, 0x0001, 1.5))
        self.assertEqual(
            pairs[("REQ", "param_write_unsupported")],
            client_mod.encode_param_write(session, 5, 0x0005, 10000.0))
        self.assertEqual(
            pairs[("REQ", "param_read")],
            client_mod.encode_param_read(session, 6, 0x0001, 0))
        self.assertEqual(
            pairs[("REQ", "permit")],
            client_mod.encode_permit(session, 7, True, 500))
        self.assertEqual(pairs[("REQ", "status")], client_mod.encode_status(session, 8))
        self.assertEqual(pairs[("REQ", "diagnostics")],
                         client_mod.encode_diagnostics(session, 9))
        # 回包方向: Python 必须能解码 C++ 写出的字节.
        ack = client_mod.decode_ack(pairs[("RES", "authorize")])
        self.assertEqual(ack["result_name"], "ACCEPTED")
        self.assertEqual(ack["sequence"], 1)
        # 对齐被拒 (机构仍在执行) 时必须回明确的短错误包, 而不是按成功布局解释.
        align_ack = client_mod.decode_ack(pairs[("RES", "align")])
        self.assertEqual(align_ack["result_name"], "REFERENCE_IN_USE")
        write = client_mod.decode_ack(pairs[("RES", "param_write")])
        self.assertEqual(write["result_name"], "ACCEPTED")
        self.assertAlmostEqual(write["effective"], 1.5, places=6)
        # 尚未实现生效路径的参数必须明确报 unsupported, 不能谎报已生效.
        unsupported = client_mod.decode_ack(pairs[("RES", "param_write_unsupported")])
        self.assertEqual(unsupported["result_name"], "UNSUPPORTED")
        value = client_mod.decode_param_value(pairs[("RES", "param_read")])
        self.assertEqual(value["count"], 1)
        self.assertEqual(value["entries"][0]["id"], 0x0001)
        self.assertEqual(value["entries"][0]["kind"], "hot")
        self.assertAlmostEqual(value["entries"][0]["value"], 1.5, places=6)
        self.assertEqual(value['entries'][0]['domain'], 'chassis_lf')
        self.assertTrue(value['entries'][0]['overridden'])
        self.assertTrue(value['entries'][0]['writable'])
        self.assertFalse(value['entries'][0]['whole_vehicle'])
        status = client_mod.decode_status(pairs[("RES", "status")])
        self.assertEqual(status["config_epoch"], config_epoch)
        self.assertEqual(status["session"], session)
        self.assertEqual(len(status["domains"]), len(client_mod.DOMAINS))
        diagnostics = client_mod.decode_diagnostics(pairs[("RES", "diagnostics")])
        self.assertEqual(diagnostics["session_id"], session)
        self.assertEqual(diagnostics["session"], session)
        self.assertEqual(pairs[('REQ', 'fast')], client_mod.encode_status(session, 10, 0))
        self.assertEqual(pairs[('REQ', 'requests')], client_mod.encode_status(session, 11, 2))
        fast = client_mod.decode_status(pairs[('RES', 'fast')])
        self.assertEqual(len(pairs[('RES', 'fast')]), 214)
        self.assertEqual(len(fast['joints']), 3)
        self.assertTrue(fast['wheel_odom']['rank_ok'])
        self.assertEqual(fast['wheel_odom']['validity'], 'valid')
        self.assertNotEqual(fast['main_platform']['validity'], 'valid')
        self.assertFalse(fast['main_platform']['local_reference'])
        self.assertEqual(fast['main_platform']['time_error_us'], (None, None, None))
        requests = client_mod.decode_status(pairs[('RES', 'requests')])
        self.assertEqual(len(requests['requests']), 10)
        self.assertIn('command_prediction', requests)
        for seq, page in enumerate((1, 2, 10, 11, 12, 13, 14, 15, 16, 17, 20), start=12):
            self.assertEqual(pairs['REQ', f'diag{page}'], client_mod.encode_diagnostics(session, seq, page))
            body = pairs['RES', f'diag{page}']
            decoded = client_mod.decode_diagnostics(body)
            self.assertEqual(decoded['page'], page)
            self.assertLessEqual(len(body), client_mod.PAYLOAD_MAX)
            for malformed in (body[:-1], body+b'\0'):
                with self.assertRaises(client_mod.ProtocolError):
                    client_mod.decode_diagnostics(malformed)
        devices = client_mod.decode_diagnostics(pairs['RES', 'diag1'])
        self.assertEqual(len(devices['devices']), 10)
        # USART6 返回与业务快状态完全相同的已发布载荷; 只请求 0x36, 无 HELLO/授权副作用.
        sent = []
        class Observer:
            def exchange(_, data):
                parser = client_mod.FrameDecoder()
                request = next(frame for byte in data if (frame := parser.push(byte)) is not None)
                sent.append(request)
                return client_mod.OBSERVE_RESPONSE, request[1], pairs['RES', 'fast']
        observed = client_mod.observe_fast(Observer(), 42)
        self.assertEqual(observed, fast)
        self.assertEqual(sent, [(0x36, 42, b'')])
        # UART 两方向独立计预算: 有效消息真实编码 + 全转义上界; 不冒充实测时延/WCET.
        incoming = [(client_mod.encode_targets(1, 12, 1, 1,
            [(name, 'position' if name in client_mod.DOMAINS[4:7] else 'velocity', 0.1, 0)
             for name in client_mod.DOMAINS[4:]], (0.1, 0, 0)), 100),
            (client_mod.encode_status(1, 13, 0), 100), (client_mod.encode_diagnostics(1, 14, 2), 20),
            (client_mod.encode_param_write(1, 15, 1, 1.5), 20)]
        outgoing = [(pairs['RES', 'targets'], 100), (pairs['RES', 'fast'], 100),
                    (pairs['RES', 'status'], 10), (pairs['RES', 'diag2'], 10),
                    (pairs['RES', 'param_write'], 20)]
        capacity = 921600/10
        rx = sum(len(client_mod.encode_frame(body[1], 0x7e7d, body))*hz for body, hz in incoming)
        tx = sum(len(client_mod.encode_frame(body[1], 0x7e7d, body))*hz for body, hz in outgoing)
        self.assertLess(rx, capacity)
        self.assertLess(tx, capacity)
        self.assertGreater(tx, 115200/10)
        self.assertLess(sum((2*(len(body)+8)+2)*hz for body, hz in incoming), capacity)
        # 最慢出口即使全为 240 B 参数页, 也受 20 Hz 上限约束.
        self.assertLess((2*(12+8)+2)*100 + (2*(214+8)+2)*100 +
                        (2*(240+8)+2)*20 + (2*(24+8)+2)*20, capacity)
        self.assertLessEqual(len(client_mod.encode_frame(0xc7, 0x7e7d, b'\x7e'*240)), 512)
        for key in ('fast', 'status', 'requests'):
            value = pairs[('RES', key)]
            for malformed in (value[:-1], value + b'\0'):
                with self.assertRaises(client_mod.ProtocolError):
                    client_mod.decode_status(malformed)

    def _decode_hello(self, payload: bytes):
        info = client_mod.decode_hello_ack(payload)
        self.assertEqual(info["identity"], "sentinel_app")
        self.assertTrue(info["startable"])
        return info["session"], info["config_epoch"], info["reference_epoch"]


class LossyLink:
    """只丢指定请求/应答, 其余字节仍通过真实 C++ 后端; 不另写理想协议服务器."""
    def __init__(self, transport):
        self.inner = transport
        self.sent, self.responses = [], []
        self.threads = set()
        self.requests = {}
        self.drop_reply = None
        self.drop_all = False
        self.drop_request = None

    def send(self, message):
        self.threads.add(threading.get_ident())
        decoder = client_mod.FrameDecoder()
        frame = next(result for byte in message if (result := decoder.push(byte)) is not None)
        kind, echo, payload = frame
        sequence = struct.unpack_from('<I', payload, 6)[0] if len(payload) >= 10 else 0
        self.sent.append((kind, sequence, message, payload))
        self.requests[echo] = kind
        if kind == self.drop_request:
            self.drop_request = None
            return
        self.inner.send(message)

    def receive(self, timeout=0):
        self.threads.add(threading.get_ident())
        for _ in range(8):
            frame = self.inner.receive(timeout)
            if frame is None:
                return None
            op = self.requests.get(frame[1])
            self.responses.append((op, frame[2]))
            if op == self.drop_reply:
                if not self.drop_all:
                    self.drop_reply = None
                continue
            return frame
        return None

    def close(self):
        self.inner.close()


class AsyncLinkTests(unittest.TestCase):
    def setUp(self):
        if BACKEND is None:
            self.skipTest('host backend not supplied')
        self.transport = LossyLink(client_mod.ProcessTransport(BACKEND, extra_args=['--plant']))
        self.peer = async_mod.AsyncSentinelClient(self.transport, timeout_s=0.08)
        self.info = self.peer.hello().result(timeout=2)
        self.assertEqual(self.peer.authorize().result(timeout=2)['result_name'], 'ACCEPTED')

    def tearDown(self):
        self.peer.close()

    def wait_for(self, predicate):
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.003)
        self.fail(f'condition not reached: {self.peer.snapshot()}')

    def stream(self):
        self.peer.set_target_source(lambda now, epochs: dict(epochs, twist=(0.1, 0, 0)))
        self.wait_for(lambda: self.peer.snapshot()['last_target_ack'] is not None)

    def test_change_retry_while_targets_continue(self):
        self.stream()
        epoch = self.peer.snapshot()['reference_epoch']
        self.transport.drop_reply = client_mod.ALIGN
        future = self.peer.align('main_yaw', 0.0)
        with self.assertRaises(client_mod.ProtocolError):
            self.peer.param_write(1, 1.25)
        reply = future.result(timeout=2)
        self.assertEqual(reply['result_name'], 'ACCEPTED')
        self.assertEqual(reply['reference_epoch'], epoch+1)
        positions = [i for i, item in enumerate(self.transport.sent) if item[0] == client_mod.ALIGN]
        self.assertEqual(len(positions), 2)
        first, second = (self.transport.sent[i] for i in positions)
        self.assertEqual(first[2], second[2])
        between = self.transport.sent[positions[0]+1:positions[1]]
        self.assertTrue(any(op == client_mod.TARGETS and seq > first[1] for op, seq, _, _ in between))
        responses = [body for op, body in self.transport.responses if op == client_mod.ALIGN]
        self.assertEqual(responses[0], responses[1])
        self.assertGreaterEqual(self.peer.snapshot()['reference_epoch'], epoch+1)
        self.assertEqual(self.transport.threads, {self.peer._thread.ident})
        info = self.peer.diagnostics().result(timeout=2)
        self.assertGreaterEqual(info['duplicate_requests'], 1)
        self.transport.drop_reply = client_mod.PARAM_WRITE
        gain = self.peer.param_write(1, 1.25).result(timeout=2)
        self.assertEqual(gain['result_name'], 'ACCEPTED')
        self.assertAlmostEqual(gain['effective'], 1.25)
        self.assertAlmostEqual(self.peer.param_read(1).result(timeout=2)['entries'][0]['value'], 1.25)
        self.assertGreaterEqual(self.peer.snapshot()['statistics']['retries'], 2)
        # 参数分页不阻塞目标生成, 也不要求 GUI 线程消费每个 ACK.
        count = len([item for item in self.transport.sent if item[0] == client_mod.TARGETS])
        entries = self.peer.read_all_params().result(timeout=2)
        self.assertGreater(len(entries), 14)
        time.sleep(0.05)
        self.assertGreater(len([item for item in self.transport.sent if item[0] == client_mod.TARGETS]), count)

    def test_request_never_arrived_is_not_reissued_under_new_sequence(self):
        self.stream()
        epoch = self.peer.snapshot()['reference_epoch']
        self.transport.drop_request = client_mod.ALIGN
        with self.assertRaises(async_mod.OutcomeUnknown):
            self.peer.align('main_yaw', 0.0).result(timeout=2)
        requests = [entry for entry in self.transport.sent if entry[0] == client_mod.ALIGN]
        self.assertEqual(len(requests), 2)
        self.assertEqual(requests[0][2], requests[1][2])
        self.assertEqual(self.peer.diagnostics().result(timeout=2)['reference_epoch'], epoch)

    def test_lost_all_change_acks_reports_unknown_but_flow_continues(self):
        self.stream()
        epoch = self.peer.snapshot()['reference_epoch']
        self.transport.drop_reply, self.transport.drop_all = client_mod.ALIGN, True
        with self.assertRaises(async_mod.OutcomeUnknown):
            self.peer.align('main_yaw', 0.0).result(timeout=2)
        requests = [entry for entry in self.transport.sent if entry[0] == client_mod.ALIGN]
        self.assertEqual(len(requests), 3)
        self.assertEqual(len({entry[2] for entry in requests}), 1)
        self.assertEqual(self.peer.diagnostics().result(timeout=2)['reference_epoch'], epoch+1)
        self.assertTrue(self.peer.snapshot()['connected'])
        self.assertGreater(len([item for item in self.transport.sent if item[0] == client_mod.TARGETS]), 5)

    def test_latest_only_and_source_age(self):
        for i in range(1000):
            self.peer.publish_targets([('trigger', 'velocity', -i/1000, 0)])
        def latest():
            targets = [entry for entry in self.transport.sent if entry[0] == client_mod.TARGETS]
            return targets and abs(struct.unpack_from('<f', targets[-1][3], 22)[0]+0.999) < 1e-5
        self.wait_for(latest)
        self.assertGreater(self.peer.snapshot()['statistics']['superseded'], 0)
        self.peer.publish_targets([('trigger', 'velocity', -0.3, 0)], produced_at=time.monotonic()-1)
        self.wait_for(lambda: any(op == client_mod.TARGETS and struct.unpack_from('<H', body, 26)[0] >= 1000
                                 for op, _, _, body in self.transport.sent))
        # 新发射许可不把这份已经过期的拨弹目标重新变新.
        self.peer.publish_permit(True, 500)
        self.wait_for(lambda: self.peer.snapshot()['status'].get(0, {}).get('launch_allowed', False))
        status = self.peer.status(1).result(timeout=2)
        self.assertFalse(next(d for d in status['domains'] if d['domain'] == 'trigger')['execute'])

    def test_reference_change_stops_fixed_position_source(self):
        epoch = self.peer.snapshot()['reference_epoch']
        self.peer.set_target_source(lambda now, epochs: dict(epochs, setpoints=[('fire_yaw', 'position', 0.1, 0)])
                                   if epochs['reference_epoch'] == epoch else None)
        self.wait_for(lambda: self.peer.snapshot()['last_target_ack'] is not None)
        self.assertEqual(self.peer.align('main_yaw', 0.0).result(timeout=2)['result_name'], 'ACCEPTED')
        self.wait_for(lambda: self.peer._source is None)
        status = self.peer.status(1).result(timeout=2)
        self.assertFalse(next(d for d in status['domains'] if d['domain'] == 'fire_yaw')['execute'])


class MechanismEntryTests(unittest.TestCase):
    def test_three_partial_profiles_and_byte_log(self):
        if BACKEND is None:
            self.skipTest('host backend not supplied')
        with tempfile.TemporaryDirectory() as folder:
            for part in test_entry.PARTS:
                path = Path(folder)/f'{part}.jsonl'
                peer = async_mod.AsyncSentinelClient(client_mod.ProcessTransport(BACKEND,
                    extra_args=['--plant', '--fixture', part]), log_path=path)
                try:
                    info = peer.hello().result(timeout=2)
                    self.assertFalse(info['config_complete'])
                    test = test_entry.MechanismTest(peer, part)
                    if part == 'chassis':
                        test.start(twist=(0.05, 0, 0))
                        target = 'chassis_lf'
                    elif part == 'gimbal':
                        test.start([('main_yaw', 'position', 0.12)])
                        target = 'main_yaw'
                    else:
                        test.permit = True
                        test.start([('trigger', 'velocity', 2)])
                        target = 'trigger'
                    state = None
                    for _ in range(80):
                        state = peer.status(1).result(timeout=2)
                    domain = next(d for d in state['domains'] if d['domain'] == target)
                    self.assertTrue(domain['execute'], state)
                    self.assertGreater(abs(domain['measurement']), 1e-4)
                    for d in state['domains']:
                        if d['domain'] not in test.domains:
                            self.assertFalse(d['execute'])
                            self.assertNotEqual(d['validity'], 'valid')
                    profile = Path(folder)/f'{part}.json'
                    test.save(profile)
                    entries = client_mod.load_config(profile)
                    self.assertTrue(entries)
                    self.assertTrue(all(e['domain'] in test.domains and not e['whole_vehicle'] for e in entries))
                    with self.assertRaises(client_mod.ProtocolError):
                        client_mod.load_config(profile, target_config_kind=2)
                    test.stop()
                finally:
                    peer.close()
                records = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
                self.assertEqual(records[0]['event'], 'begin')
                self.assertEqual(records[-1]['event'], 'end')
                self.assertEqual(records[-1]['dropped'], 0)
                parser = client_mod.FrameDecoder()
                frames = [frame for record in records if record.get('direction') == 'rx'
                          for byte in bytes.fromhex(record['wire_hex']) if (frame := parser.push(byte)) is not None]
                self.assertTrue(any(frame[0] == client_mod.HELLO_ACK for frame in frames))
                self.assertEqual(parser.rejected, 0)


class SessionTests(unittest.TestCase):
    """端到端会话: 握手/授权/目标/参数/状态/诊断, 以及错误路径."""

    def setUp(self):
        self._temp = tempfile.TemporaryDirectory()
        self.inject = feedback_file(Path(self._temp.name))
        self.transport = client_mod.ProcessTransport(BACKEND, self.inject)
        self.client = client_mod.SentinelClient(self.transport)

    def tearDown(self):
        self.transport.close()
        self._temp.cleanup()

    def settle(self, cycles: int = 3) -> dict:
        """推进若干控制周期: 每个请求在后端各推进一个 1 ms 周期."""
        status = {}
        for _ in range(cycles):
            status = self.client.status()
        return status

    def test_full_session(self):
        if BACKEND is None:
            self.skipTest("host backend not supplied")
        info = self.client.hello()
        self.assertEqual(info["identity"], "sentinel_app")
        self.assertTrue(info["startable"])
        # 未授权时目标必须被拒绝, 且不能因为目标到达而打开权限.
        denied = self.client.targets([("main_yaw", "position", 0.1, 0)])
        self.assertEqual(denied["result_name"], "NOT_PERMITTED")
        granted = self.client.authorize(True, "host")
        self.assertEqual(granted["result_name"], "ACCEPTED")
        status = self.settle()
        self.assertEqual(status["source"], "host")
        self.assertTrue(status["authority"])
        # 建立位置参考 (单圈编码器不能自行恢复多圈角).
        aligned = self.client.align("main_yaw", 0.0)
        self.assertEqual(aligned["result_name"], "ACCEPTED")
        # 对齐使参考代次上推: 用旧代次下发的目标必须按 epoch_mismatch 拒绝.
        stale = self.client.targets([("main_yaw", "position", 0.1, 0)],
                                   reference_epoch=info["reference_epoch"])
        self.assertEqual(stale["result_name"], "EPOCH_MISMATCH")
        status = self.settle(2)
        self.assertNotEqual(status["reference_epoch"], info["reference_epoch"])
        accepted = self.client.targets([("main_yaw", "position", 0.1, 0)],
                                      reference_epoch=status["reference_epoch"])
        self.assertEqual(accepted["result_name"], "ACCEPTED")
        status = self.settle(2)
        entry = next(d for d in status["domains"] if d["domain"] == "main_yaw")
        self.assertEqual(entry["mode"], "position")
        self.assertTrue(entry["execute"])
        # 未下发的域保持显式无目标, 不隐式跟随.
        other = next(d for d in status["domains"] if d["domain"] == "trigger")
        self.assertFalse(other["execute"])
        # 反馈年龄来自样本时刻, 不是收包时刻.
        self.assertIsNotNone(entry["age_ms"])
        self.assertLess(entry["age_ms"], 1000)
        unavailable = next(d for d in status["domains"] if d["domain"] == "pitch")
        self.assertIsNone(unavailable["age_ms"])
        diagnostics = self.client.diagnostics()
        self.assertGreaterEqual(diagnostics["cycles"], 6)
        self.assertEqual(diagnostics["session_id"], info["session"])

    def test_parameter_readback_and_replay(self):
        if BACKEND is None:
            self.skipTest("host backend not supplied")
        self.client.hello()
        self.client.authorize(True, "host")
        self.settle(2)
        entries = self.client.read_all_params()
        self.assertGreater(len(entries), client_mod.PARAM_PAGE_MAX
                           if hasattr(client_mod, "PARAM_PAGE_MAX") else 0)
        hot = next(e for e in entries if e["kind"] == "hot")
        ack = self.client.param_write(hot["id"], hot["value"] + 0.25)
        self.assertEqual(ack["result_name"], "ACCEPTED")
        self.assertAlmostEqual(ack["effective"], hot["value"] + 0.25, places=6)
        # 读回必须等于实际生效值, 而不是回显请求.
        reread = next(e for e in self.client.read_all_params() if e["id"] == hot["id"])
        self.assertAlmostEqual(reread["value"], hot["value"] + 0.25, places=6)
        self.assertTrue(reread["overridden"])
        # 重复请求: 同一序号且内容相同回放原完整结果, 不重复执行也不谎报成功.
        sequence = self.client._next_sequence()
        first = self.client._request(
            client_mod.encode_param_write(self.client.session, sequence, hot["id"], 1.0),
            client_mod.PARAM_WRITE, client_mod.ACK, self.client.session, sequence)
        self.assertEqual(first["result_name"], "ACCEPTED")
        replay = self.client._request(
            client_mod.encode_param_write(self.client.session, sequence, hot["id"], 1.0),
            client_mod.PARAM_WRITE, client_mod.ACK, self.client.session, sequence)
        self.assertEqual(replay["result_name"], "ACCEPTED")
        self.assertAlmostEqual(replay["effective"], 1.0, places=6)
        # 同序号换内容: 不得报假成功.
        changed = self.client._request(
            client_mod.encode_param_write(self.client.session, sequence, hot["id"], 9.0),
            client_mod.PARAM_WRITE, client_mod.ACK, self.client.session, sequence)
        self.assertEqual(changed["result_name"], "REPLAY")
        # 读回仍然是第一次的值: 重复请求没有产生副作用.
        reread = next(e for e in self.client.read_all_params() if e["id"] == hot["id"])
        self.assertAlmostEqual(reread["value"], 1.0, places=6)
        # 更小的序号才是真重放: 明确拒绝.
        older = self.client._request(
            client_mod.encode_param_write(self.client.session, sequence - 1, hot["id"], 2.0),
            client_mod.PARAM_WRITE, client_mod.ACK, self.client.session, sequence - 1)
        self.assertEqual(older["result_name"], "REPLAY")
        # 越界与未知参数各自有明确结果码.
        out_of_range = self.client.param_write(hot["id"], 1.0e6)
        self.assertEqual(out_of_range["result_name"], "INVALID_VALUE")
        unknown = self.client.param_write(0xFFFE, 1.0)
        self.assertEqual(unknown["result_name"], "UNKNOWN_PARAM")
        # 参考类参数需要机构退出执行, 结果码必须区分"在用".
        reference = next(e for e in entries if e["kind"] == "reference")
        reference_ack = self.client.param_write(reference["id"], reference["value"])
        self.assertIn(reference_ack["result_name"], ("ACCEPTED", "REFERENCE_IN_USE"))
        # 主机配置文件只保存参数值, 不触碰 MCU Flash.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "params.json"
            client_mod.save_config(path, entries)
            saved = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(saved["protocol_version"], client_mod.PROTOCOL_VERSION)
            self.assertEqual(len(saved["params"]), len(entries))
            self.assertEqual(client_mod.load_config(path), saved["params"])

    def test_error_paths(self):
        if BACKEND is None:
            self.skipTest("host backend not supplied")
        # 未握手: 任何请求都按无会话拒绝.
        early = self.client.targets([("main_yaw", "position", 0.1, 0)])
        self.assertEqual(early["result_name"], "NO_SESSION")
        info = self.client.hello()
        # 错误版本: 明确回 bad_version, 不是超时.
        wrong = bytes([99, client_mod.STATUS]) + struct.pack("<II", info["session"], 1)
        reply = self.transport.exchange(client_mod.encode_frame(client_mod.STATUS, 900, wrong))
        ack = client_mod.decode_ack(reply[2])
        self.assertEqual(ack["result_name"], "BAD_VERSION")
        # 截断载荷: 必须报 bad_length 而不是被当成有效请求.
        truncated = bytes([client_mod.PROTOCOL_VERSION, client_mod.PARAM_WRITE]) + \
            struct.pack("<II", info["session"], 2)
        reply = self.transport.exchange(client_mod.encode_frame(client_mod.PARAM_WRITE, 901, truncated))
        self.assertEqual(client_mod.decode_ack(reply[2])["result_name"], "BAD_LENGTH")
        # 未知消息类型.
        unknown = bytes([client_mod.PROTOCOL_VERSION, 0x7F])
        reply = self.transport.exchange(client_mod.encode_frame(0x7F, 902, unknown))
        self.assertEqual(client_mod.decode_ack(reply[2])["result_name"], "UNKNOWN_REQUEST")
        # 损坏 CRC: 帧层拒绝, 不产生回包.
        self.transport._process.stdin.write(
            bad_crc_frame(client_mod.STATUS, 903,
                          client_mod.encode_status(info["session"], 3)))
        self.transport._process.stdin.flush()
        # 用一次正常请求确认链路仍然可用 (损坏帧被丢弃, 没有打乱后续帧).
        status = self.client.status()
        self.assertIn("authority", status)
        # 帧层拒绝计数可观测: 损坏帧是被丢弃, 不是被当成有效请求.
        self.assertGreaterEqual(self.client.diagnostics()["frame_rejects"], 1)

    def test_no_injection_means_no_feedback(self):
        """没有注入文件时不得伪造反馈: 域有效性保持 unknown, 年龄不可用."""
        if BACKEND is None:
            self.skipTest("host backend not supplied")
        transport = client_mod.ProcessTransport(BACKEND)
        try:
            client = client_mod.SentinelClient(transport)
            client.hello()
            client.authorize(True, "host")
            status = {}
            for _ in range(3):
                status = client.status()
            entry = next(d for d in status["domains"] if d["domain"] == "main_yaw")
            self.assertEqual(entry["validity"], "unknown")
            self.assertIsNone(entry["age_ms"])
        finally:
            transport.close()


class PlantClosedLoopTests(unittest.TestCase):
    """O7: 真实协议 + 原生 APP + 合成负载的闭环 (主机侧夹具).

    后端以 --plant 启动, 因此反馈来自"命令帧 → 合成运动 → 反馈帧", 而不是固定注入.
    负载是夹具, 结论只覆盖方向/单位/生命周期与"接受≠执行"语义, 不是实车稳定性验收.
    """

    def setUp(self):
        self.transport = client_mod.ProcessTransport(BACKEND, None, ["--plant"])
        self.client = client_mod.SentinelClient(self.transport)

    def tearDown(self):
        self.transport.close()

    def start(self, setpoints, twist=None):
        """握手 → 授权 → 建参考 → 下发目标; 返回当前状态与代次."""
        info = self.client.hello()
        self.assertEqual(info["identity"], "sentinel_app")
        self.assertEqual(self.client.authorize(True, "host")["result_name"], "ACCEPTED")
        # 发射许可走真实业务协议 (不再由后端硬编码为真): 后端只执行收到的显式请求.
        self.assertEqual(self.client.permit(True, 60000)["result_name"], "ACCEPTED")
        self.assertEqual(self.client.align("main_yaw", 0.0)["result_name"], "ACCEPTED")
        status = {}
        for _ in range(3):
            status = self.client.status()
        ack = self.client.targets(setpoints, twist,
                                  reference_epoch=status["reference_epoch"])
        self.assertEqual(ack["result_name"], "ACCEPTED")
        return info, status

    def step(self, setpoints, twist, cycles, refresh=5):
        """按周期刷新目标并推进闭环 (真实上位机行为), 返回最后一次状态."""
        status = None
        for index in range(cycles):
            if index % refresh == 0:
                status = self.client.status()
                ack = self.client.targets(setpoints, twist,
                                          reference_epoch=status["reference_epoch"])
                self.assertEqual(ack["result_name"], "ACCEPTED")
            else:
                status = self.client.status()
        return status

    def test_closed_loop_converges(self):
        if BACKEND is None:
            self.skipTest("host backend not supplied")
        setpoints = [("main_yaw", "position", 0.2, 0), ("friction_left", "velocity", 3.0, 0),
                     ("friction_right", "velocity", 3.0, 0), ("trigger", "velocity", 3.0, 0)]
        twist = (0.1, 0.0, 0.0)
        self.start(setpoints, twist)
        status = self.step(setpoints, twist, 900)
        entries = {entry["domain"]: entry for entry in status["domains"]}
        # 全部域都在执行, 且反馈来自真实闭环 (样本年龄可读且很小).
        for name in ("chassis_lf", "main_yaw", "friction_left", "trigger"):
            self.assertTrue(entries[name]["execute"], name)
            self.assertEqual(entries[name]["validity"], "valid", name)
            # 速度样本是每个域都有的必需反馈; 位置样本只对已建立参考的关节域有意义.
            self.assertIsNotNone(entries[name]["velocity_age_ms"], name)
            self.assertLess(entries[name]["velocity_age_ms"], 20, name)
        self.assertIsNotNone(entries["main_yaw"]["age_ms"])
        self.assertLess(entries["main_yaw"]["age_ms"], 20)
        # 未建立位置参考的底盘轮: 位置年龄必须明确不可用, 不能用 0 冒充"刚刚采样".
        self.assertIsNone(entries["chassis_lf"]["age_ms"])
        # 位置闭环收敛到目标 (夹具位置环增益较小, 给足时间).
        self.assertAlmostEqual(entries["main_yaw"]["measurement"], 0.2, delta=0.03)
        # 底盘按 twist 解算: 测量量在速度模式下是轴侧角速度, 方向与采用目标一致.
        for name in ("chassis_lf", "chassis_rf"):
            entry = entries[name]
            self.assertNotEqual(entry["target"], 0.0, name)
            self.assertGreater(entry["measurement"] * entry["target"], 0.0, name)
        # 摩擦轮/拨弹按正目标正向转 (轴侧量; 方向位只在原始 CAN 层体现).
        self.assertGreater(entries["friction_left"]["measurement"], 0.0)
        self.assertGreater(entries["trigger"]["measurement"], 0.0)
        diagnostics = self.client.diagnostics()
        self.assertGreaterEqual(diagnostics["cycles"], 900)
        self.assertEqual(diagnostics["rejected_targets"], 0)

    def test_documented_abi_hand_built_payloads(self):
        """按文档 §9 的偏移手工构造载荷 (不用 encode_* 助手), 核对实现与 ABI 描述一致.

        这是"从文档独立构造 golden": 字节来自文档中的字段表, 不由同一实现产生 expected.
        """
        if BACKEND is None:
            self.skipTest("host backend not supplied")
        transport = client_mod.ProcessTransport(BACKEND)
        try:
            client = client_mod.SentinelClient(transport)
            info = client.hello()
            session = info["session"]
            # authorize 请求: ver=3, type=0x41, session u32, seq u32, grant u8, source u8 (host=2)
            authorize = bytes([3, 0x41]) + struct.pack("<IIBB", session, 1, 1, 2)
            self.assertEqual(len(authorize), 12)
            ack = client._request(authorize, 0x41, 0xC1, session=session, sequence=1)
            self.assertEqual(ack["result_name"], "ACCEPTED")
            # permit 请求: ver=3, type=0x48, session u32, seq u32, allow u8, ttl u16
            permit = bytes([3, 0x48]) + struct.pack("<IIBH", session, 2, 1, 500)
            self.assertEqual(len(permit), 13)
            ack = client._request(permit, 0x48, 0xC1, session=session, sequence=2)
            self.assertEqual(ack["result_name"], "ACCEPTED")
            # 文档写 param_write 应答固定 24 字节 (BAD_LENGTH 除外).
            info = client.hello()
            session = info["session"]
            write = bytes([3, 0x45]) + struct.pack("<IIHf", session, 1, 0x0001, 1.5)
            self.assertEqual(len(write), 16)
            ack = client._request(write, 0x45, 0xC1, session=session, sequence=1)
            self.assertEqual(ack["result_name"], "ACCEPTED")
            self.assertIn("effective", ack)
        finally:
            transport.close()

    def test_accept_is_not_execution(self):
        """回包只表示接受; 是否执行只能由状态快照观测, 被拒目标不得显示为成功."""
        if BACKEND is None:
            self.skipTest("host backend not supplied")
        info = self.client.hello()
        self.client.authorize(True, "host")
        self.assertEqual(self.client.permit(True, 60000)["result_name"], "ACCEPTED")
        # 对齐使参考代次上推: 用握手时的旧代次下发必须被拒.
        self.assertEqual(self.client.align("main_yaw", 0.0)["result_name"], "ACCEPTED")
        status = {}
        for _ in range(3):
            status = self.client.status()
        self.assertNotEqual(status["reference_epoch"], info["reference_epoch"])
        # 被拒绝的目标 (代次错): 回包必须明确失败, 后续状态也不得显示为在执行.
        rejected = self.client.targets([("trigger", "velocity", 3.0, 0)],
                                       reference_epoch=info["reference_epoch"])
        self.assertEqual(rejected["result_name"], "EPOCH_MISMATCH")
        after = self.client.status()
        trigger = next(e for e in after["domains"] if e["domain"] == "trigger")
        self.assertFalse(trigger["execute"])
        self.assertNotEqual(trigger["reason"], "stale_target")
        # 接受的目标: 只有在控制周期真正跑过之后状态才显示执行.
        accepted = self.client.targets([("trigger", "velocity", 3.0, 0)],
                                       reference_epoch=after["reference_epoch"])
        self.assertEqual(accepted["result_name"], "ACCEPTED")
        self.assertTrue(self.client.status()["domains"][0] is not None)
        running = self.client.status()
        trigger = next(e for e in running["domains"] if e["domain"] == "trigger")
        self.assertTrue(trigger["execute"])
        self.assertEqual(trigger["validity"], "valid")
        # 停止刷新后目标按 TTL 失效: 状态必须如实反映, 不回显最后一次成功.
        for _ in range(60):
            final = self.client.status()
        trigger = next(e for e in final["domains"] if e["domain"] == "trigger")
        self.assertFalse(trigger["execute"])
        self.assertEqual(trigger["reason"], "stale_target")

    def test_flow_and_corruption_bounded(self):
        """慢读取与损坏帧下: 每个请求恰好一个回包, 后端保持有界工作量."""
        if BACKEND is None:
            self.skipTest("host backend not supplied")
        self.client.hello()
        self.client.authorize(True, "host")
        self.client.status()
        accepted_before = self.client.diagnostics()["accepted_commands"]
        for index in range(40):
            # 损坏帧: 帧层丢弃, 不产生回包也不影响后续请求.
            self.transport._process.stdin.write(
                bad_crc_frame(client_mod.STATUS, 4000 + index,
                              client_mod.encode_status(self.client.session, 1)))
            self.transport._process.stdin.flush()
            status = self.client.status()   # 慢读取: 每步都完整往返
            self.assertIn("authority", status)
            time.sleep(0.001)
        diagnostics = self.client.diagnostics()
        self.assertGreaterEqual(diagnostics["frame_rejects"], 40)
        # 观测请求不计入控制动作, 也不产生目标接受 (与握手后计数一致).
        self.assertEqual(diagnostics["accepted_commands"], accepted_before)
        # 计数是单调累加, 不是"有界队列长度"的伪指标.
        self.assertGreaterEqual(diagnostics["cycles"], 40)


class DecoderTests(unittest.TestCase):
    """分片/粘包与损坏恢复, 不依赖后端."""

    def test_chunking_and_resync(self):
        payload = bytes(range(1, 40))
        wire = client_mod.encode_frame(client_mod.STATUS, 7, payload)
        # 帧前插入垃圾字节: 解码器必须等起始符, 不能把噪声拼进帧.
        decoder = client_mod.FrameDecoder()
        frames = [f for f in (decoder.push(b) for b in bytes([0x11, 0x22, 0x33]) + wire) if f]
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][0], client_mod.STATUS)
        self.assertEqual(frames[0][2], payload)
        # 粘包: 两帧连续投递必须分别解出.
        decoder = client_mod.FrameDecoder()
        joined = wire + wire
        out = [f for f in (decoder.push(b) for b in joined) if f]
        self.assertEqual(len(out), 2)
        self.assertEqual(decoder.rejected, 0)
        # CRC 损坏: 丢弃并能在下一帧重新同步.
        decoder = client_mod.FrameDecoder()
        broken = bytearray(wire)
        broken[-2] ^= 0xFF
        out = [f for f in (decoder.push(b) for b in bytes(broken) + wire) if f]
        self.assertEqual(len(out), 1)
        self.assertEqual(decoder.rejected, 1)

    def test_ack_declared_lengths_only(self):
        """ACK 只接受声明的 12/20/24 字节: 其它长度不能因"至少 12 字节"就被当合法 ACK."""
        body = bytes([client_mod.PROTOCOL_VERSION, client_mod.ACK]) + struct.pack(
            "<IIBB", 0x11, 0x22, 0, 0)
        self.assertEqual(len(body), 12)
        client_mod.decode_ack(body)
        client_mod.decode_ack(body + b"\x00" * 8)  # 20: +两个代次
        client_mod.decode_ack(body + b"\x00" * 12)  # 24: +生效值+两个代次
        for extra in (1, 4, 7, 9, 13, 15):
            with self.assertRaises(client_mod.ProtocolError):
                client_mod.decode_ack(body + b"\x00" * extra)


if __name__ == "__main__":
    unittest.main(verbosity=2)

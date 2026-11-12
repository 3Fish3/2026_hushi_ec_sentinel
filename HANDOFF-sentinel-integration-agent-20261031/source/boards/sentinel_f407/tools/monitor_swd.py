"""SWD 只读监控读取: 运行中 RAM 的固定描述符 + 记录表, 不 halt/不写/不设目标.

设计边界:
  - 只读. 传输层只允许发出 read_memory; 出现 halt/resume/write/step 立即视为错误.
  - 以匹配 ELF/构建身份定位描述符: 描述符地址来自 ELF 符号 g_ec_monitor_desc,
    并核对描述符里的 magic/layout/record_size/capacity 与 ELF 里的构建标识字符串.
  - 读前后各读一次发布代次; 不一致则有限重试, 仍不一致就标记 inconsistent/stale,
    不通过停核来"补救".
  - 与串口监控共用同一套记录语义: 本模块的 decode_page 与板级 encode_page 对应.

已离线核对的 OpenOCD 契约 (本机 xPack OpenOCD 0.12.0+dev):
  - 内存命令名 read_memory, 单次请求上限 64K 个元素, 地址+数量不得回绕;
  - `tcl port` 必须在 init 之前配置; 本机离线会话里 tcl 端口接受连接但不返回数据,
    故首版 transport 使用已实测可用的 telnet 端口 (默认 4444) 行协议;
  - 真机 SWD 连机效果尚未验证, 离线自检只用假内存/假服务端.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import struct
import sys

MAGIC = 0x4D4F4E31
DESCRIPTOR_SIZE = 40
LAYOUT = 1
RECORD_SIZE = 24
CAPACITY = 48
PAGE_CAPACITY = 8
READ_ONLY_COMMANDS = ('read_memory',)
NEWLINE = b'\n'

CRLF = b'\r\n'


class MonitorError(RuntimeError):
    pass


# ---------------------------------------------------------------------------
# ELF: 只需要符号名 → 地址, 不依赖工具链.
# ---------------------------------------------------------------------------
def read_elf_symbols(path: Path) -> dict:
    data = path.read_bytes()
    if data[:4] != b'\x7fELF' or data[4] != 1 or data[5] != 1:
        raise MonitorError('expected a little-endian ELF32 image')
    section_offset, = struct.unpack_from('<I', data, 32)
    section_size, section_count, string_index = struct.unpack_from('<HHH', data, 46)
    sections = [struct.unpack_from('<IIIIIIIIII', data, section_offset + i * section_size)
                for i in range(section_count)]
    if string_index >= len(sections):
        raise MonitorError('ELF has no section header string table')
    symbols = {}
    for section in sections:
        name, kind = section[0], section[1]
        if kind != 2:  # SHT_SYMTAB
            continue
        link, entsize = section[6], section[9]
        if entsize == 0:
            raise MonitorError('empty symbol entry size')
        strings = sections[link]
        string_data = data[strings[4]:strings[4] + strings[5]]
        count = section[5] // entsize
        for i in range(1, count):
            offset = section[4] + i * entsize
            st_name, st_value, st_size, st_info, st_other, st_shndx = struct.unpack_from('<IIIBBH', data, offset)
            if st_name == 0 or st_shndx == 0:
                continue
            end = string_data.find(b'\0', st_name)
            symbols[string_data[st_name:end].decode('utf-8', 'replace')] = (st_value, st_size)
    return symbols


def read_c_string(transport, address: int, limit: int = 64) -> str:
    raw = transport.read(address, limit)
    end = raw.find(b'\0')
    if end < 0:
        raise MonitorError('unterminated string in target memory')
    return raw[:end].decode('utf-8', 'replace')


# ---------------------------------------------------------------------------
# 传输层: 只允许只读内存读取.
# ---------------------------------------------------------------------------
class MemoryTransport:
    name = 'abstract'

    def read(self, address: int, length: int) -> bytes:  # pragma: no cover - 接口
        raise NotImplementedError


class FakeMemoryTransport(MemoryTransport):
    """离线核对用: 直接在给定映像上按地址读取, 不接触任何探针."""

    name = 'fake'

    def __init__(self, base: int, image: bytes):
        self.base = base
        self.image = bytes(image)
        self.reads = []

    def read(self, address: int, length: int) -> bytes:
        offset = address - self.base
        if offset < 0 or offset + length > len(self.image):
            raise MonitorError(f'fake read outside image: 0x{address:08x}+{length}')
        self.reads.append((address, length))
        return self.image[offset:offset + length]


class OpenOcdTelnetTransport(MemoryTransport):
    """OpenOCD telnet 端口的只读内存读取.

    契约: 连接后出现 '> ' 提示符; 每条命令以 \\n 结束; 官方参数顺序为
    read_memory <address> <width> <count> (本类固定 width=8, count 即字节数).
    服务端会回显命令并可能带上 IAC 协商字节, 因此响应里的 token 不能直接当成数据:
    必须先去掉回显行与协商字节, 且响应必须以提示符结束才算完整.
    只发送 read_memory.
    """

    name = 'openocd-telnet'
    PROMPT = b'> '

    def __init__(self, host: str = '127.0.0.1', port: int = 4444, timeout: float = 5.0):
        self.socket = socket.create_connection((host, port), timeout=timeout)
        self.socket.settimeout(timeout)
        self.sent = []
        self._read_until_prompt()

    def _read_until_prompt(self) -> bytes:
        """读到提示符为止; 超时或对端关闭(未见到提示符)一律报错, 不当作成功响应."""
        buffer = bytearray()
        while True:
            try:
                chunk = self.socket.recv(4096)
            except socket.timeout:
                raise MonitorError('openocd telnet response timed out before the prompt') from None
            if not chunk:
                raise MonitorError('openocd telnet connection closed before the prompt')
            buffer += chunk
            if buffer.endswith(self.PROMPT):
                return bytes(buffer)

    @staticmethod
    def _strip_negotiation(payload: bytes) -> bytes:
        """去掉 telnet IAC 协商序列(IAC + 命令 + 选项), 只保留可读文本."""
        out = bytearray()
        index = 0
        while index < len(payload):
            byte = payload[index]
            if byte == 0xff and index + 2 < len(payload):  # IAC
                index += 3
                continue
            out.append(byte)
            index += 1
        return bytes(out)

    def read(self, address: int, length: int) -> bytes:
        # OpenOCD 官方契约是 read_memory <address> <width> <count>; width 只能取 8/16/32/64.
        # 本工具固定按字节读 (width=8), 因此 count 就是字节数.
        if length <= 0 or length > 4096:
            raise MonitorError('read length must be 1..4096 bytes')
        command = f'read_memory 0x{address:08x} 8 {length}'
        self.sent.append(command)
        # 纵深防御: 命令在这里是硬编码的, 该判断恒为真, 因此它**不是**只读性的安全证据.
        # 真正的保证来自本类只暴露 read() (没有写 RAM/halt/resume 的路径), 以及自检里
        # 对实际发出的命令清单的核对.
        if command.split()[0] not in READ_ONLY_COMMANDS:
            raise MonitorError('refusing to send a non read-only command')
        self.socket.sendall(command.encode('ascii') + b'\n')
        raw = self._strip_negotiation(self._read_until_prompt())
        lines = [line.strip() for line in raw.replace(b'\r', b'\n').split(b'\n')]
        # 第一行是命令回显(服务端会把收到的命令原样写回); 回显里的地址不是数据.
        echoed = False
        values = []
        for line in lines:
            if not line:
                continue
            if not echoed and line.startswith(b'read_memory'):
                echoed = True
                continue
            for token in line.split():
                if token == self.PROMPT.strip() or not token.startswith(b'0x'):
                    # 提示符与错误文本都不当数据; 但错误行要明确报错, 不静默跳过.
                    if not token.startswith(b'0x') and b':' in line and not echoed:
                        raise MonitorError(f'openocd refused the read: {line.decode("ascii", "replace")}')
                    continue
                try:
                    value = int(token, 16)
                except ValueError:
                    raise MonitorError(f'non-hex token in read_memory response: {token!r}') from None
                if not 0 <= value <= 0xff:
                    raise MonitorError(f'read_memory width=8 value out of range: {token!r}')
                values.append(value)
        if not echoed:
            raise MonitorError('openocd response did not echo the command; refusing to guess the payload')
        if len(values) != length:
            raise MonitorError(f'read_memory returned {len(values)} bytes for {length} requested')
        return bytes(values)

    def close(self):
        try:
            self.socket.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# 描述符与记录.
# ---------------------------------------------------------------------------
class Descriptor:
    def __init__(self, raw: bytes):
        if len(raw) != DESCRIPTOR_SIZE:
            raise MonitorError('descriptor has the wrong size')
        (self.magic, self.layout, self.record_size, self.capacity, self.records_address,
         self.generation_address, self.clock_address, self.build_id_address, self.used_address,
         self.flags) = struct.unpack('<10I', raw)
        if self.magic != MAGIC:
            raise MonitorError(f'descriptor magic mismatch: 0x{self.magic:08x}')
        if self.layout != LAYOUT:
            raise MonitorError(f'descriptor layout {self.layout} != tool layout {LAYOUT}')
        if self.record_size != RECORD_SIZE or self.capacity != CAPACITY:
            raise MonitorError('descriptor record layout does not match the tool')

    @property
    def enabled(self) -> bool:
        return bool(self.flags & 1)


KIND_NAMES = {0: 'unassigned', 1: 'device', 2: 'axis', 3: 'source', 4: 'period', 5: 'environment'}
VALIDITY_NAMES = {0: 'unknown', 1: 'valid', 2: 'stale', 3: 'invalid', 4: 'unavailable'}


class Record:
    def __init__(self, raw: bytes):
        (self.id, kind, validity, self.mode, self.enabled, reject, self.detail,
         self.sampled_at, self.published_at, target_bits, measurement_bits) = struct.unpack('<HBBBBBBIIII', raw)
        self.kind = KIND_NAMES.get(kind, f'kind{kind}')
        self.validity = VALIDITY_NAMES.get(validity, f'validity{validity}')
        self.reject = reject
        self.target = struct.unpack('<f', struct.pack('<I', target_bits))[0]
        self.measurement = struct.unpack('<f', struct.pack('<I', measurement_bits))[0]
        self.target_bits = target_bits
        self.measurement_bits = measurement_bits

    def as_dict(self) -> dict:
        return {
            'id': self.id, 'kind': self.kind, 'validity': self.validity, 'mode': self.mode,
            'enabled': self.enabled, 'reject': self.reject, 'detail': self.detail,
            'sampled_at': self.sampled_at, 'published_at': self.published_at,
            'target_bits': self.target_bits, 'measurement_bits': self.measurement_bits,
        }


class Snapshot:
    def __init__(self, generation: int, clock_ms: int, records: list):
        self.generation = generation
        self.clock_ms = clock_ms
        self.records = records
        self.consistent = True
        self.retries = 0

    def as_dict(self) -> dict:
        return {'generation': self.generation, 'clock_ms': self.clock_ms,
                'consistent': self.consistent, 'retries': self.retries,
                'records': [r.as_dict() for r in self.records]}


def read_snapshot(transport: MemoryTransport, descriptor_address: int, build_id: str | None = None,
                  retries: int = 3, max_records: int = CAPACITY) -> Snapshot:
    """读一次记录表; 用发布代次判断撕裂, 有限重试, 失败标记 inconsistent.

    代次协议: 写者复制发布区之前先把代次变成**奇数**(发布中), 复制完成后才变偶数.
    因此只有“两次读取相同且为偶数”才是完整发布 —— 两次读取都落在复制窗口内时会读到
    同一个奇数, 光比较“是否相同”会把混合快照当成一致.
    """
    descriptor = Descriptor(transport.read(descriptor_address, DESCRIPTOR_SIZE))
    if not descriptor.enabled:
        raise MonitorError('registry is not publishing (flags bit0 clear)')
    if build_id is not None and descriptor.build_id_address:
        target_build = read_c_string(transport, descriptor.build_id_address, 48)
        if target_build != build_id:
            raise MonitorError(f'build identity mismatch: target {target_build!r} != ELF {build_id!r}')
    for attempt in range(retries):
        before, = struct.unpack('<I', transport.read(descriptor.generation_address, 4))
        clock, = struct.unpack('<I', transport.read(descriptor.clock_address, 4))
        used = descriptor.capacity
        if descriptor.used_address:
            used, = struct.unpack('<I', transport.read(descriptor.used_address, 4))
        count = min(descriptor.capacity, used, max_records)
        raw = transport.read(descriptor.records_address, count * RECORD_SIZE)
        after, = struct.unpack('<I', transport.read(descriptor.generation_address, 4))
        if before == after and after % 2 == 0:
            records = [Record(raw[i * RECORD_SIZE:(i + 1) * RECORD_SIZE]) for i in range(count)]
            return Snapshot(before, clock, records)
    snapshot = Snapshot(before, clock, [])
    snapshot.consistent = False
    snapshot.retries = retries
    return snapshot


def resolve_descriptor(transport: MemoryTransport, symbols: dict, build_id: str | None) -> int:
    del transport, build_id  # 地址只来自 ELF 符号; 构建身份在 read_snapshot 里比对.
    if 'g_ec_monitor_desc' not in symbols:
        raise MonitorError('ELF has no g_ec_monitor_desc symbol; monitoring may be disabled')
    return symbols['g_ec_monitor_desc'][0]


def elf_c_string(path: Path, symbols: dict, name: str) -> str:
    """从 ELF 文件里读出某个字符串符号的内容 (vaddr → 文件偏移), 用于构建身份比对."""
    if name not in symbols:
        raise MonitorError(f'ELF has no {name} symbol')
    address = symbols[name][0]
    data = path.read_bytes()
    program_offset, = struct.unpack_from('<I', data, 28)
    program_size, program_count = struct.unpack_from('<HH', data, 42)
    for index in range(program_count):
        kind, offset, vaddr, _, file_size = struct.unpack_from('<IIIII', data, program_offset + index * program_size)
        if kind != 1 or not (vaddr <= address < vaddr + file_size):  # PT_LOAD
            continue
        cursor = offset + (address - vaddr)
        end = data.find(b'\0', cursor)
        if end < 0:
            raise MonitorError('unterminated ELF string symbol')
        return data[cursor:end].decode('utf-8', 'replace')
    raise MonitorError('build identity symbol is not inside a loadable segment')


# ---------------------------------------------------------------------------
# 串口线协议解码 (与板级 encode_page 对应).
# ---------------------------------------------------------------------------
def decode_page(payload: bytes) -> dict:
    if len(payload) < 24:
        raise MonitorError('page payload too short')
    layout, generation, clock_ms, flags, used, first, count, total = struct.unpack_from('<IIIIHHHH', payload)
    if layout != LAYOUT or count > PAGE_CAPACITY or total > CAPACITY or first > total:
        raise MonitorError('page header rejected')
    if len(payload) != 24 + count * RECORD_SIZE:
        raise MonitorError('page payload length mismatch')
    records = []
    for i in range(count):
        records.append(Record(payload[24 + i * RECORD_SIZE:24 + (i + 1) * RECORD_SIZE]))
    return {'layout': layout, 'generation': generation, 'clock_ms': clock_ms, 'flags': flags,
            'used': used, 'first': first, 'count': count, 'total': total,
            'records': [r.as_dict() for r in records]}


# ---------------------------------------------------------------------------
# 离线核对: 串口载荷与假 SWD 内存必须给出同一份逻辑值.
# ---------------------------------------------------------------------------
def verify_artifacts(directory: Path) -> dict:
    image = json.loads((directory / 'image.json').read_text(encoding='utf-8'))
    raws = (directory / 'ram.bin').read_bytes()
    transport = FakeMemoryTransport(image['base'], raws)
    symbols = {'g_ec_monitor_desc': (image['descriptor_address'], DESCRIPTOR_SIZE)}
    build_id = image['build_id']
    address = resolve_descriptor(transport, symbols, build_id)
    snapshot = read_snapshot(transport, address, build_id)
    if not snapshot.consistent:
        raise MonitorError('fake SWD read could not obtain a stable generation')
    serial = decode_page((directory / 'page.bin').read_bytes())

    if snapshot.generation != image['generation'] or snapshot.clock_ms != image['clock_ms']:
        raise MonitorError('fake SWD header does not match the artifact identity')
    if serial['generation'] != snapshot.generation or serial['clock_ms'] != snapshot.clock_ms:
        raise MonitorError('serial page and SWD snapshot disagree on the publish generation')
    if serial['count'] != min(PAGE_CAPACITY, image['records_used']) or serial['total'] != image['records_used']:
        raise MonitorError('serial page and SWD snapshot disagree on the record count')
    # 串口一页对应假 SWD 内存里同一段记录: 逐字段比较逻辑值.
    window = snapshot.records[serial['first']:serial['first'] + serial['count']]
    if len(window) != serial['count']:
        raise MonitorError('serial page window is outside the SWD record range')
    for index, (from_ram, from_wire) in enumerate(zip(window, serial['records'])):
        if from_ram.as_dict() != from_wire:
            raise MonitorError(f'record {index} differs between serial page and SWD memory')
    if len(snapshot.records) != image['records_used']:
        raise MonitorError('SWD snapshot record count does not match the artifact identity')
    return {'result': 'pass', 'records': len(snapshot.records), 'generation': snapshot.generation,
            'reads': len(transport.reads), 'descriptor': hex(address), 'build_id': build_id}


# ---------------------------------------------------------------------------
# 自检: 假 telnet 服务端 + 真实 transport 类, 断言只发只读命令.
# ---------------------------------------------------------------------------
def _fake_server(base: int, image: bytes, behaviour: str = 'stable'):
    """启动一个假 OpenOCD telnet 服务端, 返回 (transport, stop, sent)."""
    import threading
    sent = []
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 0))
    server.listen(1)
    port = server.getsockname()[1]
    buffer = bytearray(image)

    def serve():
        connection, _ = server.accept()
        connection.sendall(b'> ')
        reads = 0
        while True:
            data = b''
            while not data.endswith(NEWLINE):
                chunk = connection.recv(4096)
                if not chunk:
                    connection.close()
                    return
                data += chunk
            line = data.decode('ascii').strip()
            sent.append(line)
            if line == 'shutdown':
                connection.close()
                return
            if not line.startswith('read_memory'):
                connection.sendall(b'only read_memory is allowed' + CRLF + b'> ')
                continue
            # 真实 OpenOCD telnet 会回显收到的命令(并可能带 IAC 协商): 默认行为就是回显,
            # 回显里的地址不能被当成第一个数据字节; 'no_echo' 专门模拟缺失回显的反例.
            if behaviour != 'no_echo':
                prefix = b'' if behaviour != 'echo' else bytes([0xff, 0xfb, 0x01])
                connection.sendall(prefix + line.encode('ascii') + CRLF)
            parts = line.split()
            # 假服务端只接受官方参数顺序: read_memory <address> <width> <count>.
            if len(parts) != 4:
                connection.sendall(b'read_memory: expected address width count' + CRLF + b'> ')
                continue
            address = int(parts[1], 16)
            width = int(parts[2])
            count = int(parts[3])
            if width not in (8, 16, 32, 64):
                connection.sendall(
                    f'read_memory: invalid width {width}'.encode('ascii') + CRLF + b'> ')
                continue
            reads += 1
            # 撕裂: 代次字段在每次读取时变化. 地址从镜像里的描述符读取, 不硬编码偏移,
            # 否则描述符布局一变, 自检会“静默不再测撕裂”.
            generation_address, = struct.unpack_from('<I', buffer, 0x14)
            generation_offset = generation_address - base
            if behaviour == 'tear' and address == generation_address:
                # 每次读代次都变(且为奇数 = 发布中): 读者必须有限重试后标记不一致, 不能挂死.
                value, = struct.unpack_from('<I', buffer, generation_offset)
                struct.pack_into('<I', buffer, generation_offset, value + 1 | 1)
            if behaviour == 'publishing' and address == generation_address:
                # 模拟“读取落在发布复制窗口中”: 代次为奇数(发布中).
                struct.pack_into('<I', buffer, generation_offset, 1)
            if behaviour == 'error_line' and reads == 1:
                connection.sendall(b'Error: memory read failed' + CRLF + b'> ')
                continue
            if behaviour == 'partial' and reads == 1:
                connection.sendall(b'0x01 0x02' + CRLF + b'> ')
                continue
            if behaviour == 'drop' and reads >= 3:
                connection.close()
                return
            if behaviour == 'eof_after_data' and reads == 1:
                # 有数据但没有提示符: 响应不完整, 不能当成功.
                connection.sendall(b'0x01 0x02 0x03 0x04')
                connection.close()
                return
            size = count * (width // 8)
            offset = address - base
            if offset < 0 or offset + size > len(buffer):
                connection.sendall(b'read_memory: read failed' + CRLF + b'> ')
                continue
            digits = width // 4
            values = []
            for index in range(count):
                start = offset + index * (width // 8)
                word = int.from_bytes(buffer[start:start + (width // 8)], 'little')
                values.append(f'0x{word:0{digits}x}')
            connection.sendall(' '.join(values).encode('ascii') + CRLF + b'> ')

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    transport = OpenOcdTelnetTransport(port=port)

    def stop():
        try:
            transport.socket.sendall(b'shutdown' + NEWLINE)
        except OSError:
            pass
        thread.join(timeout=5)
        transport.close()
        server.close()

    return transport, stop, sent


def _self_test_image() -> tuple:
    """构造假 SWD 内存: 记录区与描述符指向的控制字段必须互不重叠."""
    base = 0x20000000
    records_at = 0x100
    records_end = records_at + CAPACITY * RECORD_SIZE
    control_at = records_end
    offsets = {'records_at': records_at, 'records_end': records_end, 'generation_at': control_at,
               'clock_at': control_at + 4, 'used_at': control_at + 8, 'build_at': control_at + 0x10}
    # 镜像必须足够容纳记录区 + 控制字段 + 构建标识字符串的 48 字节读取窗口,
    # 否则假服务端会在越界时拒绝读取, 把“镜像太小”伪装成读取失败.
    image = bytearray(offsets['build_at'] + 0x40)
    struct.pack_into('<10I', image, 0, MAGIC, LAYOUT, RECORD_SIZE, CAPACITY,
                     base + records_at, base + offsets['generation_at'], base + offsets['clock_at'],
                     base + offsets['build_at'], base + offsets['used_at'], 1)
    for i in range(CAPACITY):
        record = struct.pack('<HBBBBBBIIII', i, 1, 1, 2, 1, 0, 0, 1000 + i, 1000, 0x3F800000, 0x40000000)
        image[records_at + i * RECORD_SIZE:records_at + (i + 1) * RECORD_SIZE] = record
    # 已发布状态用偶数代次 (写者复制前变奇数, 复制完成后变偶数).
    struct.pack_into('<I', image, offsets['generation_at'], 8)
    struct.pack_into('<I', image, offsets['clock_at'], 1234)
    struct.pack_into('<I', image, offsets['used_at'], CAPACITY)
    image[offsets['build_at']:offsets['build_at'] + 5] = b'self\x00'
    return base, bytes(image), offsets


def self_test() -> dict:
    base, image, offsets = _self_test_image()
    # 布局本身先检查: 控制字段落在记录区内时, 自检会在被覆盖的记录上“通过”.
    for name in ('generation_at', 'clock_at', 'used_at', 'build_at'):
        if offsets['records_at'] <= offsets[name] < offsets['records_end']:
            raise MonitorError(f'self-test image layout overlaps: {name}')
    transport, stop, sent = _fake_server(base, image)
    snapshot = read_snapshot(transport, base)
    stop()
    if not snapshot.consistent or snapshot.generation != 8 or len(snapshot.records) != CAPACITY:
        raise MonitorError('self-test snapshot mismatch')
    # 逐条核对记录内容: 只看数量与代次无法发现记录被覆盖.
    for index, record in enumerate(snapshot.records):
        if record.id != index or record.sampled_at != 1000 + index or record.published_at != 1000:
            raise MonitorError(f'self-test record {index} was not read back intact')
        if record.mode != 2 or record.enabled != 1 or record.target != 1.0 or record.measurement != 2.0:
            raise MonitorError(f'self-test record {index} payload was not read back intact')
    if not sent or any(line.split()[0] not in READ_ONLY_COMMANDS for line in sent if line != 'shutdown'):
        raise MonitorError('self-test transport issued a non read-only command')

    # 撕裂读取: 代次在每次读取时变化, 必须有限重试后标记不一致.
    transport, stop, _ = _fake_server(base, image, 'tear')
    torn = read_snapshot(transport, base, retries=3)
    stop()
    if torn.consistent:
        raise MonitorError('tearing was not detected')
    if torn.retries != 3:
        raise MonitorError('bounded retry count is not reported')

    # 版本不符与构建身份不符都必须拒绝, 而不是“凑合解码”.
    # 注意: “本应失败却返回”的断言必须放在 except 之外, 否则会被自己的异常捕获吞掉.
    wrong_layout = bytearray(image)
    struct.pack_into('<I', wrong_layout, 4, LAYOUT + 1)
    transport, stop, _ = _fake_server(base, bytes(wrong_layout))
    try:
        read_snapshot(transport, base)
    except MonitorError as error:
        if 'layout' not in str(error):
            raise
    else:
        raise MonitorError('layout mismatch was accepted')
    stop()

    transport, stop, _ = _fake_server(base, image)
    try:
        read_snapshot(transport, base, build_id='other-build')
    except MonitorError as error:
        if 'build identity' not in str(error):
            raise
    else:
        raise MonitorError('build identity mismatch was accepted')
    stop()

    # 回显 + IAC 协商: 回显里的地址不能被当成第一个数据字节.
    transport, stop, _ = _fake_server(base, image, 'echo')
    echoed = read_snapshot(transport, base)
    stop()
    if not echoed.consistent or len(echoed.records) != CAPACITY:
        raise MonitorError('echoed response was not parsed correctly')
    if echoed.records[0].id != 0 or echoed.records[1].id != 1:
        raise MonitorError('echoed response shifted the record payload')

    # 发布中(奇代次): 读取必须被拒绝并有限重试后标记不一致, 不能当成一致快照.
    transport, stop, _ = _fake_server(base, image, 'publishing')
    publishing = read_snapshot(transport, base, retries=3)
    stop()
    if publishing.consistent:
        raise MonitorError('a mid-publish (odd generation) read was accepted')

    # 错误行 / 数据不足 / 有数据但无提示符 / 缺失回显: 都必须明确失败, 不能返回部分数据.
    for behaviour, what in (('error_line', 'error line'), ('partial', 'short response'),
                            ('eof_after_data', 'EOF without prompt'), ('no_echo', 'missing echo')):
        transport, stop, _ = _fake_server(base, image, behaviour)
        try:
            read_snapshot(transport, base)
        except MonitorError:
            pass
        else:
            raise MonitorError(f'{what} was accepted as a complete response')
        stop()

    # 断流: 读取中途连接关闭必须报错, 不能挂死.
    transport, stop, _ = _fake_server(base, image, 'drop')
    try:
        read_snapshot(transport, base)
    except MonitorError:
        pass
    else:
        raise MonitorError('link drop was accepted')
    stop()
    return {'result': 'pass', 'commands': sent, 'records': len(snapshot.records),
            'generation': snapshot.generation, 'tearing': 'detected', 'retries': torn.retries}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', type=Path, help='目标 ELF (定位 g_ec_monitor_desc 与构建标识)')
    parser.add_argument('--ocd-host', default='127.0.0.1')
    parser.add_argument('--ocd-port', type=int, default=4444)
    parser.add_argument('--verify', type=Path, help='离线核对素材目录 (假 SWD 内存 + 串口载荷)')
    parser.add_argument('--self-test', action='store_true', help='假 telnet 服务端自检, 不连探针')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    try:
        if args.verify is not None:
            result = verify_artifacts(args.verify)
        elif args.self_test:
            result = self_test()
        else:
            if args.elf is None:
                raise MonitorError('--elf is required for a live SWD read')
            symbols = read_elf_symbols(args.elf)
            transport = OpenOcdTelnetTransport(args.ocd_host, args.ocd_port)
            address = resolve_descriptor(transport, symbols, None)
            build_id = elf_c_string(args.elf, symbols, 'g_ec_s1_build_id')
            snapshot = read_snapshot(transport, address, build_id)
            transport.close()
            result = snapshot.as_dict()
    except MonitorError as error:
        print(json.dumps({'result': 'fail', 'error': str(error)}) if args.json else f'FAIL: {error}')
        return 1
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(f"result: {result.get('result', 'ok')}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

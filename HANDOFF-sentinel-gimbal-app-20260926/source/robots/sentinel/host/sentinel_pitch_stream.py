"""Pitch 连续控制流只读解码器: 纯字节解析, 不持有串口, 不发送任何请求或执行器目标.

本模块只把固件 b3 连续帧负载还原为帧头与标准 64B 记录, 不产生任何控制授权.
上游调用方负责建立/停止采集; 解码失败一律抛出 ValueError, 不做宽容截断.
"""
from __future__ import annotations

import struct

VERSION = 1
LAYOUT = 0x52415701
CAPACITY = 512
HEADER = struct.Struct('<4B7I')
HEADER_BYTES = HEADER.size
RECORD_BYTES = 64
MAX_RECORDS = 3
MAX_PACKET = HEADER_BYTES + MAX_RECORDS * RECORD_BYTES  # 224B: 32B 头 + 3 x 64B 完整控制记录
KIND_CAN, KIND_CONTROL = 4, 5
CAN_STREAM, PITCH_DOMAIN = 9, 6
PITCH_BUS, PITCH_CAN_ID, PITCH_DLC = 2, 0x011, 8
HEADER_KEYS = ('version', 'op', 'error', 'state', 'run', 'generated', 'dropped', 'buffered',
               'start_us', 'end_us', 'layout_id')


def decode_pitch_stream_packet(payload: bytes, expected_run: int | None = None) -> tuple[dict, list[bytes]]:
    """解码一帧 Pitch 连续流负载, 返回 (帧头字典, 标准 64B 记录列表).

    帧头 state: 0 未启动, 1 记录中, 2 已冻结且有积压, 3 已结束且积压为 0.
    记录只接受 control 域 6 (完整 64B) 与 CAN2 0x011 反馈 (44B, 零填充重建为 64B).
    """
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise ValueError('连续帧负载类型错误')
    payload = bytes(payload)
    if len(payload) < HEADER_BYTES:
        raise ValueError('连续帧截断')
    if len(payload) > MAX_PACKET:
        raise ValueError(f'连续帧超出 {MAX_PACKET}B 上限')
    status = dict(zip(HEADER_KEYS, HEADER.unpack_from(payload), strict=True))
    if status['version'] != VERSION:
        raise ValueError('连续帧版本错误')
    if status['op'] > 3:
        raise ValueError('连续帧操作码错误')
    if status['state'] > 3:
        raise ValueError('连续帧状态码错误')
    if status['error'] > 3:
        raise ValueError('连续帧错误码错误')
    if status['layout_id'] != LAYOUT:
        raise ValueError('连续帧布局错误')
    if status['error']:
        raise ValueError(f'Pitch 连续流请求被拒绝: {status["error"]}')
    if expected_run is not None:
        if not isinstance(expected_run, int) or expected_run <= 0:
            raise ValueError('期望 run 非法')
        if status['run'] != expected_run:
            raise ValueError('连续帧 run 与期望不一致')
    if status['state'] != 0:
        if not status['run']:
            raise ValueError('活动连续帧缺少 run')
        # 32 位微秒时基只允许在半个回绕窗口内比较; 超出即为倒退或歧义.
        if (status['end_us'] - status['start_us']) & 0xffffffff >= 0x80000000:
            raise ValueError('连续帧时间戳超出半回绕窗口')
    if status['buffered'] > CAPACITY:
        raise ValueError('连续帧队列计数越界')
    if status['generated'] < status['buffered']:
        raise ValueError('连续帧生成数小于积压数')
    if status['state'] == 3 and status['buffered']:
        raise ValueError('已结束连续帧仍报告积压')
    if status['state'] == 2 and not status['buffered']:
        raise ValueError('冻结连续帧未报告积压')
    if status['op'] != 3:
        if len(payload) != HEADER_BYTES:
            raise ValueError('非数据连续帧携带尾部数据')
        return status, []
    if len(payload) == HEADER_BYTES:
        # 合法的最终结束帧: 停止/到期且积压已清零后, 固件只补发一个头部帧.
        # 记录中 (state 1) 或仍冻结且报告积压 (state 2) 的空数据帧一律视为损坏.
        if status['state'] != 3:
            raise ValueError('数据连续帧不含记录')
        return status, []
    offset = HEADER_BYTES
    records: list[bytes] = []
    while offset < len(payload):
        if len(records) == MAX_RECORDS:
            raise ValueError('连续帧记录数超出上限')
        if offset + 4 > len(payload):
            raise ValueError('紧凑记录截断')
        version, kind, stream, flags = payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
        if version != VERSION:
            raise ValueError('记录版本错误')
        if flags & ~7:
            raise ValueError('记录标志非法')
        if kind == KIND_CONTROL:
            if stream != PITCH_DOMAIN:
                raise ValueError('控制记录域不是 Pitch 域 6')
            size = RECORD_BYTES
        elif kind == KIND_CAN:
            if stream != CAN_STREAM:
                raise ValueError('CAN 记录不是 Pitch 反馈流 9')
            size = 44
        else:
            raise ValueError('Pitch 连续流不接受 IMU 或未知记录')
        if offset + size > len(payload):
            raise ValueError('紧凑记录截断')
        record = payload[offset:offset + size]
        if struct.unpack_from('<I', record, 28)[0] != status['run']:
            raise ValueError('记录 run 与帧头不一致')
        if kind == KIND_CAN:
            info = struct.unpack_from('<I', record, 32)[0]
            if info & 255 != PITCH_BUS:
                raise ValueError('Pitch CAN 记录总线不是 CAN2')
            if (info >> 8) & 255 != PITCH_DLC:
                raise ValueError('Pitch CAN 记录 DLC 不是 8')
            if info >> 16 != PITCH_CAN_ID:
                raise ValueError('Pitch CAN 记录 ID 不是 0x011')
            # 44B 紧凑帧省去的 data[3..7] 协议约定为零, 重建标准 64B 记录使其可无损落盘.
            record = record + bytes(RECORD_BYTES - size)
        records.append(record)
        offset += size
    return status, records

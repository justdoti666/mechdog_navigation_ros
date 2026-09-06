#!/usr/bin/env python3
"""ultrasonic_serial_parser — STM32 串口超声上报帧 (0x02) 解析器

协议 (见 ../docs/STM32_ULTRASONIC_SPEC.md §2):
    AA 55 | 0x02 | 0x09 | FL_cm u16LE | FC_cm u16LE | FR_cm u16LE | BOT_cm u16LE | flags u8 | checksum u8
    checksum = (AA 55 起至 payload 末) 字节和 & 0xFF   —— 与底盘 0x01 帧同约定
    0xFFFF 或 flags 位=0 ⇒ 该路无效; flags bit0..3 = FL/FC/FR/BOT 有效位

用途: 挂到师兄 wheel_board_bridge_node (串口唯一持有者), 解析后发布
    mechdog_ultrasonic/msg/UltrasonicArray 到 /ultrasonic; safety_node 零改动。

零 ROS 依赖 (纯 Python), 可独立单测:
    python3 ultrasonic_serial_parser.py   # 自测打印 PASS
"""

import struct

HEADER = bytes([0xAA, 0x55])
CMD_ULTRASONIC = 0x02
PAYLOAD_LEN = 0x09
FRAME_TOTAL = 2 + 1 + 1 + PAYLOAD_LEN + 1   # 14
INVALID_CM = 400.0                          # 无效读数统一 400cm (valid=False 为准)

# 字段顺序 = 算法库 get_ultrasonic_layout(): 左前/正前/右前/底部
NAMES = ('front_left', 'front_center', 'front_right', 'bottom')


def build_frame(front_left_cm, front_center_cm, front_right_cm, bottom_cm,
                front_left_valid=True, front_center_valid=True,
                front_right_valid=True, bottom_valid=True) -> bytes:
    """构造一帧 0x02 (供主机侧仿真/固件联调对照)"""
    flags = (1 << 0 if front_left_valid else 0) | (1 << 1 if front_center_valid else 0) | \
            (1 << 2 if front_right_valid else 0) | (1 << 3 if bottom_valid else 0)
    payload = struct.pack(
        '<HHHHB',
        int(front_left_cm) & 0xFFFF, int(front_center_cm) & 0xFFFF,
        int(front_right_cm) & 0xFFFF, int(bottom_cm) & 0xFFFF, flags)
    frame_no_cs = HEADER + bytes([CMD_ULTRASONIC, PAYLOAD_LEN]) + payload
    return frame_no_cs + bytes([sum(frame_no_cs) & 0xFF])


class UltrasonicFrameParser:
    """流式解析: feed(bytes) -> list[dict]; 自动处理分片/粘包/坏校验/坏帧头。"""

    def __init__(self):
        self._buf = bytearray()
        self._seq = 0

    def _parse_payload(self, payload: bytes, frame_seq: int):
        fl, fc, fr, bot, flags = struct.unpack('<HHHHB', payload)
        out = {'seq': frame_seq}
        values = (fl, fc, fr, bot)
        for i, name in enumerate(NAMES):
            valid = bool(flags & (1 << i)) and values[i] != 0xFFFF
            out[name + '_cm'] = float(values[i]) if valid else INVALID_CM
            out[name + '_valid'] = valid
        return out

    def feed(self, data: bytes):
        self._buf.extend(data)
        frames = []
        while True:
            idx = self._buf.find(HEADER)
            if idx < 0:
                # 保留末尾 1 字节, 防帧头跨分片 (AA 在上一包末尾)
                if len(self._buf) > 1:
                    del self._buf[:-1]
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < 4:          # AA 55 CMD LEN
                break
            if self._buf[2] != CMD_ULTRASONIC:
                del self._buf[:2]
                continue
            if self._buf[3] != PAYLOAD_LEN:
                del self._buf[:2]
                continue
            if len(self._buf) < FRAME_TOTAL:
                break
            payload = bytes(self._buf[4:4 + PAYLOAD_LEN])
            checksum = self._buf[4 + PAYLOAD_LEN]
            if (sum(self._buf[:4 + PAYLOAD_LEN]) & 0xFF) != checksum:
                del self._buf[:2]           # 校验失败: 丢掉疑似帧头, 重新搜索
                continue
            del self._buf[:FRAME_TOTAL]
            frames.append(self._parse_payload(payload, self._seq))
            self._seq += 1
        return frames


if __name__ == '__main__':
    # ---- 自测 ----
    p = UltrasonicFrameParser()

    frame = build_frame(25, 300, 0xFFFF, 15,
                        front_left_valid=True, front_center_valid=True,
                        front_right_valid=False, bottom_valid=True)
    got = p.feed(frame)
    assert len(got) == 1, got
    f = got[0]
    assert f['front_left_cm'] == 25.0 and f['front_left_valid'] is True
    assert f['front_center_cm'] == 300.0
    assert f['front_right_valid'] is False and f['front_right_cm'] == INVALID_CM
    assert f['bottom_cm'] == 15.0 and f['bottom_valid'] is True
    assert f['seq'] == 0

    # 分片 + 粘包
    p2 = UltrasonicFrameParser()
    f1 = build_frame(10, 20, 30, 40)
    f2 = build_frame(50, 60, 70, 80)
    chunk = f1[:5]                  # 故意从帧中间切开
    got = p2.feed(chunk)
    assert len(got) == 0, '分片未完成不应有帧'
    got = p2.feed(f1[5:] + f2[:10])
    assert len(got) == 1 and got[0]['bottom_cm'] == 40.0
    got = p2.feed(f2[10:])
    assert len(got) == 1 and got[0]['bottom_cm'] == 80.0 and got[0]['seq'] == 1

    # 坏校验丢弃
    p3 = UltrasonicFrameParser()
    bad = bytearray(build_frame(1, 2, 3, 4))
    bad[-1] ^= 0xFF
    assert p3.feed(bytes(bad)) == []
    # 坏校验后跟好帧仍能解析
    good = build_frame(5, 6, 7, 8)
    got = p3.feed(bytes(bad) + good)
    assert len(got) == 1 and got[0]['front_left_cm'] == 5.0

    print('PASS: ultrasonic_serial_parser self-test OK')

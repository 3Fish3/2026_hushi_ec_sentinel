import sys, time
sys.path.append('source/boards/sentinel_f407/tools')
sys.path.append('source/robots/sentinel/host')
import serial
from board_monitor import encode_frame, Decoder
from sentinel_capture import decode_page

port = serial.Serial('/dev/ttyUSB0', 1000000, timeout=0.03)
decoder = Decoder()
seq = 0

def fetch(page):
    global seq
    seq = (seq + 1) & 0xff
    port.write(encode_frame(0x30, seq, bytes([page])))
    t0 = time.time()
    while time.time() - t0 < 0.05:
        chunk = port.read(128)
        if chunk:
            for kind, f_seq, payload in decoder.feed(chunk):
                if kind == 0xb0:
                    return decode_page(payload)
    return None

print("=== 云台关节与 IMU 实时核验 (持续 20 秒) ===")
print("请在 20 秒内依次轻轻触碰/转动各机构:")
print("  1. 轻轻转动【主大云台】 -> 观察 main_yaw 变化")
print("  2. 轻轻转动【发射小云台】 -> 观察 fire_yaw 变化")
print("  3. 轻轻托动【pitch机构】 -> 观察 pitch_pos 与 IMU 变化\n")

base_main, base_fire, base_pitch = None, None, None
start_time = time.time()
last_print = 0

while time.time() - start_time < 20.0:
    p3 = fetch(3)
    p5 = fetch(5)
    p6 = fetch(6)
    
    m_yaw = p3['motors'][0]['encoder'] if p3 else None
    f_yaw = p3['motors'][1]['encoder'] if p3 else None
    pitch_p = None
    if p5:
        for m in p5['motors']:
            if m['kind'] == 4:
                pitch_p = m.get('p16')
    imu = p6.get('imu') if p6 else {}
    
    if base_main is None and m_yaw is not None: base_main = m_yaw
    if base_fire is None and f_yaw is not None: base_fire = f_yaw
    if base_pitch is None and pitch_p is not None: base_pitch = pitch_p

    now = time.time()
    if now - last_print > 0.4:
        last_print = now
        rem = 20.0 - (now - start_time)
        d_main = (m_yaw - base_main) if (m_yaw is not None and base_main is not None) else 0
        d_fire = (f_yaw - base_fire) if (f_yaw is not None and base_fire is not None) else 0
        d_pitch = (pitch_p - base_pitch) if (pitch_p is not None and base_pitch is not None) else 0
        
        # 判断谁在运动
        tag_main = " << 正在转动!" if abs(d_main) > 30 else ""
        tag_fire = " << 正在转动!" if abs(d_fire) > 30 else ""
        tag_pitch = " << 正在托动!" if abs(d_pitch) > 100 else ""
        
        gx, gy, gz = imu.get('gx', 0), imu.get('gy', 0), imu.get('gz', 0)
        ax, ay, az = imu.get('ax', 0), imu.get('ay', 0), imu.get('az', 0)
        
        print(f"[{rem:4.1f}s] main_yaw={m_yaw:<4d}(Δ{d_main:+4d}){tag_main:<15s} | "
              f"fire_yaw={f_yaw:<4d}(Δ{d_fire:+4d}){tag_fire:<15s} | "
              f"pitch={pitch_p}(Δ{d_pitch:+4d}){tag_pitch:<15s} | "
              f"IMU_Gz={gz:+.2f}")

port.close()
print("\n核验监测结束。")

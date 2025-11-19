# can_utils.py
import time, can

MASTER_ID = 0x00
P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -44.0, 44.0
T_MIN, T_MAX = -17.0, 17.0

def _u16_to_float(x, x_min, x_max):
    return (x / 65535.0) * (x_max - x_min) + x_min

def parse_motor_feedback(msg):
    if msg is None or len(msg.data) < 8:
        return None
    data = bytes(msg.data)
    angle   = int.from_bytes(data[0:2], 'big')
    velocity= int.from_bytes(data[2:4], 'big')
    torque  = int.from_bytes(data[4:6], 'big')
    temp    = int.from_bytes(data[6:8], 'big')
    return {
        'angle':    _u16_to_float(angle,   P_MIN, P_MAX),
        'velocity': _u16_to_float(velocity,V_MIN, V_MAX),
        'torque':   _u16_to_float(torque,  T_MIN, T_MAX),
        'temp':     temp / 10.0
    }

def send_can(canif, arbitration_id, data=b'\x00'*8):
    try:
        if hasattr(canif, "send_message"):
            canif.send_message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        else:
            canif.bus.send(can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True))
    except Exception as e:
        print(f"[WARN] CAN 송신 실패: {e}")

def request_feedback(canif, motor_id, master_id=MASTER_ID):
    req_id = (0x02 << 24) | (master_id << 8) | motor_id
    send_can(canif, req_id, b'\x00' * 8)

def get_current_angle(canif, motor_id, timeout=0.4, master_id=MASTER_ID):
    t_start = time.time()
    request_feedback(canif, motor_id, master_id=master_id)

    while time.time() - t_start < timeout:
        msg = canif.get_message(timeout=0.05)
        if not msg:
            continue
        rid    = msg.arbitration_id
        rtype  = (rid >> 24) & 0x3F
        rmotor = (rid >> 8)  & 0xFF
        if (getattr(msg, "is_extended_id", True) is False) or rtype != 0x02 or rmotor != motor_id:
            continue

        fb = parse_motor_feedback(msg)
        if fb and 'angle' in fb:
            ang = fb['angle']
            print(f"[FB] id={motor_id} angle={ang:.3f} rad")
            return ang
    raise RuntimeError(f"모터(id={motor_id}) 피드백 수신 실패")

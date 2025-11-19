# gains.py
from config import DEFAULT_KP, DEFAULT_KD, DEFAULT_TORQUE

# 모터별 게인/토크 오버라이드
GAINS_OVERRIDES = {
    1:  {'kp': 5.0, 'kd': 1.0,  'torque': 0.6},
    6:  {'kp': 5.0, 'kd': 1.0,  'torque': 0.6},

    2:  {'kp': 5.0, 'kd': 1.0,  'torque': 0.0},
    7:  {'kp': 5.0, 'kd': 1.0,  'torque': 0.0},

    3:  {'kp': 6.0, 'kd': 1.0,  'torque': 0.2},
    8:  {'kp': 6.0, 'kd': 1.0,  'torque': 0.2},

    4:  {'kp': 10.0, 'kd': 1.0,  'torque': 2.0},
    9:  {'kp': 10.0, 'kd': 1.0,  'torque': 2.0},

}

def get_motor_gains(motor_id: int):
    g = GAINS_OVERRIDES.get(motor_id)
    if g:
        return g.get('kp', DEFAULT_KP), g.get('kd', DEFAULT_KD), g.get('torque', DEFAULT_TORQUE)
    return DEFAULT_KP, DEFAULT_KD, DEFAULT_TORQUE

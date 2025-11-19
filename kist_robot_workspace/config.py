# config.py

# 전역 스위치
ENABLE_CAN0 = True
ENABLE_CAN1 = True
DRY_RUN = False
RUN_CAN_MOTION = True      # 몸(관절/CAN 쪽) 모션을 실행할지?
RUN_HAND_MOTION = True     # 손(USB/시리얼) 모션을 실행할지?

# can0(우 + 목), can1(좌 + 허리)
MOTOR_IDS_CAN0 = [1, 2, 3, 4, 5, 11, 12]
MOTOR_IDS_CAN1 = [6, 7, 8, 9, 10, 13, 14]

# 플레이 속도/타이밍
P_SPEED = 1.0          # 재생 속도 스케일(>1 빠름)
DT_STEP = 0.1 / P_SPEED

# MIT 기본 게인/토크
DEFAULT_TORQUE = 0.0
DEFAULT_KP = 2.0
DEFAULT_KD = 1.0

# 손 컨트롤러 설정
# 이제 복수 개 포트를 지원한다.
# 예: 오른손 /dev/ttyACM0, 왼손 /dev/ttyACM1
HAND_PORTS = [
    {
        "role": "right",                # "left" / "right" 등
        "port": "/dev/ttyACM2",
        "baudrate": 1_000_000,
        "timeout": 0.05,
    },
    {
        "role": "left",
        "port": "/dev/ttyACM3",
        "baudrate": 1_000_000,
        "timeout": 0.05, 
    },
]

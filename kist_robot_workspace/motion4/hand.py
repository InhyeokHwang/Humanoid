import time
from hand_functions import (
    set_speeds, set_calibration, torque_enable_right, torque_enable_left,
    OpenHand, CloseHand, OpenHand_Progressive, SpreadHand, ClenchHand,
    Index_Pointing, Nonono, Perfect, Victory, Pinched, Scissors, FingerUp
)

# 오른손 전용 시퀀스
def play_right(controller, stop_event):
    # 필요하면 여기서 오른손만 다른 속도로 세팅 가능
    # set_speeds(max_speed=7, close_speed=3)

    torque_enable_right(controller, enable=True)

    if stop_event.is_set(): return
    CloseHand(controller, hand_sel=1); time.sleep(1.0)
    OpenHand(controller, hand_sel=1); time.sleep(3.0)
    CloseHand(controller, hand_sel=1); time.sleep(3.0)
    OpenHand(controller, hand_sel=1); time.sleep(3.0)



# 왼손 전용 시퀀스
def play_left(controller, stop_event):
    # 이건 왼손만 하는 동작들
    torque_enable_left(controller, enable=True)

    if stop_event.is_set(): return
    CloseHand(controller, hand_sel=2); time.sleep(1.0)
    OpenHand(controller, hand_sel=2); time.sleep(3.0)
    CloseHand(controller, hand_sel=2); time.sleep(6.0)
    OpenHand(controller, hand_sel=2); time.sleep(3.0)
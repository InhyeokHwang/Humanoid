# hand_functions.py (DEBUG INSTRUMENTED VERSION)
# 공용 손 제스처 유틸. 각 motionN/hand.py에서 import 해서 사용.
import time
import math

# ====== 기본 설정 ======
_MAX_SPEED = 7
_CLOSE_SPEED = 3

# 캘리브레이션(기본값은 제공, 필요 시 set_calibration()으로 교체)
_MIDDLE_POS_1 = [0, 0, 0, 0, 0, 0, 0, 0]  # Right hand calibration
_MIDDLE_POS_2 = [0, 0, 0, 0, 0, 0, 0, 0]  # Left hand calibration

# 서보 ID 매핑 
# Hand=1 -> 오른손, Hand=2 -> 왼손
# 각 finger마다 (2개 서보) 튜플
_SERVO_IDS = {
    1: {  # Right hand
        "index":  (1, 2),
        "middle": (3, 4),
        "ring":   (5, 6),
        "thumb":  (7, 8),
    },
    2: {  # Left hand
        "index":  (9, 10),
        "middle": (11, 12),
        "ring":   (13, 14),
        "thumb":  (15, 16),
    },
}

# ====== 퍼블릭 API: 설정 ======

def set_speeds(max_speed=None, close_speed=None):
    """제스처 기본 속도 설정"""
    global _MAX_SPEED, _CLOSE_SPEED
    if max_speed is not None:
        _MAX_SPEED = int(max_speed)
    if close_speed is not None:
        _CLOSE_SPEED = int(close_speed)


def set_calibration(middle_pos_1=None, middle_pos_2=None):
    """캘리브레이션(중간 각) 교체: 각 8원소 리스트"""
    global _MIDDLE_POS_1, _MIDDLE_POS_2
    if middle_pos_1:
        assert len(middle_pos_1) == 8, "middle_pos_1 must have 8 elements"
        _MIDDLE_POS_1 = list(middle_pos_1)
    if middle_pos_2:
        assert len(middle_pos_2) == 8, "middle_pos_2 must have 8 elements"
        _MIDDLE_POS_2 = list(middle_pos_2)

# ====== 내부 유틸 ======

def _deg2rad(x):  # numpy 없이 동작
    return math.radians(x)


def _write_pair(controller, sid1, sid2, pos1_rad, pos2_rad, speed):
    """
    실제로 서보 두 개에 속도/목표각 명령을 쏘는 가장 하위 레벨 함수.
    여기서 죽으면 거의 확정적으로 해당 ID가 존재 안 하거나 통신이 깨진 거야.
    """

    try:
        controller.write_goal_speed(sid1, speed)
    except Exception as e:
        raise
    time.sleep(0.0002)

    try:
        controller.write_goal_speed(sid2, speed)
    except Exception as e:
        raise
    time.sleep(0.0002)

    try:
        print(f"[DEBUG][_write_pair] write_goal_position({sid1}, {pos1_rad})")
        controller.write_goal_position(sid1, pos1_rad)
    except Exception as e:
        print(f"[ERROR][_write_pair] write_goal_position sid1={sid1} failed: {e}")
        raise

    try:
        print(f"[DEBUG][_write_pair] write_goal_position({sid2}, {pos2_rad})")
        controller.write_goal_position(sid2, pos2_rad)
    except Exception as e:
        print(f"[ERROR][_write_pair] write_goal_position sid2={sid2} failed: {e}")
        raise

    time.sleep(0.0005)
    print(f"[DEBUG][_write_pair] done sid1={sid1}, sid2={sid2}")


def _middle_pos_for(hand):
    mp = _MIDDLE_POS_1 if hand == 1 else _MIDDLE_POS_2
    return mp


def _ids_for(hand, finger_name):
    pair = _SERVO_IDS[hand][finger_name]
    return pair


def _maybe_stop(stop_event):
    stopped = (stop_event is not None) and getattr(stop_event, "is_set", lambda: False)()
    return stopped


# ====== 저수준 이동 함수 (원 코드 시그니처 그대로) ======

def Move_Index(controller, Angle_1, Angle_2, Speed, Hand):
    print(f"[DEBUG][Move_Index] Hand={Hand}, Angle_1={Angle_1}, Angle_2={Angle_2}, Speed={Speed}")
    mp = _middle_pos_for(Hand)
    sid1, sid2 = _ids_for(Hand, "index")
    pos1 = _deg2rad(mp[0] + Angle_1)
    pos2 = _deg2rad(mp[1] + Angle_2)
    print(f"[DEBUG][Move_Index] sid1={sid1}, sid2={sid2}, pos1(rad)={pos1}, pos2(rad)={pos2}")
    _write_pair(controller, sid1, sid2, pos1, pos2, Speed)


def Move_Middle(controller, Angle_1, Angle_2, Speed, Hand):
    print(f"[DEBUG][Move_Middle] Hand={Hand}, Angle_1={Angle_1}, Angle_2={Angle_2}, Speed={Speed}")
    mp = _middle_pos_for(Hand)
    sid1, sid2 = _ids_for(Hand, "middle")
    pos1 = _deg2rad(mp[2] + Angle_1)
    pos2 = _deg2rad(mp[3] + Angle_2)
    print(f"[DEBUG][Move_Middle] sid1={sid1}, sid2={sid2}, pos1(rad)={pos1}, pos2(rad)={pos2}")
    _write_pair(controller, sid1, sid2, pos1, pos2, Speed)


def Move_Ring(controller, Angle_1, Angle_2, Speed, Hand):
    print(f"[DEBUG][Move_Ring] Hand={Hand}, Angle_1={Angle_1}, Angle_2={Angle_2}, Speed={Speed}")
    mp = _middle_pos_for(Hand)
    sid1, sid2 = _ids_for(Hand, "ring")
    pos1 = _deg2rad(mp[4] + Angle_1)
    pos2 = _deg2rad(mp[5] + Angle_2)
    print(f"[DEBUG][Move_Ring] sid1={sid1}, sid2={sid2}, pos1(rad)={pos1}, pos2(rad)={pos2}")
    _write_pair(controller, sid1, sid2, pos1, pos2, Speed)


def Move_Thumb(controller, Angle_1, Angle_2, Speed, Hand):
    print(f"[DEBUG][Move_Thumb] Hand={Hand}, Angle_1={Angle_1}, Angle_2={Angle_2}, Speed={Speed}")
    mp = _middle_pos_for(Hand)
    sid1, sid2 = _ids_for(Hand, "thumb")
    pos1 = _deg2rad(mp[6] + Angle_1)
    pos2 = _deg2rad(mp[7] + Angle_2)
    print(f"[DEBUG][Move_Thumb] sid1={sid1}, sid2={sid2}, pos1(rad)={pos1}, pos2(rad)={pos2}")
    _write_pair(controller, sid1, sid2, pos1, pos2, Speed)


# ====== 토크 유틸 ======

def torque_enable_right(controller, enable=True):
    """오른손: ID 1만 활성화"""
    print(f"[DEBUG][torque_enable_right] enable={enable}, controller={controller}")
    try:
        print("[DEBUG][torque_enable_right] controller.write_torque_enable(1, {enable})")
        controller.write_torque_enable(1, enable)
        print("[DEBUG][torque_enable_right] write_torque_enable(1) OK")
    except Exception as e:
        print(f"[ERROR][torque_enable_right] write_torque_enable(1) failed: {e}")
        raise
    time.sleep(0.01)


def torque_enable_left(controller, enable=True):
    """왼손: ID 9만 활성화"""
    print(f"[DEBUG][torque_enable_left] enable={enable}, controller={controller}")
    try:
        print("[DEBUG][torque_enable_left] controller.write_torque_enable(9, {enable})")
        controller.write_torque_enable(9, enable)
        print("[DEBUG][torque_enable_left] write_torque_enable(9) OK")
    except Exception as e:
        print(f"[ERROR][torque_enable_left] write_torque_enable(9) failed: {e}")
        raise
    time.sleep(0.05)


# ====== 고수준 제스처 (원 코드 동작을 함수화) ======
# hand_sel: 0=양손, 1=오른손만, 2=왼손만

def OpenHand(controller, hand_sel=0):
    print(f"[DEBUG][OpenHand] hand_sel={hand_sel}")
    for h in (1, 2) if hand_sel == 0 else (hand_sel,):
        print(f"[DEBUG][OpenHand] -> Hand={h}")
        Move_Index (controller, -35, 35, _MAX_SPEED, h)
        time.sleep(0.1)
        Move_Middle(controller, -35, 35, _MAX_SPEED, h)
        time.sleep(0.1)
        Move_Ring  (controller, -35, 35, _MAX_SPEED, h)
        time.sleep(0.1)
        Move_Thumb (controller, -35, 35, _MAX_SPEED, h)
        time.sleep(0.1)


def CloseHand(controller, hand_sel=0):
    print(f"[DEBUG][CloseHand] hand_sel={hand_sel}")
    for h in (1, 2) if hand_sel == 0 else (hand_sel,):
        print(f"[DEBUG][CloseHand] -> Hand={h}")
        Move_Index (controller, 90, -90, _CLOSE_SPEED, h)
        time.sleep(0.05)
        Move_Middle(controller, 90, -90, _CLOSE_SPEED, h)
        time.sleep(0.05)
        Move_Ring  (controller, 90, -90, _CLOSE_SPEED, h)
        time.sleep(0.05)
        Move_Thumb (controller, 90, -90, _CLOSE_SPEED + 1, h)
        time.sleep(0.05)


def OpenHand_Progressive(controller, hand_sel=0):
    print(f"[DEBUG][OpenHand_Progressive] hand_sel={hand_sel}")
    for h in (1, 2) if hand_sel == 0 else (hand_sel,):
        print(f"[DEBUG][OpenHand_Progressive] -> Hand={h}")
        Move_Index (controller, -35, 35, _MAX_SPEED-2, h)
        time.sleep(0.25)
        Move_Middle(controller, -35, 35, _MAX_SPEED-2, h)
        time.sleep(0.25)
        Move_Ring  (controller, -35, 35, _MAX_SPEED-2, h)
        time.sleep(0.25)
        Move_Thumb (controller, -35, 35, _MAX_SPEED-2, h)
        time.sleep(0.25)


def SpreadHand(controller):
    print("[DEBUG][SpreadHand] start")
    # 양손 서로 반대 동작(원 코드 재현)
    Move_Index (controller,  4,  90, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Index (controller, -90,  0, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Middle(controller, -32, 32, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Middle(controller, -32, 32, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Ring  (controller, -90, -4, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Ring  (controller,  -4, 90, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Thumb (controller, -90, -4, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Thumb (controller,  -4, 90, _MAX_SPEED, 2)
    time.sleep(0.05)
    print("[DEBUG][SpreadHand] done")


def ClenchHand(controller):
    print("[DEBUG][ClenchHand] start")
    Move_Index (controller, -60,  0, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Index (controller,   0, 60, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Middle(controller, -35, 35, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Middle(controller, -35, 35, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Ring  (controller,   0, 70, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Ring  (controller, -70,  0, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Thumb (controller,  -4, 90, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Thumb (controller, -90, -4, _MAX_SPEED, 2)
    time.sleep(0.05)
    print("[DEBUG][ClenchHand] done")


def Index_Pointing(controller, hand_sel=0):
    print(f"[DEBUG][Index_Pointing] hand_sel={hand_sel}")
    for h in (1, 2) if hand_sel == 0 else (hand_sel,):
        print(f"[DEBUG][Index_Pointing] -> Hand={h}")
        Move_Index (controller, -40,  40, _MAX_SPEED, h)
        time.sleep(0.05)
        Move_Middle(controller,  90, -90, _MAX_SPEED, h)
        time.sleep(0.05)
        Move_Ring  (controller,  90, -90, _MAX_SPEED, h)
        time.sleep(0.05)
        Move_Thumb (controller,  90, -90, _MAX_SPEED, h)
        time.sleep(0.05)
    print("[DEBUG][Index_Pointing] done")



def Nonono(controller):
    print("[DEBUG][Nonono] start")
    for _ in range(3):
        time.sleep(0.05)
        Move_Index(controller, -10, 80, _MAX_SPEED, 1)
        time.sleep(0.05)
        Move_Index(controller, -10, 80, _MAX_SPEED, 2)
        time.sleep(0.05)
        Move_Index(controller, -80, 10, _MAX_SPEED, 1)
        time.sleep(0.05)
        Move_Index(controller, -80, 10, _MAX_SPEED, 2)
        time.sleep(0.05)
    Move_Index(controller, -35, 35, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Index(controller, -35, 35, _MAX_SPEED, 2)
    time.sleep(0.05)
    print("[DEBUG][Nonono] done")


def Perfect(controller):
    print("[DEBUG][Perfect] start")
    Move_Index (controller, 55, -55, _MAX_SPEED-3, 1)
    time.sleep(0.05)
    Move_Index (controller, 55, -55, _MAX_SPEED-3, 2)
    time.sleep(0.05)
    Move_Middle(controller,  0,   0, _MAX_SPEED,    1)
    time.sleep(0.05)
    Move_Middle(controller,  0,   0, _MAX_SPEED,    2)
    time.sleep(0.05)
    Move_Ring  (controller, -20, 20, _MAX_SPEED,    1)
    time.sleep(0.05)
    Move_Ring  (controller, -20, 20, _MAX_SPEED,    2)
    time.sleep(0.05)
    Move_Thumb (controller, 85,  10, _MAX_SPEED,    1)
    time.sleep(0.05)
    Move_Thumb (controller,-10, -85, _MAX_SPEED,    2)
    time.sleep(0.05)
    print("[DEBUG][Perfect] done")


def Victory(controller):
    print("[DEBUG][Victory] start")
    Move_Index (controller, -15, 65, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Index (controller, -65, 15, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Middle(controller, -65, 15, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Middle(controller, -15, 65, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Ring  (controller,  90,-90, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Ring  (controller,  90,-90, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Thumb (controller,  90,-90, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Thumb (controller,  90,-90, _MAX_SPEED, 2)
    time.sleep(0.05)
    print("[DEBUG][Victory] done")


def Pinched(controller):
    print("[DEBUG][Pinched] start")
    Move_Index (controller, 90, -90, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Index (controller, 90, -90, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Middle(controller, 90, -90, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Middle(controller, 90, -90, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Ring  (controller, 90, -90, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Ring  (controller, 90, -90, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Thumb (controller,  5, -75, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Thumb (controller, 75,  -5, _MAX_SPEED, 2)
    time.sleep(0.05)
    print("[DEBUG][Pinched] done")


def Scissors(controller):
    print("[DEBUG][Scissors] start")
    Victory(controller)
    for _ in range(3):
        Move_Index (controller, -50, 20, _MAX_SPEED, 1)
        time.sleep(0.05)
        Move_Middle(controller, -20, 50, _MAX_SPEED, 1)
        time.sleep(0.05)

        Move_Index (controller, -20, 50, _MAX_SPEED, 2)
        time.sleep(0.05)
        Move_Middle(controller, -50, 20, _MAX_SPEED, 2)
        time.sleep(0.05)

        Move_Index (controller, -15, 65, _MAX_SPEED, 1)
        time.sleep(0.05)
        Move_Middle(controller, -65, 15, _MAX_SPEED, 1)
        time.sleep(0.05)

        Move_Index (controller, -65, 15, _MAX_SPEED, 2)
        time.sleep(0.05)
        Move_Middle(controller, -15, 65, _MAX_SPEED, 2)
        time.sleep(0.05)
    print("[DEBUG][Scissors] done")


def FingerUp(controller):  # (원 코드 'Fuck' 동작—이름만 순화)
    print("[DEBUG][FingerUp] start")
    Move_Index (controller, 90, -90, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Index (controller, 90, -90, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Middle(controller, -35, 35, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Middle(controller, -35, 35, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Ring  (controller, 90, -90, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Ring  (controller, 90, -90, _MAX_SPEED, 2)
    time.sleep(0.05)
    Move_Thumb (controller,  5, -75, _MAX_SPEED, 1)
    time.sleep(0.05)
    Move_Thumb (controller, 75,  -5, _MAX_SPEED, 2)
    time.sleep(0.05)
    print("[DEBUG][FingerUp] done")

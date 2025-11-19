import time
import argparse
import can
from typing import Optional

from config import (
    ENABLE_CAN0, ENABLE_CAN1, DRY_RUN, DT_STEP,
    RUN_CAN_MOTION, RUN_HAND_MOTION,
    MOTOR_IDS_CAN0, MOTOR_IDS_CAN1,
)
from gains import get_motor_gains
from utils import clamp, safe_get
from keyboard import wait_for_spacebar, wait_for_choice
from motion_loader import discover_motions
from hand_runtime import make_hand_controllers, HandThread
from ramps import run_startup_ramp, run_shutdown_ramp
from can_utils import get_current_angle, P_MIN, P_MAX, V_MIN, V_MAX
from canwrapper import CanWrapper
from robstride_motor import RobstrideMotor


def init_can_side():
    if not RUN_CAN_MOTION:
        print("[INIT] RUN_CAN_MOTION=False → CAN 관련 전부 비활성화")
        return None, None, [], [], [], []

    can0 = CanWrapper(channel='can0', bitrate=1000000) if ENABLE_CAN0 else None
    can1 = CanWrapper(channel='can1', bitrate=1000000) if ENABLE_CAN1 else None

    if can0 and not can0.start():
        print("[INIT] CAN0 open failed!")
        can0 = None
    if can1 and not can1.start():
        print("[INIT] CAN1 open failed!")
        can1 = None

    if not can0 and not can1:
        print("[INIT] No CAN bus active (ENABLE_CAN0/1 are off or failed)")
        return None, None, [], [], [], []

    motors0 = [RobstrideMotor(can0.bus, motor_id=m) for m in MOTOR_IDS_CAN0] if can0 else []
    motors1 = [RobstrideMotor(can1.bus, motor_id=m) for m in MOTOR_IDS_CAN1] if can1 else []

    for m in (motors0 + motors1):
        if not DRY_RUN:
            m.connect()
    for m in (motors0 + motors1):
        if not DRY_RUN:
            m.set_run_mode(RobstrideMotor.MODE_MIT)
        time.sleep(0.01)

    theta0, theta1 = [], []
    for m in motors0:
        try:
            theta0.append(get_current_angle(can0, m.motor_id))
        except Exception as e:
            print(f"[WARN] id={m.motor_id} 각도 취득 실패: {e} → 0.0 대체")
            theta0.append(0.0)
        time.sleep(0.01)

    for m in motors1:
        try:
            theta1.append(get_current_angle(can1, m.motor_id))
        except Exception as e:
            print(f"[WARN] id={m.motor_id} 각도 취득 실패: {e} → 0.0 대체")
            theta1.append(0.0)
        time.sleep(0.01)

    for m in (motors0 + motors1):
        if not DRY_RUN:
            m.enable_motor()
    time.sleep(0.5)

    return can0, can1, motors0, motors1, theta0, theta1


def shutdown_can_side(can0, can1, motors0, motors1, theta0, theta1):
    if not RUN_CAN_MOTION:
        return

    try:
        run_shutdown_ramp(motors0, motors1, theta0, theta1, DRY_RUN=DRY_RUN)
    except Exception as e:
        print(f"[WARN] shutdown ramp 실패: {e}")

    for m in (motors0 + motors1):
        if not DRY_RUN:
            try:
                m.disable_motor()
            except Exception:
                pass
            try:
                m.disconnect()
            except Exception:
                pass

    if can0:
        can0.stop()
    if can1:
        can1.stop()


def init_hand_side():
    if not RUN_HAND_MOTION:
        print("[INIT] RUN_HAND_MOTION=False → 손 동작 비활성화")
        return []
    hand_ctrls = make_hand_controllers()
    return hand_ctrls


def shutdown_hand_side(hand_ctrls):
    if not RUN_HAND_MOTION:
        return
    for hc in hand_ctrls:
        ctrl = hc.get("controller")
        if ctrl and hasattr(ctrl, "close"):
            try:
                ctrl.close()
            except Exception:
                pass


def play_body_motion_for_steps(motors0, motors1, pos0, vel0, pos1, vel1, steps):
    for step in range(steps):
        pos_row0 = pos0[step] if step < len(pos0) else []
        vel_row0 = vel0[step] if step < len(vel0) else []
        pos_row1 = pos1[step] if step < len(pos1) else []
        vel_row1 = vel1[step] if step < len(vel1) else []

        if RUN_CAN_MOTION:
            for j, mtr in enumerate(motors0):
                p_cmd = clamp(safe_get(pos_row0, j, 0.0), P_MIN, P_MAX)
                v_cmd = clamp(safe_get(vel_row0, j, 0.0), V_MIN, V_MAX)
                if not DRY_RUN:
                    kp_i, kd_i, tq_i = get_motor_gains(mtr.motor_id)
                    mtr.send_operation_mode(
                        pos=p_cmd,
                        vel=v_cmd,
                        kp=kp_i,
                        kd=kd_i,
                        torque=tq_i,
                    )

            for j, mtr in enumerate(motors1):
                p_cmd = clamp(safe_get(pos_row1, j, 0.0), P_MIN, P_MAX)
                v_cmd = clamp(safe_get(vel_row1, j, 0.0), V_MIN, V_MAX)
                if not DRY_RUN:
                    kp_i, kd_i, tq_i = get_motor_gains(mtr.motor_id)
                    mtr.send_operation_mode(
                        pos=p_cmd,
                        vel=v_cmd,
                        kp=kp_i,
                        kd=kd_i,
                        torque=tq_i,
                    )

        time.sleep(DT_STEP)


from hand_runtime import HandThread  # ensure we use the version with role/port debug


def launch_hand_threads_for_motion(hand_ctrls, motion):
    if not RUN_HAND_MOTION:
        return []

    hand_threads = []

    for hc in hand_ctrls:
        role = hc["role"]
        ctrl = hc["controller"]
        port = hc["port"]

        if role == "right":
            entry_func = motion.get("hand_entry_right") or motion.get("hand_entry")
        elif role == "left":
            entry_func = motion.get("hand_entry_left") or motion.get("hand_entry")
        else:
            entry_func = motion.get("hand_entry")

        if ctrl and entry_func:
            th = HandThread(ctrl, entry_func, role=role, port=port)
            th.start()
            hand_threads.append(th)
        elif entry_func and not ctrl:
            print(f"[WARN] {motion['name']}: {role} 손 컨트롤러 없음 → 스킵")

    return hand_threads


def stop_hand_threads(hand_threads):
    for th in hand_threads:
        th.stop(join_timeout=2.0)


def run_once(motion_idx: Optional[int]):
    """
    한 번의 퍼포먼스 사이클:
    - CAN 초기화 (필요시)
    - 손 컨트롤러 초기화 (필요시)
    - 모션들 discover 후 순차 재생
    - shutdown 후 재시작 여부 묻기
    """

    can0, can1, motors0, motors1, theta0, theta1 = init_can_side()
    hand_ctrls = init_hand_side()

    try:
        # 모션 로드 (여기서 motion_idx 반영!)
        motions = discover_motions(motion_idx=motion_idx)
        print(f"[INFO] 발견된 모션 수: {len(motions)} → {[m['name'] for m in motions]}")

        # 몸 초기자세 ramp-up
        if RUN_CAN_MOTION and (motors0 or motors1):
            try:
                run_startup_ramp(motors0, motors1, theta0, theta1, DRY_RUN=DRY_RUN)
            except Exception as e:
                print(f"[WARN] startup ramp 실패: {e}")

        # 시작 대기
        if not wait_for_spacebar():
            return "restart"

        # 모션 순차 실행 (보통 하나일 거야, --motion 지정했을 때)
        for motion in motions:
            name = motion["name"]
            pos0, vel0 = motion["pos0"], motion["vel0"]
            pos1, vel1 = motion["pos1"], motion["vel1"]

            steps = max(len(pos0), len(pos1))
            print(f"\n[RUN] {name} 시작 (steps={steps})")

            hand_threads = launch_hand_threads_for_motion(hand_ctrls, motion)

            play_body_motion_for_steps(motors0, motors1, pos0, vel0, pos1, vel1, steps)

            stop_hand_threads(hand_threads)

            print(f"[DONE] {name} 완료")

        return "restart"

    except Exception as e:
        print(f"[ERROR] run_once loop: {e}")
        return "restart"

    finally:
        shutdown_can_side(can0, can1, motors0, motors1, theta0, theta1)
        shutdown_hand_side(hand_ctrls)
        choice = wait_for_choice("스페이스=종료, r=재시작")
        return choice


def main():
    # argparse 추가
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--motion",
        type=int,
        default=None,
        help="재생할 모션 번호 (예: --motion 3 → motion3 만 실행). 지정 없으면 전체 스캔.",
    )
    args = parser.parse_args()

    while True:
        result = run_once(motion_idx=args.motion)

        if result == "exit":
            print("[MAIN] 요청에 따라 종료합니다.")
            break
        elif result == "restart":
            print("[MAIN] 재시작 루프 진입합니다.")
            # 루프 계속 ⇒ 동일한 args.motion으로 다시 run_once 호출
            continue
        else:
            print(f"[MAIN] 알 수 없는 반환값 '{result}' → 안전 종료")
            break


if __name__ == "__main__":
    main()

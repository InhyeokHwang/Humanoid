# ramps.py
import time
from gains import get_motor_gains

def run_startup_ramp(motors0, motors1, theta0, theta1, DRY_RUN=False):
    n_step = 100
    target0 = [+0.078, +0.3, +0.045, -0.223, 0.000, -0.089, -0.051][:len(motors0)]
    target1 = [-0.116, -0.3, -0.142, +0.406, 0.000, -0.553, +0.632][:len(motors1)]
    for step in range(n_step):
        alpha = (step + 1) / n_step
        for m, th, tgt in zip(motors0, theta0, target0):
            if not DRY_RUN:
                pos = th*(1 - alpha) + tgt*alpha
                kp_i, kd_i, _ = get_motor_gains(m.motor_id)
                m.send_operation_mode(pos=pos, vel=0.0, kp=kp_i, kd=kd_i, torque=0.0)
        for m, th, tgt in zip(motors1, theta1, target1):
            if not DRY_RUN:
                pos = th*(1 - alpha) + tgt*alpha
                kp_i, kd_i, _ = get_motor_gains(m.motor_id)
                m.send_operation_mode(pos=pos, vel=0.0, kp=kp_i, kd=kd_i, torque=0.0)
        time.sleep(0.01)
    print("\n[START] 모터 시작")

def run_shutdown_ramp(motors0, motors1, theta0, theta1, DRY_RUN=False):
    target0 = [+0.078, +0.01, +0.045, -0.223, 0.000, -0.089, -0.051][:len(motors0)]
    target1 = [-0.116, -0.01, -0.142, +0.406, 0.000, -0.553, +0.632][:len(motors1)]

    n_step = 100
    for step in range(n_step):
        alpha = (step + 1) / n_step
        for m, th, tgt in zip(motors0, theta0, target0):
            if not DRY_RUN:
                pos = th*(1 - alpha) + tgt*alpha
                kp_i, kd_i, _ = get_motor_gains(m.motor_id)
                m.send_operation_mode(pos=pos, vel=0.0, kp=kp_i, kd=kd_i, torque=0.0)
        for m, th, tgt in zip(motors1, theta1, target1):
            if not DRY_RUN:
                pos = th*(1 - alpha) + tgt*alpha
                kp_i, kd_i, _ = get_motor_gains(m.motor_id)
                m.send_operation_mode(pos=pos, vel=0.0, kp=kp_i, kd=kd_i, torque=0.0)
        time.sleep(0.01)

    print("[DONE] 종료 복귀 완료.")

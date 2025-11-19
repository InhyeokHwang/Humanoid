# motion_loader.py
import importlib, importlib.util
from pathlib import Path
import math
from typing import Optional

from config import P_SPEED, MOTOR_IDS_CAN0, MOTOR_IDS_CAN1
from utils import concat_series

MOTIONS_BASE_DIR = Path(__file__).resolve().parent

def _try_import(prefix: str, subname: str):
    mod_name = f"{prefix}.{subname}"  # 패키지 import 우선
    try:
        return importlib.import_module(mod_name)
    except Exception:
        pass
    py_path = MOTIONS_BASE_DIR / prefix / f"{subname}.py"
    if not py_path.is_file():
        return None
    spec = importlib.util.spec_from_file_location(mod_name, str(py_path))
    if spec is None or spec.loader is None:
        return None
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)  # type: ignore
        return mod
    except Exception as e:
        print(f"[WARN] {py_path} import 실패: {e}")
        return None


def load_motion(prefix: str):
    mods = {
        "RH": _try_import(prefix, "values_right_arm"),
        "LH": _try_import(prefix, "values_left_arm"),
        "WA": _try_import(prefix, "values_body"),
        "NK": _try_import(prefix, "values_neck"),
    }
    if not all(mods.values()):
        return None

    RH_POS = getattr(mods["RH"], "positions_list", [])
    RH_VEL = getattr(mods["RH"], "velocities_list", [])
    LH_POS = getattr(mods["LH"], "positions_list", [])
    LH_VEL = getattr(mods["LH"], "velocities_list", [])
    WA_POS = getattr(mods["WA"], "positions_list", [])
    WA_VEL = getattr(mods["WA"], "velocities_list", [])
    NK_POS = getattr(mods["NK"], "positions_list", [])
    NK_VEL = getattr(mods["NK"], "velocities_list", [])

    RH_W = len(RH_POS[0]) if RH_POS else 0
    NK_W = len(NK_POS[0]) if NK_POS else 0
    LH_W = len(LH_POS[0]) if LH_POS else 0
    WA_W = len(WA_POS[0]) if WA_POS else 0

    pos0  = concat_series(RH_POS, NK_POS)  # can0: RH + NK
    vel0  = concat_series(RH_VEL, NK_VEL)
    pos1  = concat_series(LH_POS, WA_POS)  # can1: LH + WA
    vel1  = concat_series(LH_VEL, WA_VEL)

    if not pos0 or not pos0[0]:
        raise RuntimeError(f"[{prefix}] positions_list_0 비어 있음")
    if not pos1 or not pos1[0]:
        raise RuntimeError(f"[{prefix}] positions_list_1 비어 있음")

    assert RH_W + NK_W == len(MOTOR_IDS_CAN0), f"[{prefix}] can0 열({RH_W+NK_W}) != 모터({len(MOTOR_IDS_CAN0)})"
    assert LH_W + WA_W == len(MOTOR_IDS_CAN1), f"[{prefix}] can1 열({LH_W+WA_W}) != 모터({len(MOTOR_IDS_CAN1)})"

    # === 고정 인덱스 오프셋 ===
    import math
    OFF_RIGHT_ARM_2 = math.radians(17.2)    # pos0 열 1
    OFF_LEFT_ARM_2  = math.radians(-17.2)   # pos1 열 1
    OFF_BODY_1      = math.radians(-27.3)   # pos1 열 5
    OFF_BASE_BODY   = math.radians(17.4)    # pos1 열 6

    if pos0 and len(pos0[0]) > 1:
        pos0 = [[(x + OFF_RIGHT_ARM_2) if j == 1 else x for j, x in enumerate(row)] for row in pos0]
    if pos1 and len(pos1[0]) > 1:
        pos1 = [[(x + OFF_LEFT_ARM_2)  if j == 1 else x for j, x in enumerate(row)] for row in pos1]
    if pos1 and len(pos1[0]) > 5:
        pos1 = [[(x + OFF_BODY_1)      if j == 5 else x for j, x in enumerate(row)] for row in pos1]
    if pos1 and len(pos1[0]) > 6:
        pos1 = [[(x + OFF_BASE_BODY)   if j == 6 else x for j, x in enumerate(row)] for row in pos1]

    # 속도 스케일 (vel만)
    vel0 = [[v * P_SPEED for v in row] for row in vel0]
    vel1 = [[v * P_SPEED for v in row] for row in vel1]

    # 손 모듈 로드
    hand_mod = (_try_import(prefix, "hand") or _try_import(prefix, "hand_sequence"))

    hand_entry_right  = None
    hand_entry_left   = None
    hand_entry_common = None

    if hand_mod:
        hand_entry_right  = getattr(hand_mod, "play_right", None)
        hand_entry_left   = getattr(hand_mod, "play_left", None)
        hand_entry_common = getattr(hand_mod, "play_sequence", None)

    return {
        "name": prefix,
        "pos0": pos0,
        "vel0": vel0,
        "pos1": pos1,
        "vel1": vel1,
        "hand_entry_right": hand_entry_right or hand_entry_common,
        "hand_entry_left":  hand_entry_left  or hand_entry_common,
        "hand_entry": hand_entry_common,
    }


def discover_motions(motion_idx: Optional[int] = None, max_count: int = 200):
    """
    motion_idx가 주어지면 그 모션만 로드.
      예: motion_idx=3 -> 'motion3'만
    없으면 기존처럼 motion1..motionN까지 스캔.
    """
    motions = []

    if motion_idx is not None:
        prefix = f"motion{motion_idx}"
        m = load_motion(prefix)
        if m is None:
            raise RuntimeError(f"{prefix} 디렉토리의 모션 파일이 불완전합니다.")
        motions.append(m)
        print(f"[INFO] 선택 모션 로드: {prefix}")
        return motions

    # fallback: 전체 스캔
    for i in range(1, max_count + 1):
        prefix = f"motion{i}"
        if not (MOTIONS_BASE_DIR / prefix).is_dir():
            continue
        m = load_motion(prefix)
        if m is None:
            print(f"[WARN] {prefix} 디렉토리의 모션 파일이 불완전하여 건너뜀")
            continue
        motions.append(m)

    if not motions:
        raise RuntimeError("불러올 모션 디렉토리가 없습니다. (예: ./motion1/values_*.py)")
    return motions

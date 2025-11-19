#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv

CSV_PATH = "joint_sample.csv"   # ← CSV 경로를 여기에 지정하세요

# ---- 그룹별 컬럼 정의 (pos/vel 분리) ----
BODY_POS = ["pos:body_1", "pos:BASE_body"]
BODY_VEL = ["vel:body_1", "vel:BASE_body"]

RIGHT_POS = [f"pos:UPBODY_rightarm_{i}" for i in range(1, 6)]
RIGHT_VEL = [f"vel:UPBODY_rightarm_{i}" for i in range(1, 6)]

NECK_POS = ["pos:UPBODY_neck_1", "pos:UPBODY_neck_2"]
NECK_VEL = ["vel:UPBODY_neck_1", "vel:UPBODY_neck_2"]

LEFT_POS = [f"pos:UPBODY_leftarm_{i}" for i in range(1, 6)]
LEFT_VEL = [f"vel:UPBODY_leftarm_{i}" for i in range(1, 6)]

# ---- 생성할 파일 목록 ----
GROUPS = [
    ("values_body.py",      BODY_POS,  BODY_VEL),
    ("values_right_arm.py", RIGHT_POS, RIGHT_VEL),
    ("values_neck.py",      NECK_POS,  NECK_VEL),
    ("values_left_arm.py",  LEFT_POS,  LEFT_VEL),
]

# ---- alias (CSV 헤더 변형 대응) ----
ALIASES = {
    "pos:BASE__body": "pos:BASE_body",
    "vel:BASE__body": "vel:BASE_body",

    # neck 표기 호환
    "pos:UPBODY_neck1": "pos:UPBODY_neck_1",
    "pos:UPBODY_neck2": "pos:UPBODY_neck_2",
    "vel:UPBODY_neck1": "vel:UPBODY_neck_1",
    "vel:UPBODY_neck2": "vel:UPBODY_neck_2",

    # rightarm/leftarm 표기 호환
    **{f"pos:UPBODY_rightarm{i}": f"pos:UPBODY_rightarm_{i}" for i in range(1, 6)},
    **{f"vel:UPBODY_rightarm{i}": f"vel:UPBODY_rightarm_{i}" for i in range(1, 6)},
    **{f"pos:UPBODY_leftarm{i}":  f"pos:UPBODY_leftarm_{i}"  for i in range(1, 6)},
    **{f"vel:UPBODY_leftarm{i}":  f"vel:UPBODY_leftarm_{i}"  for i in range(1, 6)},
}

# ===========================
# CSV → 그룹별 리스트 변환
# ===========================
def _read_csv(path):
    with open(path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []
        rows = list(reader)
    return header, rows

def _apply_aliases(header):
    forward, reverse = {}, {}
    for h in header:
        std = ALIASES.get(h, h)
        forward[h] = std
        reverse[std] = h
    return forward, reverse

def _require_columns(header, required, reverse_map):
    missing = [c for c in required if c not in reverse_map]
    if missing:
        raise ValueError(f"CSV에 다음 컬럼이 없습니다: {missing}")

def _extract_matrix(rows, keys, reverse_map):
    mat = []
    for r in rows:
        row = []
        for key in keys:
            real_key = reverse_map[key]
            try:
                val = float(r[real_key])
            except Exception:
                val = 0.0
            row.append(val)
        mat.append(row)
    return mat

def _format_matrix(mat):
    lines = ["["]
    for row in mat:
        lines.append("  [" + ", ".join(f"{v:.6f}" for v in row) + "],")
    lines.append("]")
    return "\n".join(lines)

def _write_py(filename, pos_data, vel_data, src):
    pos_text = _format_matrix(pos_data)
    vel_text = _format_matrix(vel_data)

    with open(filename, "w", encoding="utf-8") as f:
        f.write(f"# Auto-generated from {src}\n")
        f.write(f"positions_list = {pos_text}\n\n")
        f.write(f"velocities_list  = {vel_text}\n")

    print(f"✅ {filename} 생성 완료")

def main():
    header, rows = _read_csv(CSV_PATH)
    _, reverse = _apply_aliases(header)

    need_all = set()
    for _, pos_keys, vel_keys in GROUPS:
        need_all.update(pos_keys + vel_keys)
    _require_columns(header, need_all, reverse)

    for out_file, pos_keys, vel_keys in GROUPS:
        pos_data = _extract_matrix(rows, pos_keys, reverse)
        vel_data = _extract_matrix(rows, vel_keys, reverse)
        _write_py(out_file, pos_data, vel_data, CSV_PATH)

if __name__ == "__main__":
    main()

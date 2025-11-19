# utils.py
from itertools import zip_longest

def concat_series(A, B):
    out = []
    for a, b in zip_longest(A, B, fillvalue=[]):
        ra = list(a) if a else []
        rb = list(b) if b else []
        out.append(ra + rb)
    return out

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def safe_get(row, idx, default=0.0):
    return row[idx] if idx < len(row) else default

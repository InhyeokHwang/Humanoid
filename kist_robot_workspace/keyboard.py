# keyboard.py
import sys, termios, tty, select

def _drain_stdin():
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            break
        sys.stdin.read(1)

def wait_for_spacebar(prompt="스페이스바를 누르면 시작, Q는 중단"):
    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        _drain_stdin()
        print(f"\n[READY] {prompt}")
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch == ' ':
                print("[START]")
                return True
            if ch in ('q', 'Q'):
                print("[ABORT]")
                return False
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)

def wait_for_choice(prompt="스페이스=종료, r=재시작, q=종료"):
    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        _drain_stdin()
        print(f"\n[READY] {prompt}")
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch == ' ':
                print("[CHOICE] 종료 선택")
                return "exit"
            if ch in ('r', 'R'):
                print("[CHOICE] 재시작 선택")
                return "restart"
            if ch in ('q', 'Q'):
                print("[CHOICE] 종료 선택(q)")
                return "exit"
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)

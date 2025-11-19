# hand_runtime.py
import threading
from config import HAND_PORTS

def _make_single_hand_controller(port, baudrate, timeout):
    """
    단일 포트에 대한 rustypot 컨트롤러를 생성한다.
    실패하면 None을 리턴한다.
    """
    try:
        from rustypot import Scs0009PyController
    except Exception as e:
        print(f"[WARN] rustypot 모듈 import 실패: {e}")
        return None

    try:
        ctrl = Scs0009PyController(
            serial_port=port,
            baudrate=baudrate,
            timeout=timeout,
        )
        return ctrl
    except Exception as e:
        print(f"[WARN] 손 컨트롤러 초기화 실패({port}): {e}")
        return None


def make_hand_controllers():
    """
    HAND_PORTS에 정의된 모든 손 포트를 열고,
    성공한 것들만 리스트로 반환한다.

    반환 형태 예:
    [
      {
        "role": "right",
        "controller": <Scs0009PyController>,
        "port": "/dev/ttyACM3",
      },
      {
        "role": "left",
        "controller": <Scs0009PyController>,
        "port": "/dev/ttyACM4",
      }
    ]
    """
    result = []
    for cfg in HAND_PORTS:
        role     = cfg.get("role", "unknown")
        port     = cfg["port"]
        baudrate = cfg["baudrate"]
        timeout  = cfg["timeout"]

        ctrl = _make_single_hand_controller(port, baudrate, timeout)
        if ctrl is not None:
            result.append({
                "role": role,
                "controller": ctrl,
                "port": port,
            })
        else:
            print(f"[WARN] {role}({port}) 컨트롤러 생성 실패 → 스킵")

    if not result:
        print("[WARN] 사용 가능한 손 컨트롤러가 없습니다.")
    return result


class HandThread:
    def __init__(self, controller, entry_func, role="unknown", port="??"):
        self.controller = controller
        self.entry_func = entry_func
        self.role = role
        self.port = port
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        print(f"[HANDTHREAD] start role={self.role} port={self.port} func={getattr(self.entry_func,'__name__','?')}")
        try:
            try:
                self.entry_func(self.controller, self.stop_event)
            except TypeError:
                self.entry_func(self.controller)
        except Exception as e:
            print(f"[WARN] hand thread 예외 (role={self.role}, port={self.port}): {e}")
        print(f"[HANDTHREAD] end role={self.role} port={self.port}")

    def start(self):
        if self.controller and self.entry_func:
            self.thread.start()

    def stop(self, join_timeout=2.0):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=join_timeout)


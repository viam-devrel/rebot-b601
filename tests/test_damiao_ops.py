import threading
import time

from src.rebot_b601 import damiao
from src.rebot_b601.ops import SingleOperationManager


def test_status_decoding():
    assert damiao.status_text(0x1) == "enabled"
    assert damiao.status_text(0xA) == "over-current"
    assert damiao.is_fault(0xD) and damiao.is_transient(0xD)
    assert damiao.is_fault(0xB) and not damiao.is_transient(0xB)
    assert not damiao.is_fault(0x1)
    assert "unknown" in damiao.status_text(0x5)


def test_single_operation_cancels_previous():
    ops = SingleOperationManager()
    started = threading.Event()
    cancelled = threading.Event()

    def worker():
        with ops.new() as cancel:
            started.set()
            cancel.wait(5.0)
            cancelled.set()

    t = threading.Thread(target=worker)
    t.start()
    started.wait(1.0)
    assert ops.running
    t0 = time.monotonic()
    with ops.new():
        assert cancelled.is_set()
        assert time.monotonic() - t0 < 1.0
        assert ops.running
    t.join(1.0)
    assert not ops.running

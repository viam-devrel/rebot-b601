"""Single in-flight operation manager, modelled on the RDK's
``operation.SingleOperationManager``.

At most one motion operation runs at a time. Starting a new one cancels the
current one (its cancel event is set) and waits for it to release the slot,
so ``stop()`` and back-to-back moves never interleave commands on the bus.
"""

import threading
from contextlib import contextmanager


class OperationCancelled(Exception):
    pass


class SingleOperationManager:
    def __init__(self):
        self._slot = threading.Lock()
        self._state_lock = threading.Lock()
        self._current: threading.Event | None = None

    @property
    def running(self) -> bool:
        with self._state_lock:
            return self._current is not None

    def cancel_current(self):
        with self._state_lock:
            if self._current is not None:
                self._current.set()

    @contextmanager
    def new(self):
        """Cancel any running operation, take the slot, and yield a cancel event."""
        self.cancel_current()
        self._slot.acquire()
        cancel = threading.Event()
        with self._state_lock:
            self._current = cancel
        try:
            yield cancel
        finally:
            with self._state_lock:
                if self._current is cancel:
                    self._current = None
            self._slot.release()

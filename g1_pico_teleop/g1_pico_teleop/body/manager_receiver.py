"""Receive manager traffic without making ROS callbacks drain its OFF-state flood."""

from collections import deque
import threading
import time

import zmq

from .manager_wire import MAX_MESSAGE_SIZE, TOPICS


class ManagerInbox:
    """Bounded, ordered mailbox; only consecutive identical states are merged.

    Commands, poses and mode changes are retained. Receipt times are captured
    by the receiver, so a stalled ROS executor cannot make old input fresh.
    """

    def __init__(self, capacity=128):
        self._capacity = capacity
        self._pending = deque()
        self._fault = None
        self._lock = threading.Lock()

    def put(self, message, received_at):
        with self._lock:
            if (self._pending and message.startswith(b"manager_state")
                    and self._pending[-1][0] == message):
                self._pending[-1] = (message, received_at)
                return
            if len(self._pending) >= self._capacity:
                self._fault = "manager event backlog exceeded; control data was lost"
                self._pending.clear()
            self._pending.append((message, received_at))

    def take(self):
        with self._lock:
            messages = list(self._pending)
            fault = self._fault
            self._pending.clear()
            self._fault = None
        return messages, fault


class ManagerReceiver:
    """Own the SUB socket entirely in one thread; never send robot commands."""

    def __init__(self, endpoint):
        self._endpoint = endpoint
        self._inbox = ManagerInbox()
        self._stop = threading.Event()
        self._failure = None
        self._thread = threading.Thread(target=self._run, name="pico-manager-rx", daemon=True)
        self._thread.start()

    def _run(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        try:
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVHWM, 128)
            socket.setsockopt(zmq.MAXMSGSIZE, MAX_MESSAGE_SIZE)
            # A single conflated socket would lose stop commands and mode changes.
            for topic in TOPICS:
                socket.setsockopt(zmq.SUBSCRIBE, topic)
            socket.connect(self._endpoint)
            while not self._stop.is_set():
                if not socket.poll(20):
                    continue
                for _ in range(512):
                    try:
                        message = socket.recv(zmq.DONTWAIT)
                    except zmq.Again:
                        break
                    self._inbox.put(message, time.monotonic())
                # Yield under sustained OFF-state traffic so ROS remains responsive.
                self._stop.wait(0.001)
        except Exception as exc:
            self._failure = f"ZMQ receiver failed: {exc}"
        finally:
            socket.close(linger=0)
            context.term()

    def take(self):
        messages, fault = self._inbox.take()
        return messages, self._failure or fault

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        if self._thread.is_alive():
            raise RuntimeError("manager receiver did not stop")

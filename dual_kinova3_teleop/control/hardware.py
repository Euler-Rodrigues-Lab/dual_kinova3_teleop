"""Kortex velocity control and Ability Teensy transport from the SEW hardware demos.

A worker owns the transports and consumes only the newest target. Missing input,
stale commands, exceptions and normal shutdown stop both arms. No automatic home.
"""
from contextlib import ExitStack
import json
import math
import struct
import threading
import time
import numpy as np


def joint_speeds(q, velocity, target, kp, kd, max_speed_deg):
    q, velocity, target = (np.asarray(x, dtype=float) for x in (q, velocity, target))
    if any(x.shape != (7,) or not np.isfinite(x).all() for x in (q, velocity, target)):
        raise ValueError('arm state and targets must contain seven finite radians')
    error = (target - q + np.pi) % (2 * np.pi) - np.pi
    return np.clip(np.rad2deg(kp * error - kd * velocity), -max_speed_deg, max_speed_deg)


def ability_packet(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (10,) or not np.isfinite(q).all():
        raise ValueError('Ability target must contain ten finite radians')
    # Source Teensy protocol: index, middle, ring, pinky, thumb flexion, thumb abduction.
    degrees = np.rad2deg(q[[2, 4, 6, 8, 1, 0]])
    degrees[:5] = np.clip(degrees[:5], 0, 100)
    degrees[5] = np.clip(degrees[5], -100, 0)
    return struct.pack('<6f', *degrees)


class KortexArm:
    def __init__(self, ip, username, password, serial_port=None):
        self.ip, self.username, self.password = ip, username, password
        self.serial_port = serial_port

    def __enter__(self):
        from kortex_api.TCPTransport import TCPTransport
        from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
        from kortex_api.SessionManager import SessionManager
        from kortex_api.autogen.messages import Session_pb2, Base_pb2
        from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
        from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
        self.stack = ExitStack()
        try:
            transport = TCPTransport()
            transport.connect(self.ip, 10000)
            self.stack.callback(transport.disconnect)
            router = RouterClient(transport, RouterClient.basicErrorCallback)
            manager = SessionManager(router)
            info = Session_pb2.CreateSessionInfo()
            info.username, info.password = self.username, self.password
            info.session_inactivity_timeout = 10000
            info.connection_inactivity_timeout = 2000
            manager.CreateSession(info)
            options = RouterClientSendOptions(); options.timeout_ms = 1000
            self.stack.callback(manager.CloseSession, options)
            self.messages = Base_pb2
            self.base, self.cyclic = BaseClient(router), BaseCyclicClient(router)
            self.stack.callback(self.base.Stop)
            self.serial = None
            if self.serial_port:
                import serial
                self.serial = self.stack.enter_context(serial.Serial(
                    self.serial_port, 460800, timeout=1, write_timeout=.05))
            return self
        except BaseException:
            self.stack.close()
            raise

    def feedback(self):
        f = self.cyclic.RefreshFeedback()
        q = np.deg2rad([a.position for a in f.actuators])
        q = (q + np.pi) % (2 * np.pi) - np.pi
        v = np.deg2rad([a.velocity for a in f.actuators])
        if q.shape != (7,) or not np.isfinite(q).all() or not np.isfinite(v).all():
            raise RuntimeError('invalid Kortex feedback')
        return q, v

    def command(self, speeds, gripper=None, hand=None):
        msg = self.messages.JointSpeeds()
        for i, value in enumerate(speeds):
            joint = msg.joint_speeds.add()
            joint.joint_identifier, joint.value, joint.duration = i, float(value), 0
        self.base.SendJointSpeedsCommand(msg)
        if self.serial is not None and hand is not None:
            packet = ability_packet(hand)
            if self.serial.write(packet) != len(packet):
                raise IOError('incomplete Ability serial write')
        elif self.serial is None and gripper is not None:
            # XRT +/-1 close/open convention, same threshold as simulation.
            if gripper > .5 or gripper < -.5:
                msg = self.messages.GripperCommand()
                msg.mode = self.messages.GRIPPER_POSITION
                finger = msg.gripper.finger.add()
                finger.finger_identifier, finger.value = 1, float(gripper > .5)
                self.base.SendGripperCommand(msg)

    def stop(self):
        self.base.Stop()

    def __exit__(self, *exc):
        self.stack.close()


class DryRunArm:
    """Transport-free arm surrogate using the same velocity and hand encoding."""
    def __init__(self, q):
        self.q = np.array(q, dtype=float); self.v = np.zeros(7)
        self.last = time.monotonic(); self.commands = 0; self.stops = 0
    def __enter__(self): return self
    def __exit__(self, *exc): self.stop()
    def feedback(self):
        now = time.monotonic()
        self.q += self.v * min(now - self.last, .1); self.last = now
        return self.q.copy(), self.v.copy()
    def command(self, speeds, gripper=None, hand=None):
        self.v = np.deg2rad(speeds); self.commands += 1
        if hand is not None: ability_packet(hand)
    def stop(self):
        self.v[:] = 0; self.stops += 1


class HardwareRig:
    def __init__(self, arms, *, kp, kd=.5, max_speed_deg=20., timeout=.25, trace=None, psyonic=False):
        if any(not math.isfinite(v) or v <= 0 for v in (kp, max_speed_deg, timeout)) or not math.isfinite(kd) or kd < 0:
            raise ValueError('invalid hardware control settings')
        self.arms, self.kp, self.kd = arms, kp, kd
        self.psyonic = psyonic
        self.max_speed_deg, self.timeout, self.trace = max_speed_deg, timeout, trace
        self.lock = threading.Lock(); self.done = threading.Event(); self.ready = threading.Event()
        self.target = None; self.state = None; self.error = None

    def __enter__(self):
        self.thread = threading.Thread(target=self._worker, name='kinova-control', daemon=True)
        self.thread.start()
        if not self.ready.wait(15):
            self.done.set()
            raise TimeoutError('hardware connection/feedback timed out')
        self._check()
        return self

    def _check(self):
        if self.error is not None: raise RuntimeError('hardware worker failed; stop requested') from self.error

    def feedback(self):
        self._check()
        with self.lock:
            if self.state is None or time.monotonic() - self.state[0] > self.timeout:
                raise TimeoutError('hardware feedback is stale')
            return {s: q.copy() for s, q in self.state[1].items()}

    def submit(self, out):
        values = {s: (np.array(getattr(out, f'q_goal_{s}'), copy=True),
                      getattr(out, f'{s}_gripper_val', None),
                      getattr(out, f'q_goal_{s}_hand', None) if self.psyonic else None) for s in self.arms}
        for q, gripper, hand in values.values():
            joint_speeds(np.zeros(7), np.zeros(7), q, self.kp, self.kd, self.max_speed_deg)
            if gripper is not None and not math.isfinite(gripper): raise ValueError('invalid gripper command')
            if hand is not None: ability_packet(hand)
        self._check()
        with self.lock: self.target = (time.monotonic(), values)

    def disengage(self):
        with self.lock: self.target = None

    def _worker(self):
        try:
            with ExitStack() as stack:
                arms = {s: stack.enter_context(a) for s, a in self.arms.items()}
                log = stack.enter_context(open(self.trace, 'w')) if self.trace else None
                active = False
                try:
                    while not self.done.is_set():
                        tick = time.monotonic()
                        feedback = {s: a.feedback() for s, a in arms.items()}
                        with self.lock:
                            self.state = (time.monotonic(), {s: q for s, (q, _) in feedback.items()})
                            target = self.target
                        self.ready.set()
                        if target is not None and time.monotonic() - target[0] < self.timeout:
                            for s, a in arms.items():
                                q, v = feedback[s]; goal, gripper, hand = target[1][s]
                                speeds = joint_speeds(q, v, goal, self.kp, self.kd, self.max_speed_deg)
                                a.command(speeds, gripper, hand)
                                if log: log.write(json.dumps(dict(time=tick, side=s, speed_deg=speeds.tolist()))+'\n')
                            active = True
                        elif active:
                            # Try both stops even if one transport fails.
                            with ExitStack() as stops:
                                for a in arms.values(): stops.callback(a.stop)
                            active = False
                        self.done.wait(max(0., 1/60 - (time.monotonic() - tick)))
                finally:
                    with ExitStack() as stops:
                        for a in arms.values(): stops.callback(a.stop)
        except BaseException as exc:
            self.error = exc
        finally:
            self.ready.set()

    def __exit__(self, *exc):
        self.done.set(); self.thread.join(5)
        if self.thread.is_alive(): raise TimeoutError('hardware transport did not shut down within 5 seconds')
        if not exc[0]: self._check()

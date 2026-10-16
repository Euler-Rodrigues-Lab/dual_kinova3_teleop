from types import SimpleNamespace
import struct
import time
import numpy as np
import pytest
from dual_kinova3_teleop.control.hardware import joint_speeds, ability_packet, HardwareRig, DryRunArm


def test_wrapped_pd_units_and_limit():
    q = np.zeros(7); q[0] = np.deg2rad(179)
    goal = np.zeros(7); goal[0] = np.deg2rad(-179); goal[1] = 2
    speed = joint_speeds(q, np.zeros(7), goal, 2, .5, 20)
    np.testing.assert_allclose(speed[:2], [4, 20])
    with pytest.raises(ValueError): joint_speeds(q, q, np.full(7,np.nan),2,.5,20)


def test_teensy_packet_matches_source_order_and_clipping():
    q = np.deg2rad([-45,20,30,0,40,0,50,0,120,0])
    assert struct.unpack('<6f',ability_packet(q)) == (30,40,50,100,20,-45)
    assert len(ability_packet(q)) == 24


def test_worker_stale_target_and_exit_stop_both_arms():
    arms = {s:DryRunArm(np.zeros(7)) for s in ['left','right']}
    out = SimpleNamespace(q_goal_left=np.ones(7)*.1,q_goal_right=np.ones(7)*.1)
    with HardwareRig(arms,kp=2,timeout=.08) as rig:
        rig.submit(out)
        deadline=time.monotonic()+1
        while not all(a.commands for a in arms.values()) and time.monotonic()<deadline: time.sleep(.005)
        assert all(a.commands for a in arms.values())
        time.sleep(.15)
        assert all(a.stops and not a.v.any() for a in arms.values())
        assert set(rig.feedback())=={'left','right'}
    assert all(a.stops>=2 for a in arms.values())


def test_failure_stops_other_arm():
    class Broken(DryRunArm):
        def command(self,*args): raise IOError('simulated transport failure')
    arms={'left':Broken(np.zeros(7)), 'right':DryRunArm(np.zeros(7))}
    with pytest.raises(RuntimeError,match='worker failed'):
        with HardwareRig(arms,kp=2) as rig:
            rig.submit(SimpleNamespace(q_goal_left=np.zeros(7),q_goal_right=np.zeros(7)))
            time.sleep(.08)
    assert arms['right'].stops

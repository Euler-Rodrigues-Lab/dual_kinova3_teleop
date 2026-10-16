import hashlib,json
from pathlib import Path
import numpy as np
import mujoco
import pytest
from dual_kinova3_teleop import ASSETS,SAMPLE_MOTIONS,scene_path
from dual_kinova3_teleop.control.mujoco_controller import DualKinova3MuJoCoController
from dual_kinova3_teleop.visualization import ReplayOverlay
from types import SimpleNamespace


def test_original_assets_are_unmodified():
    for name,entry in json.loads((ASSETS/'provenance.json').read_text())['files'].items():
        assert hashlib.sha256((ASSETS/name).read_bytes()).hexdigest()==entry['sha256']

@pytest.mark.parametrize('variant',['robotiq','psyonic'])
def test_joint_mapping_and_gripper_motion(variant):
    m=mujoco.MjModel.from_xml_path(str(scene_path(variant)));d=mujoco.MjData(m)
    c=DualKinova3MuJoCoController(m,d,variant);c.setup_mocap_body()
    q=np.linspace(-.2,.2,7)
    c.set_joint_goals(dict(q_goal_left=q,q_goal_right=-q,left_gripper_val=1.,right_gripper_val=1.,q_goal_left_hand=np.ones(10)*.3,q_goal_right_hand=np.ones(10)*.3))
    c.update_kinematic()
    np.testing.assert_allclose(c.q_current_left,q);np.testing.assert_allclose(c.q_current_right,-q)
    assert np.isfinite(d.qpos).all()
    if variant=='robotiq':
        assert d.qpos[m.joint('left_gripper_left_driver_joint').qposadr[0]]==.8
    else:
        assert d.qpos[m.joint('left_gripper_index_q1').qposadr[0]]==.3
    p=np.array([.1,.2,.3]);c.update_mocap_body(p,np.eye(3));c.update_kinematic()
    np.testing.assert_allclose(c.get_sew_transform()(np.zeros(3)),p)


def test_samples_match_other_robots():
    expected={'ipman_roll':('b18ad2ed75428304249626f0451eaba9865bab964289845d837969c05c994698',600),
              'picking_up_mustard':('38c32eb839d89f4b203cf6ee8923028b595c2ff76a625321f934f2033f03a04f',843)}
    for name,path in SAMPLE_MOTIONS.items():
        assert hashlib.sha256(path.read_bytes()).hexdigest()==expected[name][0]


def test_safety_overlay_uses_solver_capsules_in_base_frame():
    m=mujoco.MjModel.from_xml_path(str(scene_path()));d=mujoco.MjData(m)
    c=DualKinova3MuJoCoController(m,d);c.setup_mocap_body();c.update_mocap_body([1.,2.,3.],np.eye(3));c.update_kinematic()
    viewer=SimpleNamespace(user_scn=mujoco.MjvScene(m,maxgeom=50))
    session=SimpleNamespace(sew_capsules=lambda:[('torso',np.zeros(3),np.array([0.,0.,.2]),.06)])
    overlay=ReplayOverlay(viewer,show_human=False,show_safety=True)
    assert overlay.draw(object(),session,to_world=c.get_sew_transform())==1
    np.testing.assert_allclose(viewer.user_scn.geoms[0].pos,[1.,2.,3.1],atol=1e-6)
    assert viewer.user_scn.geoms[0].size[0]==pytest.approx(.06)
    overlay.show_safety=False
    assert overlay.draw(object(),session,to_world=c.get_sew_transform())==0


def test_cli_defaults():
    from dual_kinova3_teleop.demos.replay_offline import parse_args
    args=parse_args([])
    assert args.variant=='robotiq' and not args.no_safety_filter
    assert args.frames==str(SAMPLE_MOTIONS['ipman_roll'])


def test_video_preserves_capsule_length():
    from dual_kinova3_teleop.video import copy_geometry
    src, dst = mujoco.MjvGeom(), mujoco.MjvGeom()
    mujoco.mjv_initGeom(src, mujoco.mjtGeom.mjGEOM_CAPSULE,
                        np.array([.065,.6,0.]), np.zeros(3), np.eye(3).ravel(), np.ones(4))
    copy_geometry(dst, src)
    np.testing.assert_allclose(dst.size, [.065,.065,.6])
    np.testing.assert_allclose(dst.pos, src.pos)
    np.testing.assert_allclose(dst.mat, src.mat)

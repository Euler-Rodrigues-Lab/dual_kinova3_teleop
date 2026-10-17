"""Shared human overlay in the solver-aligned MuJoCo world frame."""
import numpy as np
from geo_kin_core.viz import HumanCapsuleViz, capsules, draw_filtered_sew


def configure_camera(camera):
    camera.lookat[:] = [0.0, 0.0, 1.0]
    camera.distance = 3.0
    camera.azimuth = 135
    camera.elevation = -18


class ReplayOverlay:
    def __init__(self, viewer, *, show_human=True, show_safety=False, tcp_points=None):
        self.tcp_points = tcp_points
        self.viewer = viewer
        self.show_human = show_human
        self.show_safety = show_safety
        self.human = HumanCapsuleViz(viewer, color=(0.15, 0.55, 1.0, 0.55), size=0.035)

    def draw(self, frame, session, to_world=None):
        capsules.clear(self.viewer)
        rotation = getattr(session, 'R_mocap_world', None)
        position = getattr(session, 'p_mocap_world', None)
        if rotation is not None and position is not None:
            self.human.set_base_offset(position, np.asarray(rotation).T)
        count = 0
        if self.show_safety and frame is not None:
            # Read the Rust session's actual XPBD capsules, including torso
            # and shoulder. No application-side proxy geometry or radii.
            count += draw_filtered_sew(self.viewer, session, to_world=to_world)
        if self.show_human:
            count += self.human.draw(frame)
        if self.tcp_points is not None and frame is not None:
            robot = self.tcp_points()
            for side, target in human_index_mcps(frame, session).items():
                if side not in robot:
                    continue
                # Nested spheres coincide when the robot reaches the human MCP.
                count += capsules.add_sphere(self.viewer, target, .018, (0., 1., 1., .45))
                count += capsules.add_sphere(self.viewer, robot[side], .011, (1., .15, .75, 1.))
                count += capsules.add_capsule(self.viewer, target, robot[side], .002, (1., .8, .1, .8))
        return count


from contextlib import contextmanager
import threading


@contextmanager
def passive_viewer(model, data):
    """Keep Python alive until MuJoCo's daemon render thread releases GLFW."""
    import mujoco.viewer
    before = set(threading.enumerate())
    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
    # launch_passive exposes a handle but no join operation. Python 3.10+
    # names the renderer thread after its target; restrict joining to the
    # renderer created by this call (not XR or hardware workers).
    render_threads = [t for t in threading.enumerate() if t not in before
                      and t.name.endswith('(_launch_internal)')]
    try:
        yield viewer
    finally:
        viewer.close()
        for thread in render_threads:
            thread.join()


def human_index_mcps(frame, session):
    """Wrist-local MCP -> capture world -> the same viewer frame as the human."""
    if frame is None:
        return {}
    rotation = getattr(session, 'R_mocap_world', None)
    position = getattr(session, 'p_mocap_world', None)
    if rotation is None or position is None:
        return {}
    body_r = getattr(frame, 'R_world_upper_body', None)
    body_p = getattr(frame, 'p_world_upper_body', None)
    body_r = np.eye(3) if body_r is None else np.asarray(body_r)
    body_p = np.zeros(3) if body_p is None else np.asarray(body_p)
    result = {}
    for side in ('left', 'right'):
        pose = getattr(frame, side+'_sew', None)
        fingers = getattr(frame, side+'_fingers', None) or {}
        mcp = fingers.get('index', {}).get('index_finger_mcp')
        if pose is None or pose.R_world_wrist is None or mcp is None:
            continue
        point = np.asarray(pose.W) + np.asarray(pose.R_world_wrist) @ np.asarray(mcp)
        point = np.asarray(rotation).T @ (body_p + body_r @ point - np.asarray(position))
        if point.shape == (3,) and np.isfinite(point).all():
            result[side] = point
    return result

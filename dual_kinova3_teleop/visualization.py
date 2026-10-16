"""Shared human overlay in the solver-aligned MuJoCo world frame."""
import numpy as np
from geo_kin_core.viz import HumanCapsuleViz, capsules, draw_filtered_sew


def configure_camera(camera):
    camera.lookat[:] = [0.0, 0.0, 1.0]
    camera.distance = 3.0
    camera.azimuth = 135
    camera.elevation = -18


class ReplayOverlay:
    def __init__(self, viewer, *, show_human=True, show_safety=False):
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

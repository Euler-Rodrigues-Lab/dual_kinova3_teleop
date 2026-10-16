"""Offscreen MuJoCo replay capture, with the same overlays as the live viewer."""
from pathlib import Path
from types import SimpleNamespace
import subprocess
import mujoco
import numpy as np
from .visualization import ReplayOverlay,configure_camera

class ReplayVideo:
    def __init__(self,model,path,fps,**overlay):
        if fps<=0:raise ValueError('video fps must be positive')
        Path(path).parent.mkdir(parents=True,exist_ok=True)
        self.renderer=mujoco.Renderer(model,height=540,width=960)
        self.camera=mujoco.MjvCamera();configure_camera(self.camera)
        self.opt=mujoco.MjvOption();self.opt.geomgroup[0]=0
        self.proxy=SimpleNamespace(user_scn=mujoco.MjvScene(model,maxgeom=2000))
        self.overlay=ReplayOverlay(self.proxy,**overlay)
        self.frames=0
        self.encoder=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','960x540','-r',str(fps),'-i','-','-an','-c:v','libx264','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
    def __enter__(self):return self
    def write(self,data,frame,session,to_world):
        self.renderer.update_scene(data,camera=self.camera,scene_option=self.opt)
        self.overlay.draw(frame,session,to_world=to_world)
        scene=self.renderer.scene
        for i in range(self.proxy.user_scn.ngeom):
            if scene.ngeom>=scene.maxgeom:break
            src=self.proxy.user_scn.geoms[i];dst=scene.geoms[scene.ngeom]
            copy_geometry(dst, src)
            scene.ngeom+=1
        self.encoder.stdin.write(self.renderer.render().tobytes());self.frames+=1
    def __exit__(self,*exc):
        self.encoder.stdin.close();code=self.encoder.wait();self.renderer.close()
        if code and not exc[0]:raise RuntimeError(f'ffmpeg exited {code}')


def copy_geometry(dst, src):
    mujoco.mjv_initGeom(dst, src.type, src.size.astype(np.float64),
                        src.pos.astype(np.float64), src.mat.astype(np.float64).ravel(), src.rgba)
    # Stored capsule size is (radius, radius, half-length), unlike initGeom's
    # (radius, half-length, unused). Do not apply that mapping a second time.
    dst.size[:] = src.size

"""MuJoCo joint mapping. Analytic retargeting lives in the licensed wheel."""
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

class DualKinova3MuJoCoController:
    def __init__(self,model,data,variant='robotiq'):
        self.model,self.data,self.variant=model,data,variant
        self.goals={}
        self.gripper_goals={'left':0.,'right':0.}
        for side in ('right','left'):
            names=[f'{side}_Actuator{i}' for i in range(1,8)]
            setattr(self,f'{side}_arm_qpos_addrs',np.array([model.joint(n).qposadr[0] for n in names]))
        self._update_current_positions()
    def setup_mocap_body(self,name='base'):
        self.mocap_id=int(self.model.body(name).mocapid[0])
        if self.mocap_id<0:raise ValueError('base must be a mocap body')
    def _update_current_positions(self):
        for side in ('right','left'):setattr(self,f'q_current_{side}',self.data.qpos[getattr(self,f'{side}_arm_qpos_addrs')].copy())
    def set_joint_goals(self,goals):
        for key,value in goals.items():
            if value is not None:self.goals[key]=np.asarray(value).copy()
    def update_mocap_body(self,position,rotation):
        self.data.mocap_pos[self.mocap_id]=position
        q=Rotation.from_matrix(rotation).as_quat()
        self.data.mocap_quat[self.mocap_id]=q[[3,0,1,2]]
    def get_sew_transform(self):
        body=self.model.body('base').id
        rotation=self.data.xmat[body].reshape(3,3).copy()
        position=self.data.xpos[body].copy()
        return lambda point: rotation @ point + position
    def get_tcp_positions(self):
        if self.variant != 'psyonic':
            return {}
        return {side: self.data.xpos[self.model.body(f'{side}_gripper_index_L1').id].copy()
                for side in ('left', 'right')}
    def update_kinematic(self):
        for side in ('right','left'):
            goal=self.goals.get(f'q_goal_{side}')
            if goal is not None:self.data.qpos[getattr(self,f'{side}_arm_qpos_addrs')]=goal
            if self.variant=='psyonic':
                hand=self.goals.get(f'q_goal_{side}_hand')
                if hand is not None:
                    for i,finger in enumerate(('thumb','index','middle','ring','pinky')):
                        for j in (1,2):
                            joint=self.model.joint(f'{side}_gripper_{finger}_q{j}')
                            self.data.qpos[joint.qposadr[0]]=np.clip(hand[2*i+j-1],*joint.range)
            else:
                command=self.goals.get(f'{side}_gripper_val')
                if command is not None:
                    if float(command)>0.5:self.gripper_goals[side]=0.8
                    elif float(command)<-0.5:self.gripper_goals[side]=0.
                angle=self.gripper_goals[side]
                # Parallel four-bar links: driver/spring turn together;
                # follower counter-rotates to keep each finger pad parallel.
                for half in ('left','right'):
                    for link,value in [('driver',angle),('spring_link',angle),('follower',-angle)]:
                        self.data.qpos[self.model.joint(f'{side}_gripper_{half}_{link}_joint').qposadr[0]]=value
        self.data.qvel[:]=0.;self.data.qacc_warmstart[:]=0.
        mujoco.mj_forward(self.model,self.data)
        self._update_current_positions()

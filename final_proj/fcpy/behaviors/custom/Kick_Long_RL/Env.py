import numpy as np
from utils.Math_Utils import Math_Utils as U
from world.World import World


class Env():
    def __init__(self, world : World) -> None:

        self.world = world

        # State space
        self.obs = np.zeros(63, np.float32)

        self.DEFAULT_ARMS = np.array([-90,-90,8,8,90,90,70,70],np.float32)
        self.kick_ori = None
        
        
    def observe(self, init=False):

        w = self.world
        r = self.world.robot

        if init: # reset variables
            self.step_counter = 0
            self.act = np.zeros(16, np.float32) # memory variable
        
        # index       observation              naive normalization
        self.obs[0] = self.step_counter /20  # simple counter: 0,1,2,3...
        self.obs[1] = r.loc_head_z      *3   # z coordinate (torso)
        self.obs[2] = r.loc_head_z_vel  /2   # z velocity (torso)  
        self.obs[3] = r.imu_torso_roll  /15   # absolute torso roll  in deg
        self.obs[4] = r.imu_torso_pitch /15   # absolute torso pitch in deg
        self.obs[5:8] = r.gyro          /100  # gyroscope
        self.obs[8:11] = r.acc          /10   # accelerometer

        self.obs[11:17] = r.frp.get('lf', np.zeros(6)) * (10,10,10,0.01,0.01,0.01) #  left foot: relative point of origin (p) and force vector (f) -> (px,py,pz,fx,fy,fz)*
        self.obs[17:23] = r.frp.get('rf', np.zeros(6)) * (10,10,10,0.01,0.01,0.01) # right foot: relative point of origin (p) and force vector (f) -> (px,py,pz,fx,fy,fz)*
        self.obs[23:39] = r.joints_position[2:18] /100    # position of all joints except arm twist, elbow, head & toes (for robot type 4)
        self.obs[39:55] = r.joints_speed[2:18]    /6.1395 # speed of    all joints except arm twist, elbow, head & toes (for robot type 4)
        # *if foot is not touching the ground, then (px=0,py=0,pz=0,fx=0,fy=0,fz=0)

        #------------------ Ball
        ball_rel_hip_center = r.inv_kinematics.torso_to_hip_transform(w.ball_rel_torso_cart_pos)

        if init:
            self.obs[55:58] = (0,0,0) # Initial velocity is 0
        elif w.ball_is_visible:
            self.obs[55:58] = (ball_rel_hip_center - self.obs[58:61]) * 10  # Ball velocity, relative to hip center
            
        self.obs[58:61] = ball_rel_hip_center # Ball position, relative to hip center (already mirrored)
        self.obs[61] = np.linalg.norm( ball_rel_hip_center ) * 2
        self.obs[62] = U.normalize_deg(self.kick_ori - r.imu_torso_orientation) / 30 # kick direction
        
        return self.obs



    def execute(self, action, allow_aerial=True):

        w = self.world
        r = self.world.robot

        if self.step_counter < 6:
            r.joints_target_speed[2:18] = action * [2,2,1,1,0.2,0.2,0.2,0.2, 2,2,1,1, 1,1,1,1]
            r.joints_target_speed[12] += 1  # lfoot roll
            r.joints_target_speed[6] += 3
            r.joints_target_speed[5] -= 1
            r.joints_target_speed[8] += 3  # lknee
            r.joints_target_speed[9] -= 5 # rknee
        else:
            r.joints_target_speed[2:18] = action * [6,6,3,3,6,6,6,6, 6,6,2,2, 1,1,1,1]
            r.joints_target_speed[6] -= 3
            r.joints_target_speed[7] += 3
            r.joints_target_speed[9] += 3 # rknee
            r.joints_target_speed[11] -= 1 # rfoot vert
            if not allow_aerial:
                r.joints_target_speed[7] = np.clip(r.joints_target_speed[7],-10,5)

        r.set_joints_target_position_direct( [0,1], np.array([0,-44],float), harmonize=False ) # commit head (fixed position)

        self.step_counter += 1

        return self.step_counter >= 16

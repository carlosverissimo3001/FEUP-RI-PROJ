import math
import numpy as np
from utils.Math_Utils import Math_Utils as U
from behaviors.custom.Step.Step_Generator import Step_Generator
from world.World import World


class Env():
    def __init__(self, world : World) -> None:

        self.world = world
        
        # State space  
        self.obs = np.zeros(64, np.float32)
        
        # Step behavior defaults
        self.STEP_DUR = 8
        self.STEP_Z_SPAN = 0.02
        self.STEP_Z_MAX = 0.70

        # IK 
        r = world.robot
        nao_specs = r.inv_kinematics.NAO_SPECS
        self.leg_length = nao_specs[1] + nao_specs[3] # upper leg height + lower leg height
        feet_y_dev = nao_specs[0] * 1.12 # wider step
        sample_time = r.STEPTIME
        max_ankle_z = nao_specs[5]

        self.step_generator = Step_Generator(feet_y_dev, sample_time, max_ankle_z)
        self.ik = r.inv_kinematics
        self.DEFAULT_ARMS = np.array([-90,-90,10,10,90,90,80,80],np.float32)

        self.walk_ori = None
        self.walk_target = None
        self.walk_final_target = None

        

    def observe(self, init=False):

        r = self.world.robot

        if init: # reset variables
            self.step_counter = 0
            self.act = np.zeros(16, np.float32) # memory variable
            self.reward = 0                     # memory variable

        # index       observation              naive normalization
        self.obs[0] = min(self.step_counter,15*8) /100  # simple counter: 0,1,2,3...
        self.obs[1] = r.loc_torso_position[2]  *3    # z coordinate (torso)
        self.obs[2] = r.loc_torso_velocity[2]  /2    # z velocity (torso)  
        self.obs[3] = 0
        self.obs[4] = r.imu_torso_roll         /15   # absolute torso roll  in deg
        self.obs[5] = r.imu_torso_pitch        /15   # absolute torso pitch in deg
        self.obs[6:9] = r.gyro                 /100  # gyroscope
        self.obs[9:12] = r.acc                 /10   # accelerometer

        self.obs[12:18] = r.frp.get('lf', np.zeros(6)) * (10,10,10,0.01,0.01,0.01) #  left foot: relative point of origin (p) and force vector (f) -> (px,py,pz,fx,fy,fz)*
        self.obs[18:24] = r.frp.get('rf', np.zeros(6)) * (10,10,10,0.01,0.01,0.01) # right foot: relative point of origin (p) and force vector (f) -> (px,py,pz,fx,fy,fz)*
        # *if foot is not touching the ground, then (px=0,py=0,pz=0,fx=0,fy=0,fz=0)

        # Joints: Forward kinematics for ankles + feet rotation + arms (front + side)
        rel_lankle = r.inv_kinematics.get_body_part_pos_relative_to_hip("lankle") # ankle position relative to center of both hip joints
        rel_rankle = r.inv_kinematics.get_body_part_pos_relative_to_hip("rankle") # ankle position relative to center of both hip joints
        lf = r.head_to_body_part_transform("torso", r.body_parts['lfoot'].transform ) # foot transform relative to torso
        rf = r.head_to_body_part_transform("torso", r.body_parts['rfoot'].transform ) # foot transform relative to torso
        lf_rot_rel_torso = np.array( [lf.get_roll_deg(), lf.get_pitch_deg(), lf.get_yaw_deg()] ) # foot rotation relative to torso
        rf_rot_rel_torso = np.array( [rf.get_roll_deg(), rf.get_pitch_deg(), rf.get_yaw_deg()] ) # foot rotation relative to torso

        # velocity
        self.obs[40:56] = r.joints_target_last_speed[2:18] # predictions == last action
        # pose
        self.obs[24:27] = rel_lankle * (8,8,5)
        self.obs[27:30] = rel_rankle * (8,8,5)
        self.obs[30:33] = lf_rot_rel_torso / 20
        self.obs[33:36] = rf_rot_rel_torso / 20
        self.obs[36:40] = r.joints_position[14:18] /100 # arms (front + side)

        '''
        Expected observations for walking state:
        Time step        R  0   1   2   3   4   5   6   7   0
        Progress         1  0 .14 .28 .43 .57 .71 .86   1   0
        Left leg active  T  F   F   F   F   F   F   F   F   T
        '''

        if init: # the walking parameters refer to the last parameters in effect (after a reset, they are pointless)
            self.obs[56] = 0.8 # step duration in time steps
            self.obs[57] = 1 # step progress
            self.obs[58] = 1 # 1 if left  leg is active
            self.obs[59] = 0 # 1 if right leg is active
        else:
            self.obs[56] = 0.8 # step duration in time steps
            self.obs[57] = self.step_generator.external_progress # step progress
            self.obs[58] = float(self.step_generator.state_is_left_active)     # 1 if left  leg is active
            self.obs[59] = float(not self.step_generator.state_is_left_active) # 1 if right leg is active

        relative_target_pos = U.rotate_2d_vec((self.walk_target[0]-r.loc_torso_position[0], self.walk_target[1]-r.loc_torso_position[1]), -r.imu_torso_orientation)
        size = np.linalg.norm(relative_target_pos) + 1e-8

        # normalize to abstract distance of target
        self.obs[60] = relative_target_pos[0]/size
        self.obs[61] = relative_target_pos[1]/size

        self.obs[62] = U.normalize_deg(self.walk_ori - r.imu_torso_orientation)/100 # diff to target orientation
        self.obs[63] = np.clip(np.linalg.norm( self.walk_final_target-r.loc_torso_position[:2] )/5, 0.2, 1)  # target distance

        return self.obs

        

    def execute_ik(self, l_pos, l_rot, r_pos, r_rot):
        r = self.world.robot
        # Apply IK to each leg + Set joint targets
          
        # Left leg 
        indices, self.values_l, error_codes = self.ik.leg(l_pos, l_rot, True, dynamic_pose=False)

        r.set_joints_target_position_direct(indices, self.values_l, harmonize=False)

        # Right leg
        indices, self.values_r, error_codes = self.ik.leg(r_pos, r_rot, False, dynamic_pose=False)

        r.set_joints_target_position_direct(indices, self.values_r, harmonize=False)

    

    def execute(self, action):
        
        r = self.world.robot

        # Actions:
        # 0,1,2    left ankle pos
        # 3,4,5    right ankle pos
        # 6,7,8    left foot rotation
        # 9,10,11  right foot rotation
        # 12,13    left/right arm front
        # 14,15    left/right arm side

        # exponential moving average
        self.act = 0.92 * self.act + 0.08 * action
        
        # execute Step behavior to extract the target positions of each leg (we will override these targets)
        lfy,lfz,rfy,rfz = self.step_generator.get_target_positions(self.step_counter == 0, self.STEP_DUR, self.STEP_Z_SPAN, self.leg_length * self.STEP_Z_MAX)

        # Leg IK
        a = self.act
        l_ankle_pos = (a[0]*0.02, a[1]*0.02 + lfy, a[2]*0.01 + lfz)
        r_ankle_pos = (a[3]*0.02, a[4]*0.02 + rfy, a[5]*0.01 + rfz)
        l_foot_rot = a[6:9]  * (3,3,5)
        r_foot_rot = a[9:12] * (3,3,5)

        # Limit leg twist
        l_foot_rot[2] = max(0,l_foot_rot[2] + 7)
        r_foot_rot[2] = min(0,r_foot_rot[2] - 7)

        # Arms actions
        arms = np.copy(self.DEFAULT_ARMS) # default arms pose
        arm_swing = math.sin(self.step_generator.state_current_ts / self.STEP_DUR * math.pi) * 6
        inv = 1 if self.step_generator.state_is_left_active else -1
        arms[0:4] += a[12:16]*4 + (-arm_swing*inv,arm_swing*inv,0,0) # arms front+side

        # Set target positions
        self.execute_ik(l_ankle_pos, l_foot_rot, r_ankle_pos, r_foot_rot)           # legs 
        r.set_joints_target_position_direct( slice(14,22), arms, harmonize=False )  # arms

        self.step_counter += 1

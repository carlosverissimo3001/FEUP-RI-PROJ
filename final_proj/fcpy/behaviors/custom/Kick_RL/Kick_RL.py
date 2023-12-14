import numpy as np
import pickle
from behaviors.custom.Kick_RL.Env import Env
from utils.Math_Utils import Math_Utils as U
from utils.Neural_Network import run_mlp
import math

class Kick_RL():

    def __init__(self, world) -> None:
        self.world = world
        self.description = "Walk to ball and kick in a given direction"
        self.auto_head = True
        self.env = Env(self.world)
        
        if world.robot.type == 1:
            model_file = "kick_all.pkl"
            self.kick_time = (8,8)
            self.kick_target_bias = (14,7) # full kick, cap power
            self.kick_cap_spd = 4
            self.kick_dist = 7
            self.kick_init_pos = 0.3
        else:
            model_file = "kick_r2.pkl"
            self.kick_time = (11,9)
            self.kick_target_bias = (7,0)
            self.kick_cap_spd = 7.03
            self.kick_dist = 11
            self.kick_init_pos = 0.26 if world.robot.type == 0 else 0.26

        with open(U.get_active_directory("/behaviors/custom/Kick_RL/"+model_file), 'rb') as f:
            self.model = pickle.load(f)

    def get_kick_pos_rot(self, direction, ball_2d, cap_power):
        ''' Returns kick position and desired robot orientation (with correction bias) '''

        direction += self.kick_target_bias[int(cap_power)] # add bias to target location
        dir_rad = direction * math.pi / 180 
        target_unit_vec = np.array([math.cos(dir_rad),math.sin(dir_rad)])
        left_unit_vec = np.array([-target_unit_vec[1], target_unit_vec[0]])

        return ball_2d - self.kick_init_pos * target_unit_vec + 0.045 * left_unit_vec, direction

    def execute(self, reset, direction, cap_power = False):

        r = self.world.robot
        b = self.world.ball_abs_pos[:2]
        me = r.loc_torso_position[:2]

        if reset:
            self.phase = 0

        kick_dir = direction+self.kick_target_bias[int(cap_power)] if self.phase == 0 else self.env.kick_dir # add bias to target location, lock after phase 1

        dir_rad = kick_dir * math.pi / 180 
        target_unit_vec = np.array([math.cos(dir_rad),math.sin(dir_rad)])
        left_unit_vec = np.array([-target_unit_vec[1], target_unit_vec[0]])
        ball_rel_hip_center = r.inv_kinematics.torso_to_hip_transform(self.world.ball_rel_torso_cart_pos)
        ball_dist = np.linalg.norm(ball_rel_hip_center)
        rot_with_bias = kick_dir + 8 # add bias of 8 deg to agent rotation (no need to normalize)


        if self.phase == 0: 
            target = b - self.kick_init_pos * target_unit_vec + 0.045 * left_unit_vec
            if np.linalg.norm(target - me) < 0.1:
                self.phase = 1
                self.env.kick_dir = kick_dir
            reset_walk = reset and r.behavior.previous_behavior != "Walk_RL" # reset walk if it wasn't the previous behavior
            r.behavior.execute_sub_behavior("Walk_RL", reset_walk, target, target, rot_with_bias)

        elif self.phase == 1:
            target = b - 0.1 * target_unit_vec + 0.045 * left_unit_vec
            if ball_dist < 0.27:
                self.phase = 2
            else:
                r.behavior.execute_sub_behavior("Walk_RL", False, target, target, rot_with_bias, 2*ball_dist-0.32) # dynamic speed

        if self.phase == 2:
            walk_rl = r.behavior.get_custom_behavior_object("Walk_RL").env
            if not walk_rl.step_generator.state_is_left_active and walk_rl.step_generator.state_current_ts == 1: # sync step
                self.phase = 3
            else:
                r.behavior.execute_sub_behavior("Walk_RL", False, (0,0), (0,0), 0, 0) # zero residuals

        if self.phase == 3:
            obs = self.env.observe(True)
            action = run_mlp(obs, self.model)
            self.env.execute(action)
            self.phase = 4
        elif self.phase > 3:
            obs = self.env.observe(False)
            action = run_mlp(obs, self.model)
            self.env.execute(action, self.kick_cap_spd if cap_power else 7.03)
            self.phase += 1
            if self.phase >= self.kick_time[int(cap_power)]: 
                return True # Max. Ep. Length: 5

        return False

    def is_ready(self):
        ''' Returns True if Walk Behavior is ready to start under current game/robot conditions '''
        return True

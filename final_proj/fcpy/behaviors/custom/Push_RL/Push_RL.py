import pickle
from behaviors.custom.Push_RL.Env_LL import Env_LL
from behaviors.custom.Push_RL.Env_HL import Env_HL
from utils.Neural_Network import run_mlp
from utils.Math_Utils import Math_Utils as U
import numpy as np


class Push_RL():

    def __init__(self, world) -> None:
        self.world = world
        self.description = "RL push"
        self.auto_head = True
        self.env_LL = Env_LL(self.world, 0.9 if world.robot.type == 3 else 1.2)
        self.env_HL = Env_HL(self.world)

        with open(U.get_active_directory([
            "/behaviors/custom/Push_RL/push_LL_R1_X9_49152000_steps.pkl",
            "/behaviors/custom/Push_RL/push_LL_R1_X9_49152000_steps.pkl",
            "/behaviors/custom/Push_RL/push_LL_R1_X9_49152000_steps.pkl",
            "/behaviors/custom/Push_RL/push_LL_R1_X9_49152000_steps.pkl",
            "/behaviors/custom/Push_RL/push_LL_R1_X9_49152000_steps.pkl"
            ][world.robot.type]), 'rb') as f:
            self.model_LL = pickle.load(f)

        with open(U.get_active_directory([
            "/behaviors/custom/Push_RL/push_HL_R1_X9_1966080_steps.pkl",
            "/behaviors/custom/Push_RL/push_HL_R1_X9_1966080_steps.pkl",
            "/behaviors/custom/Push_RL/push_HL_R1_X9_1966080_steps.pkl",
            "/behaviors/custom/Push_RL/push_HL_R1_X9_1966080_steps.pkl",
            "/behaviors/custom/Push_RL/push_HL_R1_X9_1966080_steps.pkl"
            ][world.robot.type]), 'rb') as f:
            self.model_HL = pickle.load(f)

    

    def execute(self, reset, stop=False):
        ''' Just push the ball autonomously, no target is required '''

        w = self.world
        r = self.world.robot
        bp = w.ball_abs_pos[:2]
        me = r.loc_head_position[:2]
        step_gen = r.behavior.get_custom_behavior_object("Walk_RL3").env.step_generator
        reset_push = False

        if reset:
            self.phase = 0
            if r.behavior.previous_behavior == "Dribble_RL" and 0<b_rel[0]<0.25 and abs(b_rel[1])<0.07:
                self.phase = 1
                reset_push = True

        if self.phase == 0: 

            goal_target = (15.1, np.clip( bp[1], -0.7, 0.7))
            goal_ori = U.vector_angle( goal_target - bp )

            vec_me_ball_ori =  U.vector_angle(bp - me)
            rel_curr_angle = U.normalize_deg( vec_me_ball_ori - goal_ori ) # current relative angle
            abs_targ_angle = goal_ori + np.clip(rel_curr_angle, -60, 60) # desired absolute angle

            # out of bounds protection
            if bp[1] > 9:
                abs_targ_angle = np.clip(abs_targ_angle, -160, -20) 
            elif bp[1] < -9:
                abs_targ_angle = np.clip(abs_targ_angle, 20, 160) 
            if bp[0] > 14:
                if bp[1] > 1.1:
                    abs_targ_angle = np.clip(abs_targ_angle, -140, -100) 
                elif bp[1] < -1.1:
                    abs_targ_angle = np.clip(abs_targ_angle, 100, 140) 
                else: # near goal
                    abs_targ_angle = goal_ori 


            ball_dist = np.linalg.norm(bp - me)
            
            ori = None if ball_dist > 0.8 else abs_targ_angle
            next_pos, next_ori, dist_to_final_target = r.path_manager.get_path_to_ball(
                x_ori=abs_targ_angle, x_dev=-0.19, torso_ori=ori)

            b_rel = w.ball_rel_torso_cart_pos

            ang_diff = abs(U.normalize_deg( abs_targ_angle - r.imu_torso_orientation ))

            if (0.05 < b_rel[0] < 0.25 and abs(b_rel[1]) < 0.05 and not step_gen.state_is_left_active and 
                step_gen.switch and w.time_local_ms - w.ball_abs_pos_last_update < 300 and ang_diff < 10):
                self.phase += 1
                reset_push = True
                self.counter = 0
            else:
                dist = max(0.13, dist_to_final_target)
                reset_walk = reset and r.behavior.previous_behavior != "Walk_RL3" # reset walk if it wasn't the previous behavior
                r.behavior.execute_sub_behavior("Walk_RL3", reset_walk, next_pos, True, next_ori if dist_to_final_target < 1 else None, True, dist) # target, is_target_abs, ori, is_ori_abs, distance

            if stop:
                return True

        if self.phase == 1:

            # -------------------------------------------------- check if terminal 

            # out of bounds protection
            leaving_field = ((bp[1] > 9  and r.imu_torso_orientation > 0) or
                             (bp[1] < -9 and r.imu_torso_orientation < 0) or
                             (bp[0] > 14 and abs(bp[1]) > 1.1 and abs(r.imu_torso_orientation) < 90))
      
            ball_hip = r.inv_kinematics.torso_to_hip_transform(w.ball_rel_torso_cart_pos)[:2]
            dist_ball_our_goal = np.linalg.norm(bp - (-15,0))
            dist_us_our_goal = np.linalg.norm(me - (-15,0))
            lost = (dist_ball_our_goal + 0.2 < dist_us_our_goal) or (abs(ball_hip[1]) > 0.20)
            ball_unseen = w.time_local_ms - w.ball_last_seen >= 400
            ball_far = np.linalg.norm( ball_hip ) > 0.30

            terminal = ball_unseen or ball_far or lost or leaving_field # terminal

            # -------------------------------------------------- 

            if stop or terminal:
                self.phase += 1
            else:
                # update HL direction (2 Hz)
                if self.counter % 25 == 0:
                    obs = self.env_HL.observe(reset_push)
                    action = run_mlp(obs, self.model_HL)  
                    self.env_LL.HL_abs_direction = self.env_HL.execute(action)   
                self.counter += 1

                # run LL (50 Hz)
                self.env_LL.push_speed = 1
                obs = self.env_LL.observe(reset_push)
                action = run_mlp(obs, self.model_LL)   
                self.env_LL.execute(action) 

                d = w.draw
                if d.enabled:
                    vec = U.vector_from_angle(self.env_LL.HL_abs_direction)
                    d.line(me,  me+vec, 4, d.Color.red, "opp_vec")

                return False

        # wind down push, and then reset phase
        if self.phase > 1:
            WIND_DOWN_STEPS = 50
            #------------------------ 1. Define dribble wind down parameters 
            self.env_LL.push_speed = 1 - self.phase/WIND_DOWN_STEPS 
            self.env_LL.HL_abs_direction = r.imu_torso_orientation

            #------------------------ 2. Execute behavior
            obs = self.env_LL.observe(reset_push)
            action = run_mlp(obs, self.model_LL)   
            self.env_LL.execute(action) 

            #------------------------ 3. Reset behavior
            self.phase += 1
            if self.phase >= WIND_DOWN_STEPS - 5 or np.linalg.norm(r.get_head_abs_vel(4)) < 0.15:
                self.phase = 0
                return True


            
        return False


    def is_ready(self):
        ''' Returns True if Push Behavior is ready to start under current game/robot conditions '''
        return True

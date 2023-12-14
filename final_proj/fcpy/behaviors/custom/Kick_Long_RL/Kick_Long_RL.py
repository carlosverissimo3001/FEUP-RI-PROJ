import pickle
from behaviors.custom.Kick_Long_RL.Env import Env
from utils.Math_Utils import Math_Utils as U
from utils.Neural_Network import run_mlp

class Kick_Long_RL():

    def __init__(self, world) -> None:
        self.world = world
        self.description = "Walk to ball and perform a long kick"
        self.auto_head = False
        self.env = Env(self.world)
        
        with open(U.get_active_directory([
            "/behaviors/custom/Kick_Long_RL/long_kick_R0_00_26419200.pkl",
            "/behaviors/custom/Kick_Long_RL/long_kick_R1_00_8.4M.pkl",
            "/behaviors/custom/Kick_Long_RL/long_kick_R2_28262400.pkl",
            "/behaviors/custom/Kick_Long_RL/long_kick_R3_20275200.pkl",
            "/behaviors/custom/Kick_Long_RL/long_kick_R0_00_26419200.pkl"
            ][world.robot.type]), 'rb') as f:
            self.model = pickle.load(f)

    def execute(self, reset, orientation, allow_aerial=True):
        '''
        Parameters
        ----------
        orientation : float
            absolute orientation of torso (relative to imu_torso_orientation), in degrees
        allow_aerial : float
            allow aerial kicks, not recommeded near goal
        '''

        w = self.world
        r = self.world.robot
        step_gen = r.behavior.get_custom_behavior_object("Walk_RL3").env.step_generator
        reset_kick = False

        if reset:
            self.phase = 0
            self.reset_time = w.time_local_ms

        if self.phase == 0: 
            next_pos, next_ori, dist_to_final_target = r.path_manager.get_path_to_ball(
                x_ori=orientation, x_dev=-0.21, y_dev=0.045, torso_ori=orientation)

            ang_diff = abs(U.normalize_deg( orientation - r.loc_torso_orientation )) # the reset was learned with loc, not IMU

            if (dist_to_final_target < 0.018 and ang_diff < 6 and not step_gen.state_is_left_active and 
                step_gen.state_current_ts == 2 and w.time_local_ms - w.ball_abs_pos_last_update < 100 and
                w.time_local_ms - self.reset_time > 500): # do not kick immediately without preparation
                self.phase += 1
                reset_kick = True
            else:
                dist = max(0.07, dist_to_final_target*0.7)
                reset_walk = reset and r.behavior.previous_behavior != "Walk_RL3" # reset walk if it wasn't the previous behavior
                r.behavior.execute_sub_behavior("Walk_RL3", reset_walk, next_pos, True, next_ori, True, dist) # target, is_target_abs, ori, is_ori_abs, distance

        if self.phase == 1: # define kick parameters and execute
            self.env.kick_ori = orientation      

            obs = self.env.observe(reset_kick)
            action = run_mlp(obs, self.model)   
            return self.env.execute(action, allow_aerial) 

        return False

    def is_ready(self):
        ''' Returns True if Kick Behavior is ready to start under current game/robot conditions '''
        return True

import pickle
from behaviors.custom.Walk_RL3.Env import Env
from utils.Neural_Network import run_mlp
from utils.Math_Utils import Math_Utils as U
import numpy as np

class Walk_RL3():

    def __init__(self, world) -> None:
        self.world = world
        self.description = "Omnidirectional RL walk"
        self.auto_head = True
        self.env = Env(self.world)
        self.last_executed = 0

        with open(U.get_active_directory([
            "/behaviors/custom/Walk_RL3/walk_R0_36_91M.pkl",
            "/behaviors/custom/Walk_RL3/walk_R3_55M.pkl",
            "/behaviors/custom/Walk_RL3/walk_R2_3_169M.pkl",
            "/behaviors/custom/Walk_RL3/walk_R3_55M.pkl",
            "/behaviors/custom/Walk_RL3/walk_R4_196M.pkl"
            ][world.robot.type]), 'rb') as f:
            self.model = pickle.load(f)

    def execute(self, reset, target_2d, is_target_absolute, orientation, is_orientation_absolute, distance):
        '''
        Parameters
        ----------
        target_2d : array_like
            2D target in absolute or relative coordinates (use is_target_absolute to specify)
        is_target_absolute : bool
            True if target_2d is in absolute coordinates, False if relative to robot's head
        orientation : float
            absolute or relative orientation of torso, in degrees
            set to None to go towards the target (is_orientation_absolute is ignored)
        is_orientation_absolute : bool
            True if orientation is relative to the field, False if relative to the robot's torso
        distance : float
            distance to final target [0,0.5] (influences walk speed when approaching the final target)
            set to None to consider target_2d the final target
        '''
        r = self.world.robot

        #------------------------ 0. Override reset (since some behaviors use this as a sub-behavior)
        if reset and self.world.time_local_ms - self.last_executed == 20:
            reset = False
        self.last_executed = self.world.time_local_ms

        #------------------------ 1. Define walk parameters 

        if is_target_absolute: # convert to target relative to (head position + torso orientation)
            raw_target = target_2d - r.loc_head_position[:2]
            self.env.walk_rel_target = U.rotate_2d_vec(raw_target, -r.imu_torso_orientation)
        else:
            self.env.walk_rel_target = target_2d

        if distance is None:
            self.env.walk_distance = np.linalg.norm(self.env.walk_rel_target)
        else:
            self.env.walk_distance = distance # MAX_LINEAR_DIST = 0.5

        # Relative orientation values are decreased to avoid overshoot
        if orientation is None:
            self.env.walk_rel_orientation = U.vector_angle(self.env.walk_rel_target) * 0.3
        elif is_orientation_absolute:
            self.env.walk_rel_orientation = U.normalize_deg( orientation - r.imu_torso_orientation )
        else:
            self.env.walk_rel_orientation = orientation * 0.3

        #------------------------ 2. Execute behavior

        obs = self.env.observe(reset)
        action = run_mlp(obs, self.model)   
        self.env.execute(action)
        
        return False

    def is_ready(self):
        ''' Returns True if Walk Behavior is ready to start under current game/robot conditions '''
        return True

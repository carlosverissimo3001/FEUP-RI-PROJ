import numpy as np
import pickle
from behaviors.custom.Walk_RL.Env import Env
from utils.Math_Utils import Math_Utils as U
from utils.Neural_Network import run_mlp

class Walk_RL():

    def __init__(self, world) -> None:
        self.world = world
        self.description = "Omnidirectional RL walk"
        self.auto_head = True
        self.env = Env(self.world)

        with open(U.get_active_directory("/behaviors/custom/Walk_RL/walk.pkl"), 'rb') as f:
            self.model = pickle.load(f)

    def execute(self, reset, target_2d, final_target_2d, orientation, speed=None):
        '''
        Parameters
        ----------
        speed : float
            action multiplier from 0 to 1 (max. speed)
            default is `None`, which regulates the speed according to the distance to the final target
        '''

        self.env.walk_target = target_2d
        self.env.walk_final_target = final_target_2d
        self.env.walk_ori = orientation

        obs = self.env.observe(reset)
        action = run_mlp(obs, self.model)

        if speed is None:
            distance = np.linalg.norm(self.world.robot.loc_torso_position[:2] - final_target_2d)
            speed = min(distance + 0.5, 1)
            
        self.env.execute(action * speed)
        
        return False

    def is_ready(self):
        ''' Returns True if Walk Behavior is ready to start under current game/robot conditions '''
        return True

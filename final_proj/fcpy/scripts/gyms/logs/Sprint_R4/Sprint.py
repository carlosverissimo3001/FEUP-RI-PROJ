from agent.commons.Base_Agent import Base_Agent as Agent
from behaviors.custom.Step.Step import Step
from utils.Draw import Draw
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from scripts.commons.Server import Server
from scripts.commons.Train_Base import Train_Base
import os, gym
import numpy as np

'''
Objective:
----------
Learn how to sprint as fast as possible without falling down.
- class Sprint:  implements a custom gym environment
- class Train:  implements algorithms to train a new model or test an existing model
'''

class Sprint(gym.Env):
    def __init__(self, ip, server_p, monitor_p, r_type, enable_draw) -> None:

        self.robot_type = r_type
        self.player = Agent(ip, server_p, monitor_p, 1, self.robot_type, "Gym", True, enable_draw, [])
        self.step_counter = 0 # to limit episode size

        self.step_obj : Step = self.player.world.robot.behavior.get_custom_behavior_object("Step") # Step behavior object

        # State space
        obs_size = 70
        self.obs = np.zeros(obs_size, dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=np.full(obs_size,-np.inf,np.float32), high=np.full(obs_size,np.inf,np.float32), dtype=np.float32)

        # Action space
        MAX = np.finfo(np.float32).max
        self.no_of_actions = act_size = 22
        self.action_space = gym.spaces.Box(low=np.full(act_size,-MAX,np.float32), high=np.full(act_size,MAX,np.float32), dtype=np.float32)

        # Step behavior defaults
        self.step_default_dur = 7
        self.step_default_z_span = 0.035
        self.step_default_z_max = 0.70

        # Place ball far away to keep landmarks in FoV (head follows ball while using Step behavior)
        self.player.scom.unofficial_move_ball((14, 0, 0.042))


    def observe(self, init=False):
        # TODO: IMPLEMENT
        return

    def step(self):
        # TODO: IMPLEMENT
        return

    def reset(self):
        '''
        Reset and stabilize the robot
        Note: for some behaviors it would be better to reduce stabilization or add noise
        '''
        # TODO: IMPLEMENT
        return


    def sync(self):
        '''Run a single simulation step'''
        r = self.player.world.robot
        self.player.scom.commit_and_send(r.get_command())
        self.player.scom.receive()

    def render(self, mode="human", close=False):
        return

    def close(self):
        Draw.clear_all()
        self.player.scom.close()



class Train(Train_Base):
    def __init__(self, script) -> None:
        super().__init__(script)

    def train(self, args):

        #--------------------------------------- Learning parameters
        n_envs = min(32, os.cpu_count())
        n_steps_per_env = 512  # RolloutBuffer is of size (n_steps_per_env * n_envs)
        minibatch_size = 64    # should be a factor of (n_steps_per_env * n_envs)
        total_steps = 30000000
        learning_rate = 3e-4
        folder_name = f'Sprint_R{self.robot_type}'
        model_path = f'./scripts/gyms/logs/{folder_name}/'

        print("Model path:", model_path)

         #--------------------------------------- Run algorithm
        def init_env(i_env):
            def thunk():
                return Sprint( self.ip , self.server_p + i_env, self.monitor_p_1000 + i_env, self.robot_type, False )
            return thunk

        servers = Server( self.server_p, self.monitor_p_1000, n_envs+1 ) #include 1 extra server for testing

        env = SubprocVecEnv( [init_env(i) for i in range(n_envs)] )
        eval_env = SubprocVecEnv( [init_env(n_envs)] )

        if "model_file" in args:
            model = PPO.load( args["model_file"]    , env=env, device="cpu", n_envs=n_envs, n_steps=n_steps_per_env, batch_size=minibatch_size, learning_rate=learning_rate )
        else:
            model = PPO( "MlpPolicy", env=env, verbose=1, n_steps=n_steps_per_env, batch_size=minibatch_size, learning_rate=learning_rate, device="cpu" )

        model_path = self.learn_model( model, total_steps, model_path, eval_env=eval_env, eval_freq=n_steps_per_env*20, save_freq=n_steps_per_env*200, backup_env_file=__file__ )

        env.close()
        eval_env.close()
        servers.kill()

    def test(self, args):

        # Uses different server and monitor ports
        server = Server( self.server_p-1, self.monitor_p, 1 )
        env = Sprint( self.ip, self.server_p-1, self.monitor_p, self.robot_type, True )
        model = PPO.load( args["model_file"], env=env )

        self.test_model( model, env, log_path=args["folder_dir"], model_path=args["folder_dir"] )

        env.close()
        server.kill()



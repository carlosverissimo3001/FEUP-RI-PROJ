from agent.commons.Base_Agent import Base_Agent as Agent
from utils.Draw import Draw
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from scripts.commons.Server import Server
from scripts.commons.Train_Base import Train_Base
import os, gym
import numpy as np
from random import uniform

'''
Objective:
----------
Learn how to follow ball (using walking behavior)
- class Follow_Ball: implements an OpenAI custom gym
- class Train:  implements algorithms to train a new model or test an existing model

To move the ball, the game rules must be deactivated:
/usr/local/share/rcssserver3d/naosoccersim.rb
    #gameControlServer.initControlAspect('SoccerRuleAspect')
'''

class Follow_Ball(gym.Env):
    def __init__(self, ip, server_p, monitor_p, r_type, enable_draw) -> None:

        self.robot_type = r_type
        self.player = Agent(ip, server_p, monitor_p, 1, self.robot_type, "Gym", True, enable_draw, [])
        self.step_counter = 0 # to limit episode size

        # State space
        self.obs = np.zeros(2)
        self.observation_space = gym.spaces.Box(low=np.full(len(self.obs),-np.inf,np.float32), high=np.full(len(self.obs),np.inf,np.float32), dtype=np.float32)

        # Action space
        MAX = np.finfo(np.float32).max
        no_of_actions = 3
        self.action_space = gym.spaces.Box(low=np.full(no_of_actions,-MAX,np.float32), high=np.full(no_of_actions,MAX,np.float32), dtype=np.float32)
        

    def observe(self):

        self.obs[:] = self.player.world.ball_rel_torso_cart_pos[0:2]

        return self.obs


    def sync(self):
        ''' Run a single simulation step '''
        r = self.player.world.robot
        self.player.scom.commit_and_send( r.get_command() )
        self.player.scom.receive()


    def reset(self):
        '''
        Reset and stabilize the robot
        Note: for some behaviors it would be better to reduce stabilization or add noise
        '''

        self.step_counter = 0
        r = self.player.world.robot
        s = self.player.scom

        agent_x = -1
        agent_y = 0
        agent_ori = uniform(-180,180)
        
        for _ in range(25): 
            s.unofficial_beam((agent_x, agent_y, 0.50), agent_ori) # beam player continuously (floating above ground)
            r.behavior.execute("Zero_Bent_Knees")
            self.sync()

        # beam player to ground
        s.unofficial_beam((agent_x, agent_y, r.beam_height), agent_ori) 
        r.joints_target_speed[0] = 0.01 # move head to trigger physics update (rcssserver3d bug when no joint is moving)
        self.sync()

        # stabilize on ground
        for _ in range(7): 
            r.behavior.execute("Zero_Bent_Knees")
            self.sync()

        # place ball randomly
        ball_x = uniform(0,5)
        ball_y = uniform(-5,5)
        s.unofficial_move_ball((ball_x, ball_y, 0.042))
        self.last_ball_dist = np.linalg.norm((ball_x-agent_x, ball_y-agent_y))

        return self.observe()


    def step(self, action):

        r = self.player.world.robot

        # Control omni-walk parameters: Step X, Step Y, Step rotation angle (all relative to robot)
        # Note: removing or restricting some of these parameters may be beneficial for the final behavior
        step_x = action[0]     /10
        step_y = action[1]     /20
        step_theta = action[2] *20
       
        # Reduce control frequency from 50Hz to 5Hz (to reduce noise)
        for _ in range(10):
            r.behavior.execute("Walk", step_x, step_y, step_theta)
            self.sync() # run multiple simulation steps
        
        self.step_counter += 1
        
        # Compute reward
        ball_pos_2d = self.player.world.ball_cheat_abs_pos[0:2]
        robot_pos_2d = r.cheat_abs_pos[0:2]
        new_ball_dist = np.linalg.norm(ball_pos_2d - robot_pos_2d)
        reward = self.last_ball_dist - new_ball_dist
        self.last_ball_dist = new_ball_dist

        # Reward for catching ball
        if new_ball_dist < 0.5:
            reward = 2

        # Terminal state
        terminal = (
            r.cheat_abs_pos[2] < 0.3 or  # the robot is falling
            self.step_counter > 400 or   # timeout (8s)
            new_ball_dist < 0.5          # ball was caught
        )

        return self.observe(), reward, terminal, {}


    def render(self, mode='human', close=False):
        return

    def close(self):
        Draw.clear_all()
        self.player.scom.close()





class Train(Train_Base):
    def __init__(self, script) -> None:
        super().__init__(script)


    def train(self, args):

        #--------------------------------------- Learning parameters
        n_envs = min(4, os.cpu_count())
        n_steps_per_env = 64    # RolloutBuffer is of size (n_steps_per_env * n_envs) (*RV: >=2048)
        minibatch_size = 64     # should be a factor of (n_steps_per_env * n_envs)
        total_steps = 50000     # (*RV: >=10M)
        learning_rate = 3e-4
        # *RV -> Recommended value for more complex environments
        folder_name = f'Follow_Ball_R{self.robot_type}'
        model_path = f'./scripts/gyms/logs/{folder_name}/'

        print("Model path:", model_path)

        #--------------------------------------- Run algorithm
        def init_env(i_env):
            def thunk():
                return Follow_Ball( self.ip , self.server_p + i_env, self.monitor_p_1000 + i_env, self.robot_type, False )
            return thunk

        servers = Server( self.server_p, self.monitor_p_1000, n_envs+1 ) #include 1 extra server for testing

        env = SubprocVecEnv( [init_env(i) for i in range(n_envs)] )
        eval_env = SubprocVecEnv( [init_env(n_envs)] )

        if "model_file" in args:
            model = PPO.load( args["model_file"], env=env, n_envs=n_envs, n_steps=n_steps_per_env, batch_size=minibatch_size, learning_rate=learning_rate )
        else:
            model = PPO( "MlpPolicy", env=env, verbose=1, n_steps=n_steps_per_env, batch_size=minibatch_size, learning_rate=learning_rate )

        model_path = self.learn_model( model, total_steps, model_path, eval_env=eval_env, eval_freq=n_steps_per_env*100, backup_env_file=__file__ )
        
        env.close()
        eval_env.close()
        servers.kill()
        

    def test(self, args):

        # Uses different server and monitor ports
        server = Server( self.server_p-1, self.monitor_p, 1 )
        env = Follow_Ball( self.ip, self.server_p-1, self.monitor_p, self.robot_type, True )
        model = PPO.load( args["model_file"], env=env )

        self.test_model( model, env, log_path=args["folder_dir"], model_path=args["folder_dir"] )

        env.close()
        server.kill()


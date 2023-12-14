from random import uniform
import random
from scripts.commons.UI import UI
import numpy as np
from utils.Draw import Draw
from agent.commons.Base_Agent import Base_Agent as Agent
from scripts.commons.Script import Script
from os.path import isfile, join
from os import listdir
import importlib
from agent.commons.Role_Manager import Role_Manager
from world.World import World
from utils.Math_Utils import Math_Utils as U

'''
Formation Test
--------------
There are 3 types of roles:
- Active player(s), which only depends on the ball position and velocity (computed by the role_manager C++ module)
- Independent roles, including the GK (defined by the formation python module)
- Man-marking roles (computed by the role_manager C++ module)
'''

class Formation():

    def __init__(self,script:Script) -> None:
        self.script = script
        self.player : Agent = None

    def ask_for_formation_class(self):
        formations_path = Script.ROOT_DIR + "/formations/"
        exclusions = ["__init__.py"]
        formations = sorted([f[:-3] for f in listdir(formations_path) if isfile(join(formations_path, f)) and f.endswith(".py") and f not in exclusions])

        UI.print_table( [formations], ["Formations"], numbering=[True])
        choice, _ = UI.read_particle('Choose formation: ',[],int,[0,len(formations)])

        cls = importlib.import_module(f'formations.{formations[choice]}').Formation
        return cls

    def sync(self):
        self.script.batch_commit_and_send() 
        self.script.batch_receive()

    def execute(self):

        formation_cls = self.ask_for_formation_class()
        
        a = self.script.args    

        # Server IP, Agent Port, Monitor Port, Uniform Number, Team name, Enable Log, Enable Draw
        team_args = ((a.i, a.p, a.m, u, u%5, "Opponent", False, False) for u in range(1,8))
        self.script.batch_create(Agent,((a.i, a.p, a.m, 1, 0, a.t, True, True),))
        self.script.batch_create(Agent,team_args)

        # Beam to default position
        self.script.batch_unofficial_beam(((-20,0,0.5,0),
                                        (3,0,0.5,0),(1,-3,0.5,0),(1,3,0.5,0),(-3,-5,0.5,0),(-4,0,0.5,0),(-3,5,0.5,0),(-6,0,0.5,0)))

        for _ in range(3): 
            self.sync()

        w : World = self.script.players[0].world
        r = w.robot
        d = w.draw
        c = d.Color
        rm = Role_Manager(w, 0.5, 0.5, formation_cls(w))
        virtual_players_positions = np.random.uniform(-6,6,(11,2)) # random initial positions
        role_info = [None]*11  # (idx, position, priority unums)
        fallen_countdown = [0]*11

        while True:    
            #------------------- 1. Update common variables used by formations (to represent each virtual player)
            for i in range(11):
                w.teammates[i].state_abs_pos = virtual_players_positions[i]
                w.teammates[i].state_last_update = w.time_local_ms
                fallen_countdown[i] += -1 if fallen_countdown[i]>0 else 100 if random.random()<0.0005 else 0 # randomly fall for 2s
                w.teammates[i].state_fallen = fallen_countdown[i]>0

            w.ball_abs_pos = np.copy(w.ball_cheat_abs_pos)

            for i in range(11):
                #------------------- 1. Update common variables used by formations (to represent each virtual player)
                r.unum = i+1
                r.loc_head_position = U.to_3d(virtual_players_positions[i])
                r.loc_head_z = 0.5 if fallen_countdown[i]==0 else 0
                
                #------------------- 2. Compute target role
                role_info[i] = rm.get_role(w.ball_cheat_abs_pos[:2])

            for i in range(11):    
                #------------------- 3. Move closer to role position (if not fallen)
                target_vec = role_info[i][1] - virtual_players_positions[i]
                vec_len = np.linalg.norm(target_vec)
                if not w.teammates[i].state_fallen:
                    if role_info[i][0]>=10: # if going to ball, stop at a small distance
                        if vec_len > 0.2:
                            target_vec *= uniform(0.09,0.11)/vec_len # advance with some randomness to avoid artificial locks
                        else:
                            target_vec = (0,0)
                    else:
                        target_vec *= uniform(0.09,0.11)/vec_len if vec_len > 0.1 else 1 # advance with some randomness to avoid artificial locks
                    virtual_players_positions[i] = target_vec + virtual_players_positions[i]

                #------------------- 4. Draw virtual players
                d.annotation(virtual_players_positions[i], i+1, c.white, "pos", False)
                color = {0:c.white, 1:c.green_light, 2:c.get(0, 138, 230), 3:c.get(26, 163, 255), 4:c.get(77, 184, 255), 5:c.get(128, 204, 255),
                         6:c.yellow, 7:c.yellow, 8:c.purple_magenta, 9:c.purple_magenta, 10:c.red, 11:c.orange, 12:c.orange_ligth}[role_info[i][0]]
                d.circle(virtual_players_positions[i], 0.25, 4, color, "pos", False) 
                if w.teammates[i].state_fallen:
                    d.circle(virtual_players_positions[i], 0.35, 5, c.white, "pos", False)
                    d.annotation(virtual_players_positions[i]+(0,0.8), "Fallen", c.yellow, "pos", False)
                if vec_len > 0.1:
                    d.arrow(virtual_players_positions[i], role_info[i][1], 0.1, 2, c.yellow if w.teammates[i].state_fallen else c.white, "pos", False)

                # draw connections to higher priority teammates
                for j in role_info[i][2]:
                    d.arrow(virtual_players_positions[i], virtual_players_positions[i]*0.5+virtual_players_positions[j-1]*0.5, 0.1, 1, c.gray_50, "pos", False)

                # draw best kick positions
                if role_info[i][3][2] > 0: 
                    d.circle(role_info[i][3], 0.10, 3, c.green_lawn, "pos", False)
                    d.line(role_info[i][3][:2], virtual_players_positions[i], 2, c.green_lawn, "pos", False)
                    d.annotation((*role_info[i][3][:2],0.5), f"{role_info[i][3][2]:.2f}", c.green_lawn, "pos", False)
                if role_info[i][4][2] > 0:
                    d.circle(role_info[i][4], 0.10, 3, c.blue_light, "pos", False)
                    d.line(role_info[i][4][:2], virtual_players_positions[i], 2, c.blue_light, "pos", False)
                    d.annotation((*role_info[i][4][:2],0.5), f"{role_info[i][4][2]:.2f}", c.green_lawn, "pos", False)

            d.flush("pos")
            self.sync()



from agent.commons.Base_Agent import Base_Agent as Agent
from itertools import count
from utils.Draw import Draw
from scripts.commons.Script import Script
from utils.Math_Utils import Math_Utils as U
import numpy as np

'''
Objective:
----------
Dribble
'''

class Dribble():
    def __init__(self, script:Script) -> None:
        self.script = script

    def execute(self):

        a = self.script.args        
        
        player = Agent(a.i, a.p, a.m, a.u, a.r, "Dribbler", True, True, [])
        w = player.world
        r = w.robot

        player.scom.commit_beam((-3,0),0)


        for i in count():
            r.behavior.execute("Dribble_RL",None,None)
            w.draw.annotation(r.loc_head_position+(0,0,0.2),f"{np.linalg.norm(r.get_head_abs_vel(40)[:2]):.2f}",w.draw.Color.white,"vel_annotation")
                        
            player.scom.commit_and_send( r.get_command() ) 
            player.scom.receive()

            if r.behavior.is_ready("Get_Up"):
                player.scom.unofficial_beam((-3,0,0.45),0)
                r.behavior.execute_to_completion("Zero_Bent_Knees")

       
        #Clear drawings and close sockets
        Draw.clear_all()
        player.scom.close()
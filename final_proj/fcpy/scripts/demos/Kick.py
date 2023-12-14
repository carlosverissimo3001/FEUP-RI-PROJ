
from agent.commons.Base_Agent import Base_Agent as Agent
from itertools import count
from utils.Draw import Draw
from scripts.commons.Script import Script
from utils.Math_Utils import Math_Utils as U
import numpy as np

'''
Objective:
----------
Demonstrate short and long kicks
'''

class Kick():
    def __init__(self, script:Script) -> None:
        self.robot_type = 0
        self.script = script

    def execute(self):

        a = self.script.args        
        
        player = Agent(a.i, a.p, a.m, a.u, self.robot_type, a.t, True, True, [])
        w = player.world
        r = w.robot

        player.scom.commit_beam((-3,0),0)


        for i in count():
            b = w.ball_cheat_abs_pos[:2]
            vec = (0,0)-b
            vec_len = np.linalg.norm(vec) + 1e-8

            if vec_len > 9: # long kick
                if np.linalg.norm(w.ball_cheat_abs_vel) < 0.05:
                    w.draw.point(b+vec/vec_len*19, 8, w.draw.Color.pink, "target")
                r.behavior.execute("Kick_Long_RL", U.vector_angle(vec))
            else: # short kick
                if np.linalg.norm(w.ball_cheat_abs_vel) < 0.05:
                    w.draw.point(b+vec/vec_len*np.clip(vec_len,3,9), 8, w.draw.Color.pink, "target")
                r.behavior.execute("Kick_Short_RL", U.vector_angle(vec), vec_len)
                        
            player.scom.commit_and_send( r.get_command() ) 
            player.scom.receive()

            if r.behavior.is_ready("Get_Up"):
                player.scom.unofficial_beam(r.cheat_abs_pos,0)
                r.behavior.execute_to_completion("Zero_Bent_Knees")

       
        #Clear drawings and close sockets
        Draw.clear_all()
        player.scom.close()

from agent.commons.Base_Agent import Base_Agent as Agent
from itertools import count
from utils.Draw import Draw
from scripts.commons.Script import Script
from utils.Math_Utils import Math_Utils as U

'''
Objective:
----------
Go towards ball, but rotate towards field center
'''

class Walking():
    def __init__(self, script:Script) -> None:
        self.robot_type = 0
        self.script = script

    def execute(self):

        a = self.script.args        
        
        player = Agent(a.i, a.p, a.m, a.u, self.robot_type, a.t, True, True, [])
        player.scom.commit_beam((-3,0),0)
        behav = player.world.robot.behavior

        for i in count():

            ball = player.world.ball_rel_torso_cart_pos[0:2]
            pos  = player.world.robot.loc_head_position
            ori  = player.world.robot.imu_torso_orientation
            rot  = U.target_rel_angle(pos, ori, (0,0))
            
            if player.world.robot.loc_head_z < 0.2:
                p = player.world.robot.loc_head_position
                player.scom.unofficial_beam((p[0],p[1],0.46), rot)
                behav.execute("Walk_ZMP", 0, 0, rot )
            else:
                # Separate rotation from translation
                # TODO: this should be handled by the walking behavior
                # if abs(rot) > 6:
                #     behav.execute("Walk_ZMP", 0, 0, rot )
                # else:
                #     behav.execute("Walk_ZMP",ball[0],ball[1], 0 )
                j = i % 600
                if ( j < 300):
                    #behav.execute("Walk_ZMP",0, 0., -35 )
                    behav.execute("Walk_ZMP",0.1, 0.1, 0 )
                    print("left")
                elif (j<600):
                    #behav.execute("Walk_ZMP",0, 0.1, -35 )
                    behav.execute("Walk_ZMP",0.1, -0.1, 0 )
                    print("right")
                elif (i<900):
                    behav.execute("Walk_ZMP",0, -0.2, 35 )
                elif (i<2000):
                    behav.execute("Walk_ZMP",0, -0.2, -35 )
                elif (i<2500):
                    behav.execute("Walk_ZMP",0.1, 0, 30 )
                elif (i<3000):
                    behav.execute("Walk_ZMP",0.1, 0.1, -35 )
                elif (i<3500):
                    behav.execute("Walk_ZMP",0., 0., 35 )
                else:
                    behav.execute("Walk_ZMP",0., 0., 0 )
                        
                    
                
                
                # import time
                # time.sleep(0.1)

            player.scom.commit_and_send( player.world.robot.get_command() ) 
            player.scom.receive()

       
        #Clear drawings and close sockets
        Draw.clear_all()
        player.scom.close()
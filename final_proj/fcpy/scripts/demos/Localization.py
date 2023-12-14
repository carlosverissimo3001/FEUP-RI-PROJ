
from math import cos, sin
from world.Other_Robot import Other_Robot
import numpy as np
from utils.Math_Utils import Math_Utils as U
from utils.Draw import Draw
from scripts.commons.Script import Script
from cpp.localization import localization
from agent.commons.Base_Agent import Base_Agent as Agent

class Localization():

    def __init__(self,script:Script) -> None:
        self.script = script
        
        
    def execute(self):

        a = self.script.args

        # Team configuration 
        robot_types = (1,2,3,4)
        unums = range(1,len(robot_types)+1)
        enable_log = [False]*len(robot_types)
        enable_draw = [True]*len(robot_types)

        #Server IP, Agent Port, Monitor Port, Uniform Number, Robot Type, Team name, Enable Log, Enable Draw
        self.script.batch_create(Agent, ((a.i,a.p,a.m,u,u-1,a.t,True,True) for u in range(1,5)))
        self.script.batch_create(Agent, ((a.i,a.p,a.m,1,0,"Opponent",True,True),)) # one opponent
        no_of_players = len(self.script.players)

        #Beam players
        beam_home_team = [(-cos(i*0.01745329)*3-7,-sin(i*0.01745329)*3,i) for i in range(0,359,round(360 / len(robot_types)))]
        beam_away_team = [(-3,-4,0)]
        self.script.batch_commit_beam( beam_home_team + beam_away_team )

        #Focused player
        focused = 0
        interval_MAX = 200
        interval = interval_MAX

        #Execute
        for _ in range(interval_MAX * no_of_players * 10): #run 10x
            self.script.batch_execute_behavior("Squat")
            self.script.batch_commit_and_send()
            p : Agent
            for i,p in enumerate(self.script.players):
                p.scom.receive() #receive message and process localization data

                if i==focused and p.world.vision_is_up_to_date:
                    Draw.clear_all()

                    #localization drawings and report
                    if p.world.robot.loc_is_up_to_date:
                        localization.print_python_data()
                        localization.draw_visible_elements(not p.world.team_side_is_left)
                        localization.print_report()
                        p.world.draw.circle( p.world.ball_abs_pos, 0.1,6,Draw.Color.purple_magenta,"ball")
                    else:
                        p.world.draw.annotation( p.world.robot.cheat_abs_pos, "Not enough visual data!", Draw.Color.red,"localization")
                    
                    for r in p.world.teammates:
                        if r.state_last_update != 0:  #skip if other robot was not yet seen
                            self._draw_other_robot(p, r, Draw.Color.white)
                    
                    for r in p.world.opponents:
                        if r.state_last_update != 0:  #skip if other robot was not yet seen
                            self._draw_other_robot(p, r, Draw.Color.red)

                    p.world.draw.flush("other_robots")

            interval -= 1
            if interval == 0:
                interval = interval_MAX
                focused = focused+1 if focused<no_of_players-1 else 0 


        #Clear drawings and close sockets
        Draw.clear_all()
        self.script.batch_close()



    def _draw_other_robot(self, p:Agent, r:Other_Robot, team_color):
        #p - player that sees
        #r - other robot (player that is seen)

        white = Draw.Color.white
        green = Draw.Color.green_light
        gray = Draw.Color.gray_20

        time_diff = p.world.time_local_ms - r.state_last_update
        if time_diff > 0:
            white = Draw.Color.gray_50
            green = Draw.Color.get(107, 139, 107)
            gray = Draw.Color.gray_50

            
        #orientation
        if len(r.state_abs_pos)==3:
            line_tip = r.state_abs_pos + (0.5*U.deg_cos(r.state_orientation),0.5*U.deg_sin(r.state_orientation),0)
            p.world.draw.line( r.state_abs_pos, line_tip, 3, white, "other_robots", False)
        else:
            temp_pos = U.to_3d(r.state_abs_pos, 0.3)
            line_tip = temp_pos + (0.5*U.deg_cos(r.state_orientation),0.5*U.deg_sin(r.state_orientation),0)
            p.world.draw.line( temp_pos, line_tip, 3, Draw.Color.yellow, "other_robots", False)

        #body parts
        for pos in r.state_body_parts_abs_pos.values():
            p.world.draw.sphere( pos, 0.07, green,"other_robots", False)

        #player ground area
        p.world.draw.circle( r.state_ground_area[0], r.state_ground_area[1], 6, team_color,"other_robots", False)

        #distance
        midpoint = (r.state_abs_pos[0:2] + p.world.robot.loc_head_position[0:2])/2
        p.world.draw.line( r.state_abs_pos[0:2], p.world.robot.loc_head_position[0:2], 1, gray, "other_robots", False)
        p.world.draw.annotation( midpoint, f'{r.state_horizontal_dist:.2f}m', white, "other_robots", False)

        #velocity
        arrow_tip = r.state_abs_pos[0:2] + r.state_filtered_velocity[0:2]
        p.world.draw.arrow( r.state_abs_pos[0:2], arrow_tip, 0.2, 4, green, "other_robots", False)

        #state
        state_color = white if not r.state_fallen else Draw.Color.yellow
        p.world.draw.annotation( (r.state_abs_pos[0],r.state_abs_pos[1],1), 
                        f"({r.unum}) {'Fallen' if r.state_fallen else 'Normal'}", state_color, "other_robots", False)
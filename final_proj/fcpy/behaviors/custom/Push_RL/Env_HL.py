import math
from typing import List
import numpy as np
from utils.Math_Utils import Math_Utils as U
from behaviors.custom.Step.Step_Generator import Step_Generator
from world.Other_Robot import Other_Robot
from world.World import World


class Env_HL():
    COLS = 16 # radar specs
    LINS = 5  # radar specs

    def __init__(self, world : World) -> None:

        self.world = world
        
        # State space  
        self.obs = np.zeros(163, np.float32)
        

    def fill_radar(self, radar, team:List[Other_Robot], radar_part, RADIAL_START, RADIAL_MULT):

        w = self.world
        bp = w.ball_abs_pos[:2]
        C = self.COLS
        L = self.LINS # radial lines

        vec_b_goal = (15.5,0) - bp
        vec_b_goal_absdir = U.vector_angle(vec_b_goal)

        dist_closest_player = 10

        for t in team:
            if w.time_local_ms - t.state_last_update > 500: # only accept recently seen players
                continue

            vec_b_opp = t.state_abs_pos[:2] - bp
            dist_b_opp = np.linalg.norm(vec_b_opp)

            if dist_b_opp < dist_closest_player:
                dist_closest_player = dist_b_opp

            #------ find 2 angular columns and their respective weight
 
            vec_b_opp_dir = U.normalize_deg( U.vector_angle(vec_b_opp) - vec_b_goal_absdir )

            div, mod = divmod(vec_b_opp_dir+180, 360/C)

            zone = int(div) % C # angular zone {0,...,C-1}
            prog = mod * C / 360 # progress inside zone

            ang_column_weight_1 = zone, 1 - prog  # angular column + radar weight
            ang_column_weight_2 = (zone+1)%C, prog            # angular column + radar weight

            #------ find 2 radial lines and their respective weight

            zone = max(1, 1 + math.log((dist_b_opp+1e-6)/RADIAL_START, RADIAL_MULT))
            prog = zone % 1
            zone = math.ceil(zone)-1

            if zone >= L: continue # check if opp is too distant

            rad_line_weight_1 = None if zone == 0 else (zone-1, 1 - prog) # radial line + radar weight
            rad_line_weight_2 = zone, 1 if zone == 0 else prog            # radial line + radar weight

            #------ compute final radar cells weight

            if rad_line_weight_1 is not None:
                radar[radar_part, rad_line_weight_1[0], ang_column_weight_1[0]] += rad_line_weight_1[1] * ang_column_weight_1[1]
                radar[radar_part, rad_line_weight_1[0], ang_column_weight_2[0]] += rad_line_weight_1[1] * ang_column_weight_2[1]

            radar[radar_part, rad_line_weight_2[0], ang_column_weight_1[0]] += rad_line_weight_2[1] * ang_column_weight_1[1]
            radar[radar_part, rad_line_weight_2[0], ang_column_weight_2[0]] += rad_line_weight_2[1] * ang_column_weight_2[1]
    
        return dist_closest_player   


    def observe(self, init=False):

        if init: # reset variables
            self.output = 0

        radar = np.zeros((2,self.LINS,self.COLS)) # teammates/opponents, L radial lines, C angular columns
        
        RADIAL_START = 0.3 # distance of first radial line
        RADIAL_MULT = 1.7  # distance of radial line d(x) = RADIAL_MULT * d(x-1)

        dist_closest_tm  = self.fill_radar(radar, self.world.teammates, 0, RADIAL_START, RADIAL_MULT)
        dist_closest_opp = self.fill_radar(radar, self.world.opponents, 1, RADIAL_START, RADIAL_MULT)

        self.obs = np.append(radar.flatten(), (dist_closest_tm*0.5, dist_closest_opp*0.5, self.output/40))

        return self.obs


    def execute(self, action):

        vec_b_goal = (15.5,0) - self.world.ball_abs_pos[:2]
        vec_b_goal_absdir = U.vector_angle(vec_b_goal)

        rel_direction = action[0] * 60
        self.output += np.clip( U.normalize_deg( rel_direction - self.output ), -45, 45) # clipped transformation
        abs_direction = U.normalize_deg(vec_b_goal_absdir + self.output)

        return abs_direction



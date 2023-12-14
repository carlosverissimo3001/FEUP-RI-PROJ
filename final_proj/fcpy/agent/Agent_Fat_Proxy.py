from agent.commons.Role_Manager import Role_Manager
from agent.commons.Base_Agent import Base_Agent
from cpp.a_star import a_star
import numpy as np
from utils.Math_Utils import Math_Utils as U
import math
from formations.Formation_2022 import Formation
from world.World import World

class Agent(Base_Agent):

    def __init__(self, host:str, agent_port:int, monitor_port:int, unum:int,
                 team_name:str, enable_log, enable_draw, wait_for_server=True) -> None:

        # define robot type
        robot_type = (1,1,2,2,0,0,0,0,0,0,0)[unum-1]

        # initialize base agent
        super().__init__(host, agent_port, monitor_port, unum, robot_type, team_name, enable_log, enable_draw, None, wait_for_server)


        self.our_max_speed = 0.7
        self.their_max_speed = 0.7
        self.role_manager = Role_Manager(self.world, self.our_max_speed, self.their_max_speed, Formation(self.world))
        self.enable_draw = enable_draw

        # ------------------------------------------------------------------ Define state of each player

        self.it = None # current intention
        self.sk = None # current skill
        self.kt = None # current kick target
        self.kd = None # current kick distance (is larger than ball <-> target, when shooting)
        self.is_push_1v1 = False # False if current push is in critical areas (near goals), True if opponent is very close
        self.k_stamp = [0,-1] # last time the kick was set while the ball was close, and respective play mode

        # Role name-index mapping
        self.R_GK = 0   # Goalkeeper
        self.R_CB = 1   # Central Back
        self.R_MM1 = 2  # Man-marking player 1
        self.R_MM2 = 3  # Man-marking player 2
        self.R_MM3 = 4  # Man-marking player 3
        self.R_MM4 = 5  # Man-marking player 4
        self.R_LS = 6   # Left  support behind active player (does not exist while defending)
        self.R_RS = 7   # Right support behind active player (does not exist while defending)
        self.R_LF = 8   # Left  support in front of active player
        self.R_RF = 9   # Right support in front of active player
        self.R_AP = 10  # Main active player (closest to ball)
        self.R_AP2 = 11 # Secondary active player (2nd closest to ball) (does not exist while attacking)
        self.R_AP3 = 12 # Secondary active player (3rd closest to ball) (does not exist while attacking)

        # Intentions
        self.I_MOVE = 0
        self.I_PUSH = 2
        self.I_KICK = 3
        self.I_IDLE = 5
        self.I_ACTIVE_BEAM = 6
        self.I_PASSIVE_BEAM = 7
        
        # Skills
        self.S_WALK = 0
        self.S_WALK_KICK = 2

        # Special skills
        self.SS_BEAM = 5    # beam to formation
        self.SS_IDLE = 6

        self.INTENTIONS_NAMES = {None:"", self.I_MOVE: "move", self.I_PUSH: "push", self.I_KICK: "kick", self.I_IDLE: "idle", self.I_ACTIVE_BEAM: "active_beam", self.I_PASSIVE_BEAM: "passive_beam"}
        self.SKILLS_NAMES = {None:"", self.S_WALK: "walk", self.S_WALK_KICK: "walk_kick", self.SS_BEAM: "beam", self.SS_IDLE: "idle"}

        # ------------------------------------------------------------------ Define groups of play modes

        # play mode groups
        self.MG_OUR_KICK = 0
        self.MG_THEIR_KICK = 1
        self.MG_ACTIVE_BEAM = 2
        self.MG_PASSIVE_BEAM = 3
        self.MG_OTHER = 4 # play on, game over
                       
        self.OUR_KICK_PLAY_MODES  = [World.M_OUR_KICKOFF, World.M_OUR_KICK_IN, World.M_OUR_CORNER_KICK, World.M_OUR_GOAL_KICK, World.M_OUR_FREE_KICK, World.M_OUR_PASS, World.M_OUR_DIR_FREE_KICK, World.M_OUR_OFFSIDE]
        self.THEIR_KICK_PLAY_MODES = [World.M_THEIR_KICK_IN,World.M_THEIR_CORNER_KICK,World.M_THEIR_GOAL_KICK,World.M_THEIR_FREE_KICK,World.M_THEIR_PASS,World.M_THEIR_DIR_FREE_KICK, World.M_THEIR_OFFSIDE]
        self.ACTIVE_BEAM_PLAY_MODES  = [World.M_BEFORE_KICK_OFF,World.M_THEIR_GOAL]
        self.PASSIVE_BEAM_PLAY_MODES = [World.M_OUR_GOAL,World.M_THEIR_KICKOFF]
        self.OTHER_PLAY_MODES = [World.M_PLAY_ON, World.M_GAME_OVER]


    def set_best_kick(self, best_short_kick, best_long_kick, is_long_to_goal):

        w = self.world
        PM = w.play_mode
        ball_2d = w.ball_abs_pos[:2]
        ball_dist = np.linalg.norm(w.robot.loc_head_position[:2] - ball_2d)
        shoot = False # true if shooting to goal
        ball_is_far = ball_dist > 0.5

        # set new kick if ball is far or it was set long ago or the play mode changed
        if ball_is_far or self.k_stamp[0] < self.world.time_local_ms - 3000 or self.k_stamp[1] != PM:

            if PM == w.M_PLAY_ON or PM == w.M_OUR_DIR_FREE_KICK: # allow shooting to goal
                if (is_long_to_goal and best_long_kick[2] > 0.2 and ball_2d[0]>3) or (not is_long_to_goal and best_long_kick[2]>best_short_kick[2]):
                    best_kick_pos = best_long_kick[:2]
                    shoot = is_long_to_goal
                else:
                    best_kick_pos = best_short_kick[:2]   
            else: # avoid shooting to goal
                if best_short_kick[2] > best_long_kick[2] or is_long_to_goal:
                    best_kick_pos = best_short_kick[:2]
                else:
                    best_kick_pos = best_long_kick[:2]

            distance = np.linalg.norm(best_kick_pos - ball_2d)

            if shoot:
                self.kd = 15 if 1 < distance < 7 else 20 # 20 means aerial, 15 means avoid aerial
            else:
                self.kd = distance # actual length if passing

            self.kt = best_kick_pos

            if not ball_is_far:
                self.k_stamp[0] = self.world.time_local_ms
                self.k_stamp[1] = PM




    def think_and_send(self):

        w = self.world
        r = self.world.robot 
        my_head_pos_2d = r.loc_head_position[:2]
        my_ori = r.imu_torso_orientation
        ball_2d = w.ball_abs_pos[:2]
        ball_vec = ball_2d - my_head_pos_2d
        ball_ori = U.vector_angle(ball_vec)
        ball_dist = np.linalg.norm(ball_vec)
        PM = w.play_mode
        fat_proxy_command = ""

        #--------------------------------------- 1. Prepare internal variables

        '''
        Play mode group:
        - Certain play modes share characteristics, so it makes sense to group them
        '''

        if PM in self.OTHER_PLAY_MODES: # most common group
            PM_GROUP = self.MG_OTHER
        elif PM in self.OUR_KICK_PLAY_MODES:
            PM_GROUP = self.MG_OUR_KICK
        elif PM in self.THEIR_KICK_PLAY_MODES:
            PM_GROUP = self.MG_THEIR_KICK
        elif PM in self.ACTIVE_BEAM_PLAY_MODES:
            PM_GROUP = self.MG_ACTIVE_BEAM
        elif PM in self.PASSIVE_BEAM_PLAY_MODES:
            PM_GROUP = self.MG_PASSIVE_BEAM
        else:
            raise ValueError(f'Unexpected play mode ID: {PM}')

        self.PM_GROUP = PM_GROUP # allow access in member functions

        # Get distance of closest standing opponent
        standing_opp_d = min(np.linalg.norm(p.state_abs_pos[:2]-ball_2d) if (p.state_last_update != 0 and w.time_local_ms - p.state_last_update <= 360 and not p.state_fallen) else 100 for p in w.opponents)

        #--------------------------------------- 2. Get role

        # add bias to ball position in passive kick play modes so that the AP is placed between the ball and our goal
        biased_ball_pos = np.copy(ball_2d)
        if PM_GROUP == self.MG_THEIR_KICK:
            unit_vec_ball_goal = U.normalize_vec( (-15.1,0) - ball_2d )
            if PM == w.M_THEIR_GOAL_KICK:
                pass
            elif PM == w.M_THEIR_PASS:
                biased_ball_pos += unit_vec_ball_goal * 0.2
            elif PM == w.M_THEIR_CORNER_KICK:
                biased_ball_pos += unit_vec_ball_goal * 0.6 + (0.40,0)
            else: # kick in, free kick, direct free kick, offside
                biased_ball_pos += unit_vec_ball_goal * 0.5

        role_idx, role_position, priority_unums, best_short_kick, best_long_kick, is_long_to_goal = self.role_manager.get_role( biased_ball_pos, PM != w.M_PLAY_ON )

        #--------------------------------------- 3. Decide intention

        if PM == w.M_GAME_OVER:
            self.it = self.I_IDLE
        elif PM_GROUP == self.MG_ACTIVE_BEAM:
            self.it = self.I_ACTIVE_BEAM
        elif PM_GROUP == self.MG_PASSIVE_BEAM:
            self.it = self.I_PASSIVE_BEAM
        elif role_idx != self.R_AP: # if not the AP, beam during our kickoff, or move in any other case
            if PM == w.M_OUR_KICKOFF:
                self.it = self.I_PASSIVE_BEAM
            else:
                self.it = self.I_MOVE
        else: # from this point forward, it only concerns the AP
            if ball_dist > 1:
                self.it = self.I_MOVE 
            elif PM_GROUP == self.MG_OUR_KICK:
                self.it = self.I_KICK
                if PM == w.M_OUR_KICKOFF:
                    self.sk = self.S_WALK_KICK
                    self.kt = (-3,0)
                    self.kd = 3
                else:
                    self.set_best_kick(best_short_kick, best_long_kick, is_long_to_goal)
            elif PM_GROUP == self.MG_THEIR_KICK:
                self.it = self.I_MOVE
            elif PM == w.M_PLAY_ON:
                if self.sk == self.S_WALK_KICK: # commit to kick, do not change intention or skill
                    if ball_2d[0] < -2 and standing_opp_d < 1.5:
                        self.scom.commit_pass_command() 
                elif self.sk == self.S_WALK:
                    self.it = self.I_KICK
     
            else:
                raise ValueError(f'Unexpected play mode ID: {PM}')

        if self.enable_draw: # this is redundant since the agent will ignore the draw command, but it saves some time
            d = w.draw
            d.annotation((*my_head_pos_2d,0.6), f"{self.INTENTIONS_NAMES[self.it]} {self.SKILLS_NAMES[self.sk]}", d.Color.white if len(priority_unums)==0 else d.Color.gray_50, "role")


        #--------------------------------------- 4. Decide active skill    


        if self.it == self.I_IDLE:
            self.sk = self.SS_IDLE
        elif self.it == self.I_ACTIVE_BEAM:
            if np.linalg.norm(self.role_manager.active_beam_position - my_head_pos_2d) > 0.1:
                self.sk = self.SS_BEAM
            elif r.behavior.is_ready("Get_Up"):
                self.sk = self.SS_BEAM
            else:
                self.sk = self.SS_IDLE
        elif self.it == self.I_PASSIVE_BEAM:
            if np.linalg.norm(self.role_manager.passive_beam_position - my_head_pos_2d) > 0.1:
                self.sk = self.SS_BEAM
            elif r.behavior.is_ready("Get_Up"):
                self.sk = self.SS_BEAM
            else:
                self.sk = self.SS_IDLE
        elif self.it == self.I_MOVE:
            self.sk = self.S_WALK
        elif self.it == self.I_PUSH:
            self.sk = self.S_WALK

        elif self.it == self.I_KICK:
            if self.sk == self.S_WALK_KICK:
                pass
            elif ball_dist < 1:
                self.sk = self.S_WALK_KICK
                self.set_best_kick(best_short_kick, best_long_kick, is_long_to_goal)
            else:
                self.sk = self.S_WALK
        else:
            raise NotImplementedError


        #--------------------------------------- 5. Define skill target

        if self.it == self.I_KICK:
            if self.sk == self.S_WALK:
                walk_target, walk_orientation, walk_distance = r.path_manager.get_path_to_target(ball_2d, torso_ori=ball_ori, is_aggressive=True)
            elif self.sk == self.S_WALK_KICK:
                ang = U.vector_angle(self.kt - ball_2d)
                walk_target, walk_orientation, walk_distance = r.path_manager.get_path_to_ball(1,ang,-0.15,0,ang, safety_margin=0.2)
        elif self.it == self.I_MOVE:

            is_active = role_idx in [self.R_AP, self.R_AP2, self.R_AP3]

            # --------------------------------- 1. Define walk orientation

            opp_goal_vec = np.array([15,0]) - my_head_pos_2d
            if role_idx in [self.R_LF, self.R_RF]: # LF/RF are oriented towards the opponents' goal
                walk_orientation = U.vector_angle(opp_goal_vec)
            elif is_active: # if chasing the ball, orient towards its predicted position
                walk_orientation = U.vector_angle(role_position - my_head_pos_2d)
            else:
                role_vec = role_position - my_head_pos_2d
                role_dist = np.linalg.norm(role_vec)
                role_ori = U.vector_angle(role_vec)
                
                if role_dist < 0.3: # turn to ball
                    walk_orientation = ball_ori
                elif role_dist > 2.5: # turn to role
                    walk_orientation = role_ori
                else: # turn to closest orientation (ball or role)
                    ball_ori_diff = abs(U.normalize_deg(ball_ori - my_ori))
                    role_ori_diff = abs(U.normalize_deg(role_ori - my_ori))
                    walk_orientation = ball_ori if ball_ori_diff < role_ori_diff else ball_ori

            # --------------------------------- 2. Define next walk target

            walk_target, _, walk_distance = r.path_manager.get_path_to_target(role_position, priority_unums=priority_unums, is_aggressive=is_active)
           

        elif self.it == self.I_PUSH:
          
            if my_head_pos_2d[0] < 8: 
                if standing_opp_d < ball_dist + 0.1: # if enemy is (almost) closer
                    target = U.normalize_vec( ball_2d - (-15.5,0) ) + ball_2d # push away from our goal
                else:
                    target = (10,9 if my_head_pos_2d[1]>0 else -9) # push to lateral points
            else:
                target = (15.1,np.clip( ball_2d[1], -0.7, 0.7)) # push to their goal

            vec_ball_target_unit = U.normalize_vec(target - ball_2d) # unit vector ball->target

            # if near dribble area, keep dribbling
            dribble_start = ball_2d + 0.1 * vec_ball_target_unit
            dribble_end   = ball_2d - 0.3 * vec_ball_target_unit
            dribble_ori   = U.vector_angle(vec_ball_target_unit)

            if self.enable_draw:
                ang = dribble_ori / 180 * math.pi
                w.team_draw.line(my_head_pos_2d, my_head_pos_2d+(math.cos(ang),math.sin(ang)), 2, w.draw.Color.red,  "path", False)


            if U.distance_point_to_segment(my_head_pos_2d, dribble_start, dribble_end) < 0.13: # if aligned, dribble to goal
                walk_distance = 0.20 if self.is_push_1v1 else 10 # high distance to motivate full speed
            else: # otherwise, align robot
                x_dev = -0.15 if self.is_push_1v1 else -0.20
                target, target_ori, walk_distance = r.path_manager.get_path_to_ball(x_ori=dribble_ori, x_dev=x_dev, is_aggressive=True, safety_margin=0.3) # new target: aligned position
                # orientation when aligning with target: angle between current and ball
                dribble_ori = U.normalize_deg(target_ori - my_ori) * 0.2 + my_ori 


            walk_target = target
            walk_orientation = dribble_ori


        #--------------------------------------- 6. Execute skill (or get up)

        if self.sk == self.SS_BEAM:
            pos = self.role_manager.active_beam_position if self.it == self.I_ACTIVE_BEAM else self.role_manager.passive_beam_position
            self.scom.commit_beam(pos, U.vector_angle((-pos[0],-pos[1])))
        elif self.sk == self.S_WALK:
            target_vec = U.normalize_vec( U.rotate_2d_vec( walk_target - my_head_pos_2d, -my_ori ) ) * 90 * min(walk_distance*3,1)
            target_ori = np.clip(U.normalize_deg(walk_orientation - my_ori), -60, 60)
            fat_proxy_command += f"(proxy dash {target_vec[0]:.2f} {target_vec[1]:.2f} {target_ori:.1f})"

        elif self.sk == self.SS_IDLE:
            fat_proxy_command += f"(proxy dash 0 0 0)"
        elif self.sk == self.S_WALK_KICK:

            target_vec = U.normalize_vec( U.rotate_2d_vec( walk_target - my_head_pos_2d, -my_ori ) ) * 70
            target_ori = np.clip(U.normalize_deg(walk_orientation - my_ori), -60, 60)
            fat_proxy_command += f"(proxy dash {target_vec[0]:.2f} {target_vec[1]:.2f} {target_ori:.1f})"

            if ball_dist < 0.24 or ((w.ball_abs_speed > 4 or standing_opp_d < 0.5) and ball_dist < 0.4):
                if self.kd == 20:
                    vert = 35
                elif self.kd == 15:
                    vert = 20
                else:
                    vert = 30
                fat_proxy_command += f"(proxy kick {min(self.kd*1.2,10):.2f} {U.normalize_deg( U.target_rel_angle(my_head_pos_2d,my_ori,self.kt)):.2f} {vert})"

        else:
            raise NotImplementedError


        #--------------------------------------- 7. Broadcast
        self.radio.broadcast() # commit announcement to all players

        #--------------------------------------- 8. Send to server

        self.scom.commit_and_send( fat_proxy_command.encode() ) 

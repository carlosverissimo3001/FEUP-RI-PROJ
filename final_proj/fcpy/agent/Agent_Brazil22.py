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
        robot_type = (0,0,2,2,1,1,1,1,1,1,1)[unum-1]

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
        self.I_DRIBBLE = 1
        self.I_PUSH = 2
        self.I_KICK = 3
        self.I_GETUP = 4
        self.I_IDLE = 5
        self.I_ACTIVE_BEAM = 6
        self.I_PASSIVE_BEAM = 7
        
        # Skills
        self.S_WALK = 0
        self.S_DRIBBLE = 1
        self.S_WALK_KICK = 2
        self.S_DRIBBLE_KICK = 3
        self.S_PUSH = 4

        # Special skills
        self.SS_GET_UP = 5  # get up
        self.SS_BEAM = 6    # beam to formation
        self.SS_IDLE = 7

        self.INTENTIONS_NAMES = {None:"(none)", self.I_MOVE: "move", self.I_DRIBBLE: "dribble", self.I_PUSH: "push", self.I_KICK: "kick", self.I_GETUP: "get_up", self.I_IDLE: "idle", self.I_ACTIVE_BEAM: "active_beam", self.I_PASSIVE_BEAM: "passive_beam"}
        self.SKILLS_NAMES = {None:"(none)", self.S_WALK: "walk", self.S_DRIBBLE: "dribble", self.S_WALK_KICK: "walk_kick", self.S_DRIBBLE_KICK: "dribble_kick", self.SS_GET_UP: "get_up", self.SS_BEAM: "beam", self.SS_IDLE: "idle", self.S_PUSH: "push"}

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
        if ball_is_far or self.k_stamp[0] < self.world.time_local_ms - 5000 or self.k_stamp[1] != PM:

            if PM == w.M_PLAY_ON or PM == w.M_OUR_DIR_FREE_KICK: # allow shooting to goal
                if (is_long_to_goal and best_long_kick[2] > 0.4) or best_long_kick[2]>best_short_kick[2]:
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
                self.kd = 15 if 1.2 < distance < 11 else 20 # 20 means aerial, 15 means avoid aerial
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

        max_dist = 18.0 if self.sk == self.S_DRIBBLE else 16.0 # avoid starting to dribble if near restricted zone
        AVOID_DRIBBLE = np.linalg.norm((-15,0) - ball_2d) > (40.0 if w.goals_scored < w.goals_conceded + 2 else max_dist)

        if PM == w.M_GAME_OVER:
            self.it = self.I_IDLE
        elif PM_GROUP == self.MG_ACTIVE_BEAM:
            self.it = self.I_ACTIVE_BEAM
        elif PM_GROUP == self.MG_PASSIVE_BEAM:
            self.it = self.I_PASSIVE_BEAM
        elif self.it == self.I_GETUP:
            pass # the intention is kept until the behavior is finished
        elif r.behavior.is_ready("Get_Up"): # detect fall
            self.it = self.I_GETUP
        elif role_idx != self.R_AP: # if not the AP, beam during our kickoff, or move in any other case
            if PM == w.M_OUR_KICKOFF:
                self.it = self.I_PASSIVE_BEAM
            else:
                if self.it == self.I_DRIBBLE and self.sk == self.S_DRIBBLE and (
                    w.time_local_ms - w.ball_last_seen < 120 and -0.2<w.ball_rel_torso_cart_pos[0]<0.4 and 0.3<w.ball_rel_torso_cart_pos[0]<0.3):
                    pass # keep dribbling if not active player but the ball is still close
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
                elif self.sk == self.S_DRIBBLE:
                    self.it = self.I_PUSH if AVOID_DRIBBLE else self.I_DRIBBLE
                else: # Skill: walk, push, none
                     
                    # Distance to closest opp ground area
                    opp_d = 10
                    opp_d_standing = 10 # distance to closest standing opponent
                    vec_ball_goal = (-15.6,0) - ball_2d

                    for p in w.opponents:
                        if w.time_local_ms - p.state_last_update <= 300:
                            vec_p_goal = (-15.6,0) - p.state_ground_area[0]
                            if U.vectors_angle(vec_ball_goal, vec_p_goal) < 80: # opponent in direction of goal
                                dist = max(0, np.linalg.norm(p.state_ground_area[0]-ball_2d)-p.state_ground_area[1])
                                if dist < opp_d:
                                    opp_d = dist
                                if not p.state_fallen and dist < opp_d_standing:
                                    opp_d_standing = dist

                    if ball_2d[0] > 14.5 and abs(ball_2d[1]) < 1.6: 
                        self.it = self.I_PUSH # push if near opp goal
                    elif ball_2d[0] < -14.5 and np.linalg.norm(np.abs(ball_2d)-(15.02,  1.07)) < 0.5:
                        self.it = self.I_PUSH # push if near our goal posts
                    elif self.it == self.I_KICK and opp_d > 0.1:
                        pass # commit to kick intention up to a certain point
                    elif opp_d < ball_dist+0.3:
                        self.it = self.I_PUSH # push if opponent is dangerous
                    else:
                        best_kick_score = max(best_short_kick[2], best_long_kick[2])

                        if (not AVOID_DRIBBLE and ( best_long_kick[2] > 0.4 and is_long_to_goal and opp_d_standing > 0.8)) or (
                            AVOID_DRIBBLE and ( best_kick_score > 0.4 and opp_d_standing > 0.8 )):
                            self.it = self.I_KICK
                        else:
                            self.it = self.I_PUSH if AVOID_DRIBBLE else self.I_DRIBBLE

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
        elif self.it == self.I_GETUP:
            self.sk = self.SS_GET_UP
        elif self.sk == self.S_DRIBBLE:
            pass # wait for dribble to stop
        elif self.sk == self.S_PUSH and self.it in [self.I_MOVE,  self.I_KICK]:
            pass # wait for push to stop
        elif self.it == self.I_MOVE:
            self.sk = self.S_WALK
        elif self.it == self.I_PUSH:
            self.sk = self.S_PUSH

        elif self.it == self.I_DRIBBLE:
            # there is no direct transition from kick to dribble
            ball_hip = r.inv_kinematics.torso_to_hip_transform(w.ball_rel_torso_cart_pos)  
            ball_dist = np.linalg.norm(ball_hip[:2])

            # Dribble skill is kept until it ends
            if self.sk == None:
                self.sk = self.S_WALK       
            elif self.sk == self.S_WALK or self.sk == self.S_PUSH:
                if ball_dist < 0.8:
                    self.sk = self.S_DRIBBLE

        elif self.it == self.I_KICK:
            ball_hip = r.inv_kinematics.torso_to_hip_transform(w.ball_rel_torso_cart_pos) 
            if self.sk == self.S_DRIBBLE and False:
                # transition to kick at right phase
                if self.is_dribble_right_phase_to_kick():
                    self.sk = self.S_DRIBBLE_KICK
            else:
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
            elif self.sk != self.S_WALK_KICK and self.sk != self.S_DRIBBLE and self.sk != self.S_PUSH:
                print("not expected", self.sk)
        elif self.it == self.I_MOVE or (self.it == self.I_DRIBBLE and self.sk == self.S_WALK):

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
           


        #--------------------------------------- 6. Execute skill (or get up)

        if self.sk == self.SS_BEAM:
            pos = self.role_manager.active_beam_position if self.it == self.I_ACTIVE_BEAM else self.role_manager.passive_beam_position
            self.scom.commit_beam(pos, U.vector_angle((-pos[0],-pos[1])))
        elif self.sk == self.SS_GET_UP:
            if r.behavior.execute("Get_Up"):
                self.it = self.sk = None # if done, reset
        elif self.sk == self.S_WALK:
            r.behavior.execute("Walk_RL3", walk_target, True, walk_orientation, True, walk_distance) # target_2d, is_target_absolute, orientation, is_orientation_absolute, distance
        elif self.sk == self.S_DRIBBLE:
            if r.behavior.execute("Dribble_RL", None, None, 1, self.it != self.I_DRIBBLE):
                self.sk = None
        elif self.sk == self.SS_IDLE:
            r.behavior.execute("Zero_Bent_Knees_Auto_Head")
        elif self.sk == self.S_WALK_KICK:
            if self.kd > 10:
                r.behavior.execute("Kick_Long_RL", U.vector_angle(self.kt - ball_2d), self.kd > 15) # allow aerial if kd > 15
            else:
                r.behavior.execute("Kick_Short_RL", U.vector_angle(self.kt - ball_2d), self.kd-0.2)
        elif self.sk == self.S_PUSH:
            if r.behavior.execute("Push_RL", self.it != self.I_PUSH):
                self.sk = None
        else:
            raise NotImplementedError


        #--------------------------------------- 7. Broadcast
        self.radio.broadcast() # commit announcement to all players

        #--------------------------------------- 8. Send to server
        self.scom.commit_and_send( r.get_command() ) 

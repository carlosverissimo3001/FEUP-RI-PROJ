from world.World import World
from utils.Math_Utils import Math_Utils as U
from cpp.role_manager import role_manager
import numpy as np

'''
Role ID:
GK  (0)     CB  (1)                             Goalkeeper, Central Back
MM1 (2)     MM2 (3)     MM3 (4)    MM4 (5)      Man-marking players
LS  (6)     RS  (7)                             Support behind active player
LF  (8)     RF  (9)                             Support in front of active player
AP  (10)    AP2 (11)    AP3 (12)                Main active player, Secondary active players

Role precendence:
  Attacking  |  Defending
-------------+-------------
     AP      |     AP
     GK      |     GK
    LF+RF    |     AP2
   (LS+RS)   |   (4 MM)
   (4 MM)    |    (CB)
    (CB)     |     AP3
             |    LF+RF
'''

class Role_Manager():
    def __init__(self, world : World, our_max_speed, their_max_speed, formation) -> None:
        self.our_max_speed = our_max_speed
        self.their_max_speed = their_max_speed
        self.world = world
        self.formation = formation

        # GK beam position ( assume ball is at center )
        GK0 = self.formation(np.zeros(2))[0]

        # Beam position for our kick off
        self.active_beam_position  = [GK0, (-11,0), (-12,-8), (-12,8), (-6,-5), (-6,5), 
                                      (-0.5,-2.3), (-0.5,2.3), (-5,0), (-6,-1), (-0.5,-0.5)][world.robot.unum-1] 

        # Beam position for their kick off
        self.passive_beam_position = (-2.3,0) if world.robot.unum == 11 else self.active_beam_position

    def get_role(self, ball_2d, is_ball_stationary=False):
        ''' 
        Get role ID + position, and priority unums 
        
        Parameters
        ----------
        ball_2d: array_like
            ball position, real or not
        is_ball_stationary : bool
            in some play modes, we know the ball is stationary
        '''

        w = self.world
        r = self.world.robot

        #------------------------------- update internal ball velocity
        
        if len(w.ball_abs_pos_history) > 0 and not is_ball_stationary:
            # avg velocity over avg_size*0.06s
            avg_size = min(6, len(w.ball_abs_pos_history)) 
            internal_ball_vel = (w.ball_abs_pos[:2] - w.ball_abs_pos_history[avg_size-1][:2]) / (avg_size*World.VISUALSTEP)
        else:
            internal_ball_vel = np.zeros(2,dtype=float)

        #------------------------------- get position of roles

        GK, CB, LS, RS, LF, RF = self.formation(ball_2d)

        #------------------------------- assign roles to players

        # Add self to teammates list
        me = w.teammates[r.unum-1]
        me.state_last_update = r.loc_last_update
        me.state_abs_pos = r.loc_head_position
        me.state_fallen = r.loc_head_z < 0.3

        # Prepare list of parameters for the role manager
        rm_pos = lambda t,i: t.state_abs_pos[i] if t.state_last_update > 0 else 1000 # invalid coordinates if not yet seen   
        rm_teammates = [rm_pos(t,i) for t in w.teammates for i in range(2)] # 2d position of each teammate (incl self)
        rm_opponents = [rm_pos(t,i) for t in w.opponents for i in range(2)] # 2d position of each opponent

        rm_roles = [rr[i] for rr in [GK, CB, LS, RS, LF, RF] for i in range(2)] # 2d position of each role
        rm_my_index = r.unum-1 # returns from 'role_manager.compute()' as soon as this player is assigned
        rm_is_teammate_down = [float(t.state_fallen) for t in w.teammates] # 1 if teammate is down, 0 otherwise

        rm_param = np.array([*rm_teammates,*rm_opponents,*rm_roles,*ball_2d,*internal_ball_vel,*rm_is_teammate_down,
                             rm_my_index,self.our_max_speed,self.their_max_speed],np.float32)
        rm_ret = role_manager.compute(rm_param)
        rm_ret_assignments = rm_ret[:33]
        rm_best_short_kick = rm_ret[33:36]
        rm_best_long_kick = rm_ret[36:39]
        rm_is_long_to_goal = bool(rm_ret[39])
        rm_assignment_id = rm_ret_assignments[::3]
        rm_assignment_pos = rm_ret_assignments[rm_my_index*3+1:rm_my_index*3+3]

        role_idx = int(rm_assignment_id[rm_my_index])

        # Return role position, and priority unums
        if role_idx == 10:
            priority_unums = [] # there are no priority unums
        elif role_idx == 11:
            priority_unums = np.where(rm_assignment_id == 10)[0]+1 # AP is the only priority role
        elif role_idx == 12:
            priority_unums =  np.where((rm_assignment_id > 9) & (rm_assignment_id < 12))[0]+1 # AP, AP2 are the only priority roles
        elif role_idx == 8 or role_idx == 9:
            priority_unums =  np.where(rm_assignment_id > 9)[0]+1 # AP, AP2, AP3 are the only priority roles
        else:
            priority_unums =  np.where((rm_assignment_id > 7))[0]+1 # APs, LF, RF are the only priority roles


        return role_idx, rm_assignment_pos, priority_unums, rm_best_short_kick, rm_best_long_kick, rm_is_long_to_goal


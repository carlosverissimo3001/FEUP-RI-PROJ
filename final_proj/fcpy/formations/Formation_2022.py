import numpy as np
from numpy import inf
import math
from utils.Math_Utils import Math_Utils as U

class Formation():

    def __init__(self, world) -> None:

        self.opp_goal = np.array([15.1,0])
        self.own_goal = np.array([-15.1,0])
        self.world = world


    def _expand(self, pos, vx, vy):
        '''
        Expand position is the direction of (vx,vy) without going out of bounds or inside opponent's goal
        '''
        MAX_X = 15.3
        MIN_X = -15.5
        MM_Y = 9.5   # limit for min and max

        ratio_1 = np.clip(vx, MIN_X-pos[0], MAX_X-pos[0]) / vx if vx != 0 else 1
        ratio_2 = np.clip(vy, -MM_Y-pos[1], MM_Y -pos[1]) / vy if vy != 0 else 1
        ret =  pos + np.array([vx,vy]) * min(ratio_1, ratio_2)

        if ret[0]>14.8 and abs(ret[1])<1.5: # eject from opponent goal
            ret[1] = np.sign(ret[1])*1.5

        return ret


    def _restrict_pair(self, left, right, y_dist, old_pair):
        '''
        y_dist: restricts left player from crossing the field to the right side for more than y_dist (and vice versa)
        old_pair: weighted average of old value and new value
        '''

        if old_pair is not None:
            old_weight = old_pair[2]
            left  = old_pair[0]*old_weight + left *(1-old_weight)
            right = old_pair[1]*old_weight + right*(1-old_weight)
        
        if y_dist is not None:
            left[1]  = max(y_dist, left[1])
            right[1] = min(right[1], -y_dist)

        return left, right



    def _get_pair_from_opp(self,center,side_dist,y_dist=None, old_pair=None):
        '''
        Center: point from line segment opp_goal<->ball at given distance from opp_goal
        Side distance: distance from center, perpendicular to the line segment opp_goal<->ball
        '''

        center = self.vec_opp_unit * np.clip(center[0], center[1], center[2]) + self.opp_goal
        left  = self._expand( center,  self.vec_opp_unit[1]*side_dist,-self.vec_opp_unit[0]*side_dist )
        right = self._expand( center, -self.vec_opp_unit[1]*side_dist, self.vec_opp_unit[0]*side_dist )

        return self._restrict_pair(left,right,y_dist,old_pair)

    def _get_pair_from_own(self,center,side_dist,y_dist=None, old_pair=None):
        '''
        Center: point from line segment own_goal<->ball at given distance from own_goal
        Side distance: distance from center, perpendicular to the line segment own_goal<->ball
        '''

        center = self.vec_own_unit * np.clip(center[0], center[1], center[2]) + self.own_goal
        left  = self._expand( center, -self.vec_own_unit[1]*side_dist, self.vec_own_unit[0]*side_dist )
        right = self._expand( center,  self.vec_own_unit[1]*side_dist,-self.vec_own_unit[0]*side_dist )
        
        return self._restrict_pair(left,right,y_dist,old_pair)



    def __call__(self, ball_2d):
        '''
        Formation as a function of the ball position

        Parameters
        ----------
        ball_2d: array_like
            ball position, real or not
        '''

        w = self.world
    
        self.vec_opp = vec_opp = ball_2d - self.opp_goal
        self.opp_d = opp_d = np.linalg.norm(vec_opp) # distance to center of opponent's goal
        self.vec_own = vec_own = ball_2d - self.own_goal
        self.own_d = own_d = np.linalg.norm(vec_own) # distance to center of own goal
        self.vec_opp_unit = vec_opp_unit = vec_opp / (opp_d + 1e-6)
        self.vec_own_unit = vec_own_unit = vec_own / (own_d + 1e-6)

        #========================================== front support

        LF, RF = self._get_pair_from_opp((1, opp_d-4 ,19), 2) # support center: at 3m from ball, 1m<center<19m from opp_goal, 2m side dist

        #========================================== back support

        side_d = 1

        if own_d<7: # defense 
            LS, RS = self._get_pair_from_own((8, own_d-1.3, inf), side_d, 
                                old_pair=(np.array([-7, side_d]), np.array([-7,-side_d]), 0.7*(1-own_d/7)))
        else:
            LS, RS = self._get_pair_from_own((8, own_d-1.3, inf), side_d)
            if opp_d<5: # attack
                LS, RS = self._get_pair_from_opp((-inf, opp_d+1.4, inf), 0.5, old_pair=(LS,RS,opp_d/4))


        #========================================== central back

        CB_dist = 0.6  # 0-our goal, 1-the ball
        CB = vec_own * CB_dist + self.own_goal
        CB[0] = max(-13,CB[0])

        #========================================== GK

        GOAL_HALF_WIDTH = 0.9 # useful width
        AB = np.linalg.norm(ball_2d-(-15,GOAL_HALF_WIDTH))
        BC = np.linalg.norm(ball_2d-(-15,-GOAL_HALF_WIDTH))+1e-8
        x = (GOAL_HALF_WIDTH+GOAL_HALF_WIDTH) / (AB/BC + 1)
        endline_p = np.array([-15,x-GOAL_HALF_WIDTH])
        vec = ball_2d - endline_p
        cp = endline_p - (-15-GOAL_HALF_WIDTH,0)

        # find intersection of vec with convenient circle
        a = vec[0]*vec[0] + vec[1]*vec[1]
        b = 2*( vec[0] * cp[0] + vec[1] * cp[1] )
        c = cp[0]*cp[0] + cp[1]*cp[1] - 2*GOAL_HALF_WIDTH*GOAL_HALF_WIDTH

        k = (-b + math.sqrt(b*b - 4*a*c))/(2*a)

        GK = endline_p + k*vec
        GK[0] = max(-14.9, GK[0])

        #========================================== Play mode tactics

        if w.play_mode == w.M_OUR_CORNER_KICK:
            LF = np.array([13.2,1],float)
            RF = np.array([13.2,-1],float)
            LS = np.array([12.5,0],float)
            RS = np.array([12,0],float)
        elif w.play_mode == w.M_THEIR_CORNER_KICK:
            LS = np.array([-14,1],float)
            RS = np.array([-14,-1],float)
            GK = np.array([-15,0],float)
            CB = np.array([-14.8, 3.3 if ball_2d[1] > 0 else -3.3],float)
        elif w.play_mode == w.M_THEIR_GOAL_KICK:
            LF = np.array([11.2,5],float)
            RF = np.array([11.2,-5],float)

        return [GK, CB, LS, RS, LF, RF]
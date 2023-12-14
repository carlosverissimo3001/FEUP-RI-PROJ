import numpy as np


class Head():
    def __init__(self, world) -> None:
        self.world = world
        self.look_left = True
        self.target = 0
        self.ball_d = 0

    def execute(self):

        if self.world.time_local_ms - self.world.ball_last_seen < 60: # ball was visible in last visual step
            if self.world.ball_is_visible:
                # if ball is visible, use it as target
                self.ball_d, ball_h, ball_v = self.world.ball_rel_head_sph_pos # ball distance, horizontal angle, vertical angle

                predicted_delta = np.clip(self.world.robot.joints_target_last_speed[0] * 1.1459156,-7.03,7.03) # saturate predicted horizontal movement
                self.target = self.world.robot.joints_position[0] + predicted_delta + ball_h

            if self.ball_d > 1:
                targ = self.target + (1 if self.look_left else -1) * 40
                if self.world.robot.set_joints_target_position_direct([0,1], np.array([targ ,-40]), False, max_speed=4) <= 0:
                    self.look_left = not self.look_left
            else:
                self.world.robot.set_joints_target_position_direct([0,1], np.array([self.target,-40]), False, max_speed=4)
        else:
            # if ball is not visible, look left/right
            if self.look_left:
                remaining = self.world.robot.set_joints_target_position_direct([0,1], np.array([+119,-40]), False)
            else:
                remaining = self.world.robot.set_joints_target_position_direct([0,1], np.array([-119,-40]), False)
            
            if remaining <= 0:
                self.look_left = not self.look_left

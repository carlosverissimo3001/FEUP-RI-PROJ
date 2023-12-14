import numpy as np
from behaviors.custom.Walk_ZMP.Omni_Walk import Omni_Walk


class Walk_ZMP():

    def __init__(self, world) -> None:
        self.world = world
        self.ik = self.world.robot.inv_kinematics
        self.description = "Omnidirectional analytical walk"
        self.auto_head = True
        self.walk = Omni_Walk(self.world.time_local_ms/1000.0)

    def execute(self,reset, x,y,rot):

        if reset:
            self.walk = Omni_Walk(self.world.time_local_ms/1000.0)

        rf, lf = self.walk.execute(self.world.time_local_ms/1000.0)
          
        self.walk.NewStepX_raw = x
        self.walk.NewStepY_raw = y
        self.walk.NewStepTheta_raw = rot
        self.walk.restrict_raw_walk_command()


        indices, values, error_codes = self.ik.leg(lf[0:3], [0,0,lf[3]], True, True)
        # if -1 in error_codes: 
        #     print("Position is out of reach!")
        #     error_codes.remove(-1)
        # for i in error_codes:
        #     print(f"Joint {i} is out of range!")

        self.world.robot.set_joints_target_position_direct(indices,values)


        indices, values, error_codes = self.ik.leg(rf[0:3], [0,0,rf[3]], False, True)
        # if -1 in error_codes: 
        #     print("Position is out of reach!")
        #     error_codes.remove(-1)
        # for i in error_codes:
        #     print(f"Joint {i} is out of range!")

        self.world.robot.set_joints_target_position_direct(indices,values)

        # -----------------arm movement
        indices = [14,16,18,20]
        values  = np.array([lf[4],20,90,0])
        self.world.robot.set_joints_target_position_direct(indices,values)

        indices = [15,17,19,21]
        values  = np.array([rf[4],20,90,0])
        self.world.robot.set_joints_target_position_direct(indices,values)

        return False
        

        

    def is_ready(self):
        ''' Returns True if Walk Behavior is ready to start under current game/robot conditions '''
        return True



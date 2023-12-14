from behaviors.custom.Walk_ZMP.GenerateWalkingTrajectories import GenrateWalkingTrajectories

import math
import numpy as np

class Omni_Walk():

    def __init__(self,time)->None:
        self.twalk0 = time

        self.FirstStepIsRight = 0 
        self.Zleg = -0.18
        self.NewCommand = 0
        self.NewStepX = 0
        self.NewStepY = 0
        self.NewStepTheta = 0
        self.StepTime = 0.3

        self.NewStepX_raw = 0
        self.NewStepY_raw = 0
        self.NewStepTheta_raw = 0
        

        self.amp_arm = 10
        self.arm_offset = -80
        
        self.NewStepTime = self.StepTime
        self.StopForOneStep = 0
        self.SamplingTime = 0.005
         #Generate walking trajectory
        self.StepX= 0.0
        self.StepY= 0.0
        self.StepTheta = 0.0
        self.SwingStepZ = 0.03
        self.DistanceBetweenFeet = 0.12

        self.walk_init = False

        if (self.StepY<0):
            if (self.FirstStepIsRight==0): # robot should stop
                self.StopForOneStep = 1
            self.FirstStepIsRight = 1
        elif  (self.StepY>0 or self.StepTheta>0):
            if (self.FirstStepIsRight==1): # robot should stop
                self.StopForOneStep = 1
            self.FirstStepIsRight = 0
        elif  (self.StepY>0 or self.StepTheta<0):
            if (self.FirstStepIsRight==0): # robot should stop
                self.StopForOneStep =1
            self.FirstStepIsRight = 1
        

        if (self.StopForOneStep):
            self.StepTheta = 0             
            #self.StepX = 0             
            self.StepY = 0             
            self.StepTime = 0.2  
        else:
            self.StepTime = 0.3 #I added this because otherwise, it would get stuck on 0.2, right mohammad?
                

        if (self.StopForOneStep):
            if (not self.FirstStepIsRight):
                self.FR0X = -self.StepX/2
                self.FR0Y = -self.DistanceBetweenFeet/2 
                self.FL0X = 0
                self.FL0Y = self.DistanceBetweenFeet/2  
            else:
                self.FR0X = 0
                self.FR0Y = -self.DistanceBetweenFeet/2   
                self.FL0X = -self.StepX/2    
                self.FL0Y = self.DistanceBetweenFeet/2  
        else:
            if (self.FirstStepIsRight):
                self.FR0X = -self.StepX/2
                self.FR0Y = -self.DistanceBetweenFeet/2 
                self.FL0X = 0
                self.FL0Y = self.DistanceBetweenFeet/2  
            else:
                self.FR0X = 0
                self.FR0Y = -self.DistanceBetweenFeet/2   
                self.FL0X = -self.StepX/2    
                self.FL0Y = self.DistanceBetweenFeet/2               

        self.Walking = GenrateWalkingTrajectories(self.FR0X,self.FR0Y,self.FL0X,self.FL0Y,self.StepX,self.StepY,
            self.SwingStepZ,self.StepTime,self.SamplingTime,self.FirstStepIsRight,self.Zleg,0)
        self.Walking.Generate()  
        self.idx = -1


    def apply_walk_command(self):
    
        self.NewStepX +=     max( min( self.NewStepX_raw - self.NewStepX, 0.03) , -0.03)
        self.NewStepY +=     max( min( self.NewStepY_raw - self.NewStepY, 0.02) , -0.02)
        self.NewStepTheta += max( min( self.NewStepTheta_raw - self.NewStepTheta, 10) , -10)

        self.NewCommand = 1

        #print("Apply:", self.NewStepX, self.NewStepY, self.NewStepTheta)

    def translate(self,value, IN, OUT):
        """
        Maps from range IN=[low,high] to OUT=[low,high] without saturating output
        e.g. map x=0 from [-1,1] to [2,3] returns 2.5
        e.g. map x=3 from [ 0,1] to [0,2] returns 6

        :param IN: input range (list[2])
        :param OUT: output range (list[2])
        :return: mapped value
        """
        # Figure out how 'wide' each range is
        InSpan = IN[1] - IN[0]
        OutSpan = OUT[1] - OUT[0]

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - IN[0]) / float(InSpan)

        # Convert the 0-1 range into a value in the right range.
        return OUT[0] + (valueScaled * OutSpan)



    def restrict_raw_walk_command(self, isNormalized=False):

        stepx_lim = [-0.2,0.2]
        stepy_lim = [-0.1, 0.1]
        steptheta_lim = [-50, 50]

        #Normalize if not already
        if not isNormalized:
            x = self.translate( self.NewStepX_raw, stepx_lim, [-0.5,0.5] ) #map to relative values [-0.5, 0.5]
            y = self.translate( self.NewStepY_raw, stepy_lim, [-0.5,0.5] ) #map to relative values [-0.5, 0.5]
            t = self.translate( self.NewStepTheta_raw, steptheta_lim, [-0.5,0.5] ) #map to relative values [-0.5, 0.5]
        else:
            x = self.NewStepX_raw
            y = self.NewStepY_raw
            t = self.NewStepTheta_raw

        #avoid transition stop
        if abs(y) < 0.03: 
            y = 0
        if abs(t) < 0.1: 
            t = 0

        #compensate rotation
        #t += y*1.17

        #limit overall maximum (only affects the command when the limit is actually exceeded)
        manhatDist = abs(x) + abs(y) + abs(t)

        if manhatDist > 0.5:
            factor = 1.0 #1 means maximum performance (<1 means safe parameters)
            norm = manhatDist / (0.5*factor)

            x /= norm
            y /= norm
            t /= norm

        #denormalize
        self.NewStepX_raw = np.interp( x, [-0.5,0.5], stepx_lim ) #map to absolute commands
        self.NewStepY_raw = np.interp( y, [-0.5,0.5], stepy_lim ) #map to absolute commands
        self.NewStepTheta_raw = np.interp( t, [-0.5,0.5], steptheta_lim ) #map to absolute commands
        # self.apply_walk_command()


     
        

    def execute(self,time):

        # Initialize pose before walking (m.abreu: I disabled this preparation time because it is not needed)
        # if not self.walk_init:
        #     self.walk_init = time - self.twalk0 > 0.1
        #     self.RfootPosition = [0,-self.DistanceBetweenFeet/2,self.Zleg,0,self.arm_offset]
        #     self.LfootPosition = [0, self.DistanceBetweenFeet/2,self.Zleg,0,self.arm_offset]
        #     return self.RfootPosition,self.LfootPosition
    


        if ( abs(time - self.twalk0) > ((2-self.StopForOneStep)*self.StepTime)-(self.SamplingTime/2)):
            self.twalk0 = time
            self.StopForOneStep = 0

            # self.NewStepX     = 0.0
            # self.NewStepY     = -0.04
            # self.NewStepTheta = 0.0
            # self.NewCommand = 1
            
            self.apply_walk_command()

            if (self.NewCommand): 
                self.StepTheta = self.NewStepTheta
                self.StepX = self.NewStepX*math.cos(math.radians(self.StepTheta))-self.NewStepY*math.sin(math.radians(self.StepTheta))
                self.StepY = self.NewStepX*math.sin(math.radians(self.StepTheta))+self.NewStepY*math.cos(math.radians(self.StepTheta))
                self.StepTime = self.NewStepTime
                self.NewCommand = 0


            if (self.StepY < 0):
                if (self.FirstStepIsRight==0): # robot should stop
                    self.StopForOneStep = 1
                self.FirstStepIsRight = 1

            elif  (self.StepY >= 0 and self.StepTheta < 0):
                if (self.FirstStepIsRight==0): # robot should stop
                    self.StopForOneStep =1
                self.FirstStepIsRight = 1
            
            elif  (self.StepY >= 0 and self.StepTheta >= 0):
                if (self.FirstStepIsRight==1): # robot should stop
                    self.StopForOneStep = 1
                self.FirstStepIsRight = 0
            

            if (self.StopForOneStep):
                self.StepTheta = 0             
                #self.StepX = 0             
                self.StepY = 0

                self.StepTime = 0.03#0.05 
            else:
                self.StepTime = 0.3 
                    

            if (self.StopForOneStep):
                if (not self.FirstStepIsRight):
                    self.FR0X = -self.StepX/2
                    self.FR0Y = -self.DistanceBetweenFeet/2 
                    self.FL0X = 0
                    self.FL0Y = self.DistanceBetweenFeet/2  
                else:
                    self.FR0X = 0
                    self.FR0Y = -self.DistanceBetweenFeet/2   
                    self.FL0X = -self.StepX/2    
                    self.FL0Y = self.DistanceBetweenFeet/2  
            else:
                if (self.FirstStepIsRight):
                    self.FR0X = -self.StepX/2
                    self.FR0Y = -self.DistanceBetweenFeet/2 
                    self.FL0X = 0
                    self.FL0Y = self.DistanceBetweenFeet/2  
                else:
                    self.FR0X = 0
                    self.FR0Y = -self.DistanceBetweenFeet/2   
                    self.FL0X = -self.StepX/2    
                    self.FL0Y = self.DistanceBetweenFeet/2               

                    
            self.Walking = GenrateWalkingTrajectories(self.FR0X,self.FR0Y,self.FL0X,self.FL0Y,self.StepX,self.StepY,
                self.SwingStepZ,self.StepTime,self.SamplingTime,self.FirstStepIsRight,self.Zleg,0)
            self.Walking.Generate()  
            self.idx = -1

            # fig = plt.figure()
            # plt.plot(self.Walking.RightLegX)   
            # plt.grid()
            # plt.plot(self.Walking.RightLegY) 
            # plt.plot(self.Walking.RightLegZ) 
            # plt.plot(self.Walking.LeftLegX) 

            # plt.plot(self.Walking.LeftLegY)   
            # plt.plot(self.Walking.LeftLegZ) 
            # # #--------------------------------------
            # plt.plot(self.Walking.FT.LeftY)
            # plt.plot(self.Walking.FT.RightY)
            # plt.show()

            return self.RfootPosition,self.LfootPosition


        else:

            twalk = time - self.twalk0
            self.idx = int(round(twalk/self.SamplingTime))
            
          

            if (self.StopForOneStep == 1):
                self.RfootPosition = [0,-self.DistanceBetweenFeet/2,self.Zleg,0,self.arm_offset]
                self.LfootPosition = [0,self.DistanceBetweenFeet/2,self.Zleg,0,self.arm_offset]
                return self.RfootPosition,self.LfootPosition

              
            elif (self.idx>=0 and self.idx<len(self.Walking.LeftLegX)):
                amp_arm = self.amp_arm * (np.clip(abs(self.StepX)/0.05,1,2.5)) * ( 1 if self.StepX>= 0 else -1)

                if (self.FirstStepIsRight):
                    self.LShX = self.arm_offset - amp_arm*math.sin(twalk*2*math.pi/(2*self.StepTime)) #- EN_ArmX * 2 * PDArmX
                    self.RShX = self.arm_offset  + amp_arm*math.sin(twalk*2*math.pi/(2*self.StepTime)) #- EN_ArmX * 2 * PDArmX
                    self.Rtheta = self.StepTheta * (np.clip( (twalk+0.02-self.StepTime) / self.StepTime,0,1))
                    self.Ltheta = self.StepTheta * (np.clip( (twalk+0.02) / (self.StepTime),0,1))
                    
                else:
                    self.LShX = self.arm_offset + amp_arm*math.sin(twalk*2*math.pi/(2*self.StepTime)) #- EN_ArmX * 2 * PDArmX
                    self.RShX = self.arm_offset - amp_arm*math.sin(twalk*2*math.pi/(2*self.StepTime)) #- EN_ArmX * 2 * PDArmX
                    self.Rtheta = self.StepTheta * (np.clip( (twalk+0.02) / (self.StepTime),0,1))
                    self.Ltheta = self.StepTheta * (np.clip( (twalk+0.02-self.StepTime) / self.StepTime,0,1))
                    # print (f"t_w {twalk:.2f}\t t_r {np.clip((twalk+0.02) / self.StepTime,0,1):.2f}\t t_l {np.clip((twalk+0.02-self.StepTime)/(self.StepTime),0,1):.2f}")
                    

                if (self.StepTheta<0):
                    self.Rtheta =  self.Rtheta
                    self.Ltheta = -self.Ltheta
                else:
                    self.Rtheta = -self.Rtheta
                    self.Ltheta = self.Ltheta
              

                dY = 0#-0.0025*math.sin(twalk*2*math.pi/(2*self.StepTime)) 
                self.RfootPosition = [self.Walking.RightLegX[self.idx], self.Walking.RightLegY[self.idx]+dY,self.Walking.RightLegZ[self.idx],self.Rtheta,self.RShX]
                self.LfootPosition = [self.Walking.LeftLegX[self.idx],self.Walking.LeftLegY[self.idx]-dY,self.Walking.LeftLegZ[self.idx],self.Ltheta,self.LShX]           

                # print (f"R {self.RfootPosition[0]:+.3f} \t {self.RfootPosition[1]:+.3f} \t {self.RfootPosition[2]:+.3f} \t {self.RfootPosition[3]:+.3f}")
                # print (f"L {self.LfootPosition[0]:+.3f} \t {self.LfootPosition[1]:+.3f} \t {self.LfootPosition[2]:+.3f} \t {self.LfootPosition[3]:+.3f}")
                
                return self.RfootPosition,self.LfootPosition
 

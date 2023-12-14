"""
Created on Mon Jun 22 14:51:52 2020

@author: mohammad
"""

import numpy
import math
import statistics


class FeetGenerator:
    RightX  = []
    RightY  = []
    RightZ  = []

    LeftX  = []
    LeftY  = []
    LeftZ  = []
    
    def __init__(self):
        self.RightX  = []
        self.RightY  = []
        self.RightZ  = []
    
        self.LeftX  = []
        self.LeftY  = []
        self.LeftZ  = []

        
    def Generate(self,StepX,StepY,SwingStepZ,StepTime,SamplingTime,NumberOfStep,zmp_x,zmp_y,FR0X,FR0Y,FL0X,FL0Y,FirstStepIsRight,SmoothHeightIndex):

            t = 0
            index =0
            swing=[]
            RightIsMoving = FirstStepIsRight
            LeftLeg = []
            RightLeg = []
            
            TFinalSimulation = (NumberOfStep)*StepTime    
            for GlobalTime in numpy.arange(0,TFinalSimulation,SamplingTime):
                if (GlobalTime==0 or t > StepTime-(SamplingTime) ):           
                    if (index==0):
                        RightIsMoving = FirstStepIsRight
                        if (FirstStepIsRight):
                            Xp = FR0X 
                            Yp = FR0Y
                        else:
                            Xp = FL0X 
                            Yp = FL0Y
                        
                    else:
                        if (RightIsMoving):
                            RightIsMoving = 0
                        else:
                            RightIsMoving = 1
                        
                        Xp = ZMPX
                        Yp = ZMPY
                    
                    l = int(index*round(StepTime/SamplingTime))
                    ZMPX = statistics.mean(zmp_x[ l:l+int(round(0.1*StepTime/SamplingTime))])
                    ZMPY = statistics.mean(zmp_y[l:l+int(round(0.1*StepTime/SamplingTime))])
                    
                    index=index+1
                    t = 0    
                else:
                    t = t + SamplingTime
                
 
                ZMPZ = 0
                t0 = 0.
                if (SmoothHeightIndex>1 and index<SmoothHeightIndex):
                    if (t<t0):
                        SWINGX = Xp+(StepX*(t/(StepTime)))
                        SWINGY = Yp+(StepY*(t/StepTime))
                        SWINGZ = 0*(index/SmoothHeightIndex)*SwingStepZ*(math.sin(math.pi*t/StepTime))        
                    else:
                        SWINGX = Xp+(StepX*(t/(StepTime)))
                        SWINGY = Yp+(StepY*(t/StepTime))
                        SWINGZ = (index/SmoothHeightIndex)*SwingStepZ*(math.sin(math.pi*(t-t0)/(StepTime-t0)))        
                else:
                    if (t<t0):            
                        SWINGX = Xp+(StepX*(t/(StepTime)))
                        SWINGY = Yp+(StepY*(t/StepTime))
                        SWINGZ = 0        
                    else:
                         SWINGX = Xp+(StepX*(t/(StepTime)))
                         SWINGY = Yp+(StepY*(t/StepTime))
                         SWINGZ = SwingStepZ*(math.sin(math.pi*(t-t0)/(StepTime-t0)))
                    
                
                if (RightIsMoving):    
                   self.RightX.append(SWINGX)
                   self.RightY.append(SWINGY)
                   self.RightZ.append(SWINGZ)
                   
                   self.LeftX.append(ZMPX)
                   self.LeftY.append(ZMPY)
                   self.LeftZ.append(ZMPZ)
                else:
                   self.LeftX.append(SWINGX)
                   self.LeftY.append(SWINGY)
                   self.LeftZ.append(SWINGZ)
                   
                   self.RightX.append(ZMPX)
                   self.RightY.append(ZMPY)
                   self.RightZ.append(ZMPZ)
            
         
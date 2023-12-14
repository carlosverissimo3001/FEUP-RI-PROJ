
from itertools import count
from typing import List
from scripts.commons.Script import Script
from agent.Agent_Thailand import Agent
from utils.Draw import Draw

class Radio_Localization():

    def __init__(self,script:Script) -> None:
        self.script = script


    def draw_others(self,p:Agent):
        me = p.world.robot.loc_head_position
        d:Draw = self.script.players[0].world.draw # get draw object from same player to always overwrite previous drawing

        others = p.world.teammates + p.world.opponents

        for o in others:
            if o.is_self or o.state_last_update==0: 
                continue

            pos = o.state_abs_pos
            is_down = o.state_fallen
            is_3D = pos is not None and len(pos)==3

            # 0.06s is the time it takes to get a visual update
            is_current = o.state_last_update > p.world.time_local_ms - 60

            # 0.12s is the time it takes to do a full broadcast with all positions if every group is completely visible
            # here we use >= instead of > because the radio message comes with a delay of 20ms
            is_recent = o.state_last_update >= p.world.time_local_ms - 120

            if is_down:
                c = d.Color.pink
            elif is_current and is_3D:
                c = d.Color.green_light
            elif is_recent and is_3D:
                c = d.Color.green
            elif is_current:
                c = d.Color.yellow
            elif is_recent:
                c = d.Color.yellow_light
            else:
                c = d.Color.red

            d.arrow(me,pos,0.1,3,c,"others",False)

        d.flush("others")


        
        
    def execute(self):

        a = self.script.args

        #Server IP, Agent Port, Monitor Port, Uniform Number, Robot Type, Team name, Enable Log, Enable Draw
        self.script.batch_create(Agent, ((a.i,a.p,a.m,u,a.t,       True,True) for u in range(1,12))) # types: 0,0,1,1,2,2,3,3,4,4,4
        self.script.batch_create(Agent, ((a.i,a.p,a.m,u,"Opponent",True,True) for u in range(1,12)))
        players : List[Agent] = self.script.players

        #Beam players
        beam_home_team = [(-(i//2)-3,(i%2*2-1)*(i//2+1),0) for i in range(11)]
        self.script.batch_commit_beam( beam_home_team * 2 )

        #Execute
        for j in count():
            for i in range(11): 
                players[i].think_and_send()
            self.draw_others(players[j//15%11])
            self.script.batch_commit_and_send()
            self.script.batch_receive()

from scripts.commons.UI import UI
import numpy as np
from utils.Draw import Draw
from agent.commons.Base_Agent import Base_Agent as Agent
from scripts.commons.Script import Script

class Behavior():

    def __init__(self,script:Script) -> None:
        self.script = script
        self.player : Agent = None

    def ask_for_behavior(self):
        names, descriptions = self.player.world.robot.behavior.get_all_behaviors()

        UI.print_table( [names,descriptions], ["Behavior Name","Description"], numbering=[True,False])
        choice, is_str_opt = UI.read_particle('Choose behavior ("" to skip 2 time steps, "b" to beam, "q" to quit):',["","b","q"],int,[0,len(names)])
        if is_str_opt: return choice #skip 2 time steps or quit
        return names[choice]

    def sync(self):
        self.player.scom.commit_and_send( self.player.world.robot.get_command() ) 
        self.player.scom.receive()

    def execute(self):

        a = self.script.args  

        # Server IP, Agent Port, Monitor Port, Uniform Number, Robot Type, Team name, Enable Log, Enable Draw, Other Players (only used for multi-agent)
        self.player = Agent(a.i, a.p, a.m, a.u, a.r, a.t, True, True, [])
        r = self.player.world.robot

        # Beam
        self.player.scom.commit_beam((-3,0),0)
        for _ in range(3): self.sync()

        # Special behaviors with complex arguments
        special_behaviors = {"Walk":(0.1,0.0,0), "Step":(), "Kick_RL":(0,), "Walk_RL":((-15,0),(-15,0),-180), "Walk_RL3":((0,0),True,0,False,None)}

        while True:
            behavior_name = self.ask_for_behavior()
            if behavior_name == 0: # skip 2 time steps (user request)
                self.sync()
                self.sync()
            elif behavior_name == 1: # beam
                self.player.scom.commit_beam((-3,0),0)
                for _ in range(3): self.sync()    
            elif behavior_name == 2: # quit
                break
            else:
                if behavior_name in special_behaviors: # not using execute_to_completion to abort behavior after a timeout
                    for _ in range(150): 
                        if r.behavior.execute(behavior_name, *special_behaviors[behavior_name]): break
                        self.player.scom.commit_and_send( r.get_command() ) 
                        self.player.scom.receive()
                else:
                    r.behavior.execute_to_completion(behavior_name)

        self.player.scom.close()

from abc import abstractmethod
from communication.Radio import Radio
from communication.World_Parser import World_Parser
from communication.Server_Comm import Server_Comm
from world.World import World
from logs.Logger import Logger


class Base_Agent():
    other_agents = []

    def __init__(self, host:str, agent_port:int, monitor_port:int, unum:int, robot_type:int, 
                 team_name:str, enable_log, enable_draw, hear_callback=None, wait_for_server=True) -> None:

        self.radio = None # hear_message may be called during Server_Comm instantiation
        self.logger = Logger(enable_log, f"{team_name}_{unum}")
        self.world = World(robot_type, team_name, unum, enable_draw, self.logger, host)
        self.world_parser = World_Parser(self.world, self.hear_message if hear_callback is None else hear_callback)
        self.scom = Server_Comm(host,agent_port,monitor_port,unum,robot_type,team_name,self.world_parser,self.world,Base_Agent.other_agents,wait_for_server)
        self.world.robot.initialize_dependent_objects(self.world, self.scom)
        self.radio = Radio(self.world, self.scom.commit_announcement)
        Base_Agent.other_agents.append(self)

    @abstractmethod
    def think_and_send(self):
        pass    

    def hear_message(self, msg:bytearray, direction, timestamp:float) -> None:
        if direction != "self" and self.radio is not None:
            self.radio.receive(msg)

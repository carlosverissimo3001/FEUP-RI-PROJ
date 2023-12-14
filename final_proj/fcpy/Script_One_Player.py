from scripts.commons.Script import Script
script = Script(cpp_builder_unum=1) # Initialize: load config file, parse arguments, build cpp modules
a = script.args

from agent.Agent_Thailand import Agent

# Server IP, Agent Port, Monitor Port, Uniform Number, Team name, Enable Log, Enable Draw, Do not wait for server
player = Agent(a.i, a.p, a.m, a.u, a.t, True, True, False)


while True:
    player.think_and_send()
    player.scom.receive()
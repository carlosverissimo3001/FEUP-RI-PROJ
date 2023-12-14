from scripts.commons.Script import Script
script = Script(cpp_builder_unum=1, hide_UI=True) # Initialize: load config file, parse arguments, build cpp modules
a = script.args

from agent.Agent_Brazil22 import Agent

# Server IP, Agent Port, NO Monitor Port, Uniform Number, Team name, DISABLE Log, DISABLE Draw, Do not wait for server
player = Agent(a.i, a.p, None, a.u, a.t, False, False, False)


while True:
    player.think_and_send()
    player.scom.receive()
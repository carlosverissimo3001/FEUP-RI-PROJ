from scripts.commons.Script import Script
script = Script() # Initialize: load config file, parse arguments, build cpp modules
a = script.args

from agent.Agent_Thailand import Agent

# Server IP, Agent Port, Monitor Port, Uniform Number, Robot Type, Team name, Enable Log, Enable Draw, Other Players (only used for multi-agent)
script.batch_create(Agent, ((a.i, a.p, a.m, a.u, a.t,        True, True),)) #one player for home team
script.batch_create(Agent, ((a.i, a.p, a.m, a.u, "Opponent", True, True),)) #one player for away team


while True:
    script.batch_execute_agent()
    script.batch_receive()
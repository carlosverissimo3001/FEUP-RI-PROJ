#!/bin/bash
time=0
export OMP_NUM_THREADS=1

python3 Script_Fat_Proxy_Player.py -u 1  &
sleep $time 
python3 Script_Fat_Proxy_Player.py -u 2  &
sleep $time 
python3 Script_Fat_Proxy_Player.py -u 3  &
sleep $time 
python3 Script_Fat_Proxy_Player.py -u 4  &
sleep $time 
python3 Script_Fat_Proxy_Player.py -u 5  &
sleep $time 
python3 Script_Fat_Proxy_Player.py -u 6  &
sleep $time 
python3 Script_Fat_Proxy_Player.py -u 7  &
sleep $time 
python3 Script_Fat_Proxy_Player.py -u 8  &
sleep $time 
python3 Script_Fat_Proxy_Player.py -u 9  &
sleep $time 
python3 Script_Fat_Proxy_Player.py -u 10 &
sleep $time 
python3 Script_Fat_Proxy_Player.py -u 11 &
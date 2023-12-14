#!/bin/bash
export OMP_NUM_THREADS=1

for i in {1..11}; do
python3 ./Script_Official_Player.py -u $i -t "FCPortugal" &
done

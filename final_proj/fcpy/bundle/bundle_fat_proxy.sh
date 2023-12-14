#!/bin/bash

#Call this script from any directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

#cd to main folder
cd "${SCRIPT_DIR}/.."

rm -rf ./bundle/build
rm -rf ./bundle/dist

onefile="--onefile"

#bundle app, dependencies and data files into single executable
pyinstaller \
--add-data './robots:robots' \
--add-data './behaviors/slot/common:behaviors/slot/common' \
--add-data './behaviors/slot/r0:behaviors/slot/r0' \
--add-data './behaviors/slot/r1:behaviors/slot/r1' \
--add-data './behaviors/slot/r2:behaviors/slot/r2' \
--add-data './behaviors/slot/r3:behaviors/slot/r3' \
--add-data './behaviors/slot/r4:behaviors/slot/r4' \
--add-data './behaviors/custom/Dribble_RL/*.pkl:behaviors/custom/Dribble_RL' \
--add-data './behaviors/custom/Walk_RL/*.pkl:behaviors/custom/Walk_RL' \
--add-data './behaviors/custom/Walk_RL3/*.pkl:behaviors/custom/Walk_RL3' \
--add-data './behaviors/custom/Kick_RL/*.pkl:behaviors/custom/Kick_RL' \
--add-data './behaviors/custom/Kick_Short_RL/*.pkl:behaviors/custom/Kick_Short_RL' \
--add-data './behaviors/custom/Kick_Long_RL/*.pkl:behaviors/custom/Kick_Long_RL' \
--add-data './behaviors/custom/Push_RL/*.pkl:behaviors/custom/Push_RL' \
${onefile} --distpath ./bundle/dist/ --workpath ./bundle/build/ --noconfirm --name fcp Script_Fat_Proxy_Player.py

cat > ./bundle/dist/start.sh << EOF
#!/bin/bash
export OMP_NUM_THREADS=1

port=\${1:-3110}

for i in {1..11}; do
./fcp -u \$i -t FCPortugal -p \$port &
done
EOF

chmod +x ./bundle/dist/start.sh
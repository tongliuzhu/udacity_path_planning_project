#!/bin/bash
# Script to build all components from scratch, using the maximum available CPU power

# Go into the directory where this bash script is contained.
cd `dirname $0`

# Compile code.
mkdir -p build
cd build
cmake ..
make

cd ..

#if [ $(which gnome-terminal) ]; then
#    TERMINAL=gnome-terminal
#fi

if [ $(which xfce4-terminal) ]; then
    TERMINAL=xfce4-terminal
fi

#roscore &
# here you don`t sleep for a few seconds, other roslaunch or rosrun will failed sometimes
#sleep 3

${TERMINAL} --title "simulator" -x bash -c "./run_simulator.sh;" --maximize &
sleep 1
${TERMINAL} --tab --title "planning_algorithm" -x bash -c "./run_algorithm.sh;"&


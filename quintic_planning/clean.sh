#!/bin/bash

function kill_process()
{
    proc_name=$1
    echo 'killing '$proc_name
    find_result=$(ps -ef | grep $proc_name | awk '{print $2}')
    if [[ ${find_result} != 0 ]];then
        kill -2 ${find_result} # kill with ctrl^c
        kill -9 ${find_result} # force kill
    fi
}


kill_process path_planning
kill_process term3_sim


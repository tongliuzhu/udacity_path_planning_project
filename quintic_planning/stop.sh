#!/bin/bash

function stop_process()
{
    proc_name=$1
    echo 'killing '$proc_name
    find_result=$(ps -ef | grep $proc_name | awk '{print $2}')
    if [[ ${find_result} != 0 ]];then
        kill -STOP ${find_result} # stop the process
    fi
}


stop_process path_planning
stop_process term3_sim


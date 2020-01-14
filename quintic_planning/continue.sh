#!/bin/bash

function continue_process()
{
    proc_name=$1
    echo 'killing '$proc_name
    find_result=$(ps -ef | grep $proc_name | awk '{print $2}')
    if [[ ${find_result} != 0 ]];then
        kill -CONT ${find_result} # stop the process
    fi
}


continue_process path_planning
continue_process term3_sim


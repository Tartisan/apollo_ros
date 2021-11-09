#! /bin/bash

function kill_proc() {  
    proc_name=$1
    proc_num=`ps -ef |grep -w ${proc_name}|grep -v grep|wc -l`
    if [ ${proc_num} -gt 0 ];then
        pid=$(pgrep -f ${proc_name})
        sudo kill $pid
    fi
}

kill_proc "devel/lib"
echo "stop process done"

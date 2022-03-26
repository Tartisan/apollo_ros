#! /usr/bin/env bash
APOLLO_ROOT_DIR="/apollo_ros"

if [ -f /${APOLLO_ROOT_DIR}/devel/setup.bash ]; then
    source /${APOLLO_ROOT_DIR}/devel/setup.bash
fi
alias sr='source devel/setup.bash'
PS1='${debian_chroot:+($debian_chroot)}\u@in_dev_docker:\w\$ '

export GLOG_log_dir="${APOLLO_ROOT_DIR}/data/log"
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0
# for DEBUG log
# export GLOG_v=4

export sysmo_start=0
echo ${GLOG_log_dir}
export GLOG_log_dir="${APOLLO_ROOT_DIR}/data/log"
echo ${GLOG_log_dir}
PS1='${debian_chroot:+($debian_chroot)}\u@in_dev_docker:\w\$ '

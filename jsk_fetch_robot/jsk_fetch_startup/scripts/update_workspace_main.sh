#!/usr/bin/env bash

function usage()
{
    echo "Usage: $0 [-w workspace_directory] [-h] [-l]

optional arguments:
    -h                  show this help
    -w WORKSPACE_PATH   specify target workspace
    -l                  do not send a mail
"
}

function get_full_path()
{
    echo "$(cd $(dirname $1) && pwd)/$(basename $1)"
}

SEND_MAIL=true
WORKSPACE=$(get_full_path $HOME/ros/melodic)

while getopts hlw: OPT
do
    case $OPT in
        w)
            WORKSPACE=$(get_full_path $OPTARG)
            ;;
        l)
            SEND_MAIL=false
            ;;
        h)
            usage
            exit 1
            ;;
        *)
            usage
            exit 1
            ;;
    esac
done

if [ ! -e $WORKSPACE ]; then
    echo "No workspace $WORKSPACE"
    exit 1
fi

. $WORKSPACE/devel/setup.bash
# Filename should end with .txt to preview the contents in a web browser
LOGFILE=$WORKSPACE/update_workspace.txt

TMP_MAIL_BODY_FILE=/tmp/update_workspace_mailbody.txt

{
set -x
# Update workspace

wstool foreach -t $WORKSPACE/src --git 'git stash'
wstool foreach -t $WORKSPACE/src --git 'git fetch origin --prune'
wstool update -t $WORKSPACE/src jsk-ros-pkg/jsk_robot
ln -sf $(rospack find jsk_fetch_startup)/../jsk_fetch.rosinstall.$ROS_DISTRO $WORKSPACE/src/.rosinstall
wstool update -t $WORKSPACE/src --delete-changed-uris
# Forcefully checkout specified branch
wstool foreach -t $WORKSPACE/src --git --shell 'branchname=$(git rev-parse --abbrev-ref HEAD); if [ $branchname != "HEAD" ]; then git reset --hard HEAD; git checkout origin/$branchname; git branch -D $branchname; git checkout -b $branchname --track origin/$branchname; fi'
wstool update -t $WORKSPACE/src
WSTOOL_UPDATE_RESULT=$?
# Rosdep Install
sudo apt-get update -y
rosdep update
# jsk_footstep_planner is not released for melodic
# jsk_footstep_controller is not released for melodic
# librealsense2 should not be installed from ROS repository
# realsense-ros should not be installed from ROS repository
rosdep install --from-paths $WORKSPACE/src --ignore-src -y -r --skip-keys \
"\
jsk_footstep_controller \
jsk_footstep_planner \
librealsense2 \
realsense2_camera \
realsense2_description \
"
ROSDEP_INSTALL_RESULT=$?
# Build workspace
cd $WORKSPACE
catkin clean aques_talk collada_urdf_jsk_patch libcmt -y
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build --continue-on-failure
CATKIN_BUILD_RESULT=$?
# Send mail
echo "" > $TMP_MAIL_BODY_FILE
if [ $WSTOOL_UPDATE_RESULT -ne 0 ]; then
    echo "Please wstool update workspace manually.\n" >> $TMP_MAIL_BODY_FILE
fi
if [ $ROSDEP_INSTALL_RESULT -ne 0 ]; then
    echo "Please install dependencies manually.\n" >> $TMP_MAIL_BODY_FILE
fi
if [ $CATKIN_BUILD_RESULT -ne 0 ]; then
    echo "Please catkin build workspace manually.\n" >> $TMP_MAIL_BODY_FILE
fi
set +x
} 2>&1 | tee $LOGFILE

# MAIL_BODY variable cannot be set directly in a subshell. So it is set from temporary mail body text file.
# The mail body text is put as $TMP_MAIL_BODY_FILE.
# See https://github.com/jsk-ros-pkg/jsk_robot/issues/1569
MAIL_BODY=$(cat $TMP_MAIL_BODY_FILE)
echo "MAIL_BODY: $MAIL_BODY"

if [ -n "$MAIL_BODY" ] && [ "${SEND_MAIL}" == "true" ]; then
    echo "Sent a mail"
    rostopic pub -1 /email jsk_robot_startup/Email "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
subject: 'Daily workspace update fails'
body:
- {type: 'text', message: '${MAIL_BODY}', file_path: '', img_data: '', img_size: 0}
sender_address: '$(hostname)@jsk.imi.i.u-tokyo.ac.jp'
receiver_address: 'fetch@jsk.imi.i.u-tokyo.ac.jp'
smtp_server: ''
smtp_port: ''
attached_files: ['$LOGFILE']"
fi
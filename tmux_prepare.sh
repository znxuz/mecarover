#!/usr/bin/env bash

tmux neww
# tmux splitw -b -h "docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888"
tmux splitw -b -h "cd $HOME/code/microros_ws && source /opt/ros/jazzy-base/setup.zsh && source install/local_setup.zsh && ROS_DOMAIN_ID=42 ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
tmux splitw -b "picocom -b 460800 /dev/ttyACM0 --imap lfcrlf"
tmux selectp -R

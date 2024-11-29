#!/usr/bin/env zsh

tmux neww
tmux splitw -b -h "source $ZDOTDIR/.zshrc && ross-ws && ROS_DOMAIN_ID=42 ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
tmux splitw -b "picocom -b 1152000 /dev/ttyACM0 --imap lfcrlf"
tmux selectp -R

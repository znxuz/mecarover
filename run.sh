#!/usr/bin/env bash

topleft_pane()
{
	echo "tmux splitw -b 'picocom -b 115200 /dev/ttyACM0 --imap lfcrlf'"
}

bottom_left_pane()
{
	echo "tmux splitw -h -b 'docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888'"
}

current_pane()
{
	echo "docker run -it --rm --net=host -e ROS_DOMAIN_ID=56 ros2-jazzy-mine"
}

tmux bind-key C-l send-keys -R \; clear-history
tmux neww "$(bottom_left_pane); $(topleft_pane); tmux selectp -R; $(current_pane)"

#!/usr/bin/env bash

topleft_pane()
{
	echo "tmux splitw -b 'docker run -it --rm --net=host microros/micro-ros-agent:foxy udp4 --port 8888'"
}

bottom_left_pane()
{
	echo "tmux splitw -h -b 'picocom -b 115200 /dev/ttyACM0'"
}

current_pane()
{
	echo "docker run -it --rm --net=host -e ROS_DOMAIN_ID=56 ros2-modified:latest"
}

tmux neww "$(bottom_left_pane); $(topleft_pane); tmux selectp -R; $(current_pane)"

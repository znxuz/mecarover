#!/usr/bin/env bash

tmux neww
tmux splitw -b -h "docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888"
tmux splitw -b "picocom -b 115200 /dev/ttyACM0 --imap lfcrlf"
tmux selectp -R

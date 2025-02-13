#!/usr/bin/env bash

tmux neww 'distrobox enter ros'
tmux splitw -b -h 'distrobox enter ros'
tmux splitw -b "picocom -b 1152000 /dev/ttyACM0 --imap lfcrlf" && tmux resizep -D 16
tmux selectp -R && tmux resizep -R 68

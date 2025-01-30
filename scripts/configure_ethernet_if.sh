#!/usr/bin/env bash

IP="192.168.188.200/24"
ETHERNET_IF="enp2s0f0"

sudo ip a add $IP dev $ETHERNET_IF && sudo ip link set $ETHERNET_IF up

#pragma once
#ifndef ROS_CONFIG_H
#define ROS_CONFIG_H

constexpr char base_link[] = "/base_link";
constexpr char odom[] = "/odom";

//constexpr char server[] = "172.22.166.125";  // IP address of rosserial-server node
constexpr char server[] = "192.168.137.1"; // IP address of rosserial-server node
//constexpr char server[] = "192.168.178.11";  // IP address of rosserial-server node
constexpr uint16_t serverPort = 11411; // rosserial socket server port

#endif

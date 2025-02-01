#!/bin/bash
ros2 topic pub /can_node/c620_0/target_current std_msgs/msg/Float64 "data: 0.0" -1
ros2 topic pub /can_node/c620_1/target_current std_msgs/msg/Float64 "data: 0.0" -1
ros2 topic pub /can_node/gm6020_0/target_volt std_msgs/msg/Int64 "data: 0" -1
ros2 topic pub /can_node/gm6020_1/target_volt std_msgs/msg/Int64 "data: 0" -1
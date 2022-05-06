#!/bin/bash
rostopic pub /smallbot/front_left_wheel/target_velocity std_msgs/Float64 -1 -- $1
rostopic pub /smallbot/front_right_wheel/target_velocity std_msgs/Float64 -1 -- $1
rostopic pub /smallbot/middle_left_wheel/target_velocity std_msgs/Float64 -1 -- $1
rostopic pub /smallbot/middle_right_wheel/target_velocity std_msgs/Float64 -1 -- $1
rostopic pub /smallbot/rear_left_wheel/target_velocity std_msgs/Float64 -1 -- $1
rostopic pub /smallbot/rear_right_wheel/target_velocity std_msgs/Float64 -1 -- $1

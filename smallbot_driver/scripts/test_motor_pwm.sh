#!/bin/bash
rostopic pub /smallbot/front_left_wheel/pwm std_msgs/Float64 0.25 -1
rostopic pub /smallbot/front_right_wheel/pwm std_msgs/Float64 0.25 -1
rostopic pub /smallbot/middle_left_wheel/pwm std_msgs/Float64 0.25 -1
rostopic pub /smallbot/middle_right_wheel/pwm std_msgs/Float64 0.25 -1
rostopic pub /smallbot/rear_left_wheel/pwm std_msgs/Float64 0.25 -1
rostopic pub /smallbot/rear_right_wheel/pwm std_msgs/Float64 0.25 -1

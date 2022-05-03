
#include "dc_motors.hpp"
#include "dc_motor.hpp"
#include <std_msgs/Float64.h>

DCMOTOR *frontMotors = new DCMOTOR(0, 0x40, true, false);
DCMOTOR *middleMotors = new DCMOTOR(0, 0x42, true, false);
DCMOTOR *rearMotors = new DCMOTOR(0, 0x41, true, false);

void frontLeftMotorCallback(const std_msgs::Float64& msg) {
    int16_t pwm = msg.data * 100;
    if (pwm > 0) {
        frontMotors->motorRun(0, MOTOR_DIRECTION_FORWARD, pwm);
    } else if (pwm < 0) {
        frontMotors->motorRun(0, MOTOR_DIRECTION_BACKWARD, abs(pwm));
    } else if (pwm == 0) {
        frontMotors->motorStop(0);
    }
}

void frontRightMotorCallback(const std_msgs::Float64& msg) {
    int16_t pwm = msg.data * 100;
    if (pwm > 0) {
        frontMotors->motorRun(1, MOTOR_DIRECTION_FORWARD, pwm);
    } else if (pwm < 0) {
        frontMotors->motorRun(1, MOTOR_DIRECTION_BACKWARD, abs(pwm));
    } else if (pwm == 0) {
        frontMotors->motorStop(1);
    }
}

void middleLeftMotorCallback(const std_msgs::Float64& msg) {
    int16_t pwm = msg.data * 100;
    if (pwm > 0) {
        middleMotors->motorRun(0, MOTOR_DIRECTION_FORWARD, pwm);
    } else if (pwm < 0) {
        middleMotors->motorRun(0, MOTOR_DIRECTION_BACKWARD, abs(pwm));
    } else if (pwm == 0) {
        middleMotors->motorStop(0);
    }
}

void middleRightMotorCallback(const std_msgs::Float64& msg) {
    int16_t pwm = msg.data * 100;
    if (pwm > 0) {
        middleMotors->motorRun(1, MOTOR_DIRECTION_FORWARD, pwm);
    } else if (pwm < 0) {
        middleMotors->motorRun(1, MOTOR_DIRECTION_BACKWARD, abs(pwm));
    } else if (pwm == 0) {
        middleMotors->motorStop(1);
    }
}

void rearLeftMotorCallback(const std_msgs::Float64& msg) {
    int16_t pwm = msg.data * 100;
    if (pwm > 0) {
        rearMotors->motorRun(0, MOTOR_DIRECTION_FORWARD, pwm);
    } else if (pwm < 0) {
        rearMotors->motorRun(0, MOTOR_DIRECTION_BACKWARD, abs(pwm));
    } else if (pwm == 0) {
        rearMotors->motorStop(0);
    }
}

void rearRightMotorCallback(const std_msgs::Float64& msg) {
    int16_t pwm = msg.data * 100;
    if (pwm > 0) {
        rearMotors->motorRun(1, MOTOR_DIRECTION_FORWARD, pwm);
    } else if (pwm < 0) {
        rearMotors->motorRun(1, MOTOR_DIRECTION_BACKWARD, abs(pwm));
    } else if (pwm == 0) {
        rearMotors->motorStop(1);
    }
}

void sigint_handler(int sig) {
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    // All the default sigint handler does is call shutdown()
    ROS_INFO("Shutting down motors on exit");
    frontMotors->motorStop(0);
    frontMotors->motorStop(1);
    middleMotors->motorStop(0);
    middleMotors->motorStop(1);
    rearMotors->motorStop(0);
    rearMotors->motorStop(1);
    ros::shutdown();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "dc_motors");
	ros::NodeHandle node;
    signal(SIGINT, sigint_handler);
	ros::Subscriber front_left_motor_target_vel_sub = node.subscribe("/smallbot/front_left_wheel/pwm", 1, &frontLeftMotorCallback);
	ros::Subscriber front_right_motor_target_vel_sub = node.subscribe("/smallbot/front_right_wheel/pwm", 1, &frontRightMotorCallback);
	ros::Subscriber middle_left_motor_target_vel_sub = node.subscribe("/smallbot/middle_left_wheel/pwm", 1, &middleLeftMotorCallback);
	ros::Subscriber middle_right_motor_target_vel_sub = node.subscribe("/smallbot/middle_right_wheel/pwm", 1, &middleRightMotorCallback);
	ros::Subscriber rear_left_motor_target_vel_sub = node.subscribe("/smallbot/rear_left_wheel/pwm", 1, &rearLeftMotorCallback);
	ros::Subscriber rear_right_motor_target_vel_sub = node.subscribe("/smallbot/rear_right_wheel/pwm", 1, &rearRightMotorCallback);
	ros::spin();
	return 0;
}
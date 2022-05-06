
#ifndef SMALLBOT_HARDWARE_INTERFACE_HPP_
#define SMALLBOT_HARDWARE_INTERFACE_HPP_

#include <boost/assign/list_of.hpp>
#include <sstream>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/ros.h>

class SmallBotHardwareInterface : public hardware_interface::RobotHW {
public:
	SmallBotHardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node, double target_max_wheel_angular_speed);

	void updateJointsFromHardware(const ros::Duration& period);
	void writeCommandsToHardware();

private:
	ros::NodeHandle _node;
	ros::NodeHandle _private_node;

	hardware_interface::JointStateInterface _joint_state_interface;
	hardware_interface::VelocityJointInterface _velocity_joint_interface;

	ros::Subscriber _front_left_wheel_angle_sub;
	ros::Subscriber _front_right_wheel_angle_sub;
	ros::Subscriber _middle_left_wheel_angle_sub;
	ros::Subscriber _middle_right_wheel_angle_sub;
	ros::Subscriber _rear_left_wheel_angle_sub;
	ros::Subscriber _rear_right_wheel_angle_sub;

	ros::Publisher _front_left_wheel_vel_pub;
	ros::Publisher _front_right_wheel_vel_pub;
	ros::Publisher _middle_left_wheel_vel_pub;
	ros::Publisher _middle_right_wheel_vel_pub;
	ros::Publisher _rear_left_wheel_vel_pub;
	ros::Publisher _rear_right_wheel_vel_pub;

	struct Joint {
		double position;
		double position_offset;
		double velocity;
		double effort;
		double velocity_command;

		Joint()
			: position(0)
			, velocity(0)
			, effort(0)
			, velocity_command(0) { }
	} _joints[6];

	double _front_left_wheel_angle;
	double _front_right_wheel_angle;
	double _middle_left_wheel_angle;
	double _middle_right_wheel_angle;
	double _rear_left_wheel_angle;
	double _rear_right_wheel_angle;
	double _max_wheel_angular_speed;

	void registerControlInterfaces();
	void frontLeftWheelAngleCallback(const std_msgs::Float64& msg);
	void frontRightWheelAngleCallback(const std_msgs::Float64& msg);
	void middleLeftWheelAngleCallback(const std_msgs::Float64& msg);
	void middleRightWheelAngleCallback(const std_msgs::Float64& msg);
	void rearLeftWheelAngleCallback(const std_msgs::Float64& msg);
	void rearRightWheelAngleCallback(const std_msgs::Float64& msg);
	void limitDifferentialSpeed(double& diff_speed_left_side, double& diff_speed_right_side);
};

SmallBotHardwareInterface::SmallBotHardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node, double target_max_wheel_angular_speed)
	: _node(node)
	, _private_node(private_node)
	, _max_wheel_angular_speed(target_max_wheel_angular_speed) {
	registerControlInterfaces();

	_front_left_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/smallbot/front_left_wheel/target_velocity", 1);
	_front_right_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/smallbot/front_right_wheel/target_velocity", 1);
	_middle_left_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/smallbot/middle_left_wheel/target_velocity", 1);
	_middle_right_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/smallbot/middle_right_wheel/target_velocity", 1);
	_rear_left_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/smallbot/rear_left_wheel/target_velocity", 1);
	_rear_right_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/smallbot/rear_right_wheel/target_velocity", 1);

	_front_left_wheel_angle_sub = _node.subscribe("/smallbot/front_left_wheel/angle", 1, &SmallBotHardwareInterface::frontLeftWheelAngleCallback, this);
	_front_right_wheel_angle_sub = _node.subscribe("/smallbot/front_right_wheel/angle", 1, &SmallBotHardwareInterface::frontRightWheelAngleCallback, this);
	_middle_left_wheel_angle_sub = _node.subscribe("/smallbot/middle_left_wheel/angle", 1, &SmallBotHardwareInterface::middleLeftWheelAngleCallback, this);
	_middle_right_wheel_angle_sub = _node.subscribe("/smallbot/middle_right_wheel/angle", 1, &SmallBotHardwareInterface::middleRightWheelAngleCallback, this);
	_rear_left_wheel_angle_sub = _node.subscribe("/smallbot/rear_left_wheel/angle", 1, &SmallBotHardwareInterface::rearLeftWheelAngleCallback, this);
	_rear_right_wheel_angle_sub = _node.subscribe("/smallbot/rear_right_wheel/angle", 1, &SmallBotHardwareInterface::rearRightWheelAngleCallback, this);
}

void SmallBotHardwareInterface::writeCommandsToHardware() {
	double diff_angle_speed_front_left = _joints[0].velocity_command;
	double diff_angle_speed_front_right = _joints[1].velocity_command;
	double diff_angle_speed_middle_left = _joints[2].velocity_command;
	double diff_angle_speed_middle_right = _joints[3].velocity_command;
	double diff_angle_speed_rear_left = _joints[4].velocity_command;
	double diff_angle_speed_rear_right = _joints[5].velocity_command;

	limitDifferentialSpeed(diff_angle_speed_front_left, diff_angle_speed_front_right);
	limitDifferentialSpeed(diff_angle_speed_middle_left, diff_angle_speed_middle_right);
	limitDifferentialSpeed(diff_angle_speed_rear_left, diff_angle_speed_rear_right);

	std_msgs::Float64 front_left_wheel_vel_msg;
	std_msgs::Float64 front_right_wheel_vel_msg;
	std_msgs::Float64 middle_left_wheel_vel_msg;
	std_msgs::Float64 middle_right_wheel_vel_msg;
	std_msgs::Float64 rear_left_wheel_vel_msg;
	std_msgs::Float64 rear_right_wheel_vel_msg;

	front_left_wheel_vel_msg.data = diff_angle_speed_front_left;
	front_right_wheel_vel_msg.data = diff_angle_speed_front_right;
	middle_left_wheel_vel_msg.data = diff_angle_speed_middle_left;
	middle_right_wheel_vel_msg.data = diff_angle_speed_middle_right;
	rear_left_wheel_vel_msg.data = diff_angle_speed_rear_left;
	rear_right_wheel_vel_msg.data = diff_angle_speed_rear_right;

	_front_left_wheel_vel_pub.publish(front_left_wheel_vel_msg);
	_front_right_wheel_vel_pub.publish(front_right_wheel_vel_msg);
	_middle_left_wheel_vel_pub.publish(middle_left_wheel_vel_msg);
	_middle_right_wheel_vel_pub.publish(middle_right_wheel_vel_msg);
	_rear_left_wheel_vel_pub.publish(rear_left_wheel_vel_msg);
	_rear_right_wheel_vel_pub.publish(rear_right_wheel_vel_msg);
}

void SmallBotHardwareInterface::updateJointsFromHardware(const ros::Duration& period) {
	double delta_front_left_wheel = _front_left_wheel_angle - _joints[0].position - _joints[0].position_offset;
	double delta_front_right_wheel = _front_right_wheel_angle - _joints[1].position - _joints[1].position_offset;
	double delta_middle_left_wheel = _middle_left_wheel_angle - _joints[2].position - _joints[2].position_offset;
	double delta_middle_right_wheel = _middle_right_wheel_angle - _joints[3].position - _joints[3].position_offset;
	double delta_rear_left_wheel = _rear_left_wheel_angle - _joints[4].position - _joints[4].position_offset;
	double delta_rear_right_wheel = _rear_right_wheel_angle - _joints[5].position - _joints[5].position_offset;

	if (std::abs(delta_front_left_wheel) < 1) {
		_joints[0].position += delta_front_left_wheel;
		_joints[0].velocity = delta_front_left_wheel / period.toSec();
	} else {
		_joints[0].position_offset += delta_front_left_wheel;
	}

	if (std::abs(delta_front_right_wheel) < 1) {
		_joints[1].position += delta_front_right_wheel;
		_joints[1].velocity = delta_front_right_wheel / period.toSec();
	} else {
		_joints[1].position_offset += delta_front_right_wheel;
	}

	if (std::abs(delta_middle_left_wheel) < 1) {
		_joints[2].position += delta_middle_left_wheel;
		_joints[3].velocity = delta_middle_left_wheel / period.toSec();
	} else {
		_joints[2].position_offset += delta_middle_left_wheel;
	}

	if (std::abs(delta_middle_right_wheel) < 1) {
		_joints[3].position += delta_middle_right_wheel;
		_joints[3].velocity = delta_middle_right_wheel / period.toSec();
	} else {
		_joints[3].position_offset += delta_middle_right_wheel;
	}
	if (std::abs(delta_rear_left_wheel) < 1) {
		_joints[4].position += delta_rear_left_wheel;
		_joints[4].velocity = delta_rear_left_wheel / period.toSec();
	} else {
		_joints[4].position_offset += delta_rear_left_wheel;
	}

	if (std::abs(delta_rear_right_wheel) < 1) {
		_joints[5].position += delta_rear_right_wheel;
		_joints[5].velocity = delta_rear_right_wheel / period.toSec();
	} else {
		_joints[5].position_offset += delta_rear_right_wheel;
	}

}

void SmallBotHardwareInterface::registerControlInterfaces() {
	ros::V_string joint_names = boost::assign::list_of("base_to_front_left_wheel")("base_to_front_right_wheel")
		("base_to_middle_left_wheel")("base_to_middle_right_wheel")("base_to_rear_left_wheel")("base_to_rear_right_wheel");

	for (unsigned int i = 0; i < joint_names.size(); i++) {
		hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &_joints[i].position, &_joints[i].velocity, &_joints[i].effort);
		_joint_state_interface.registerHandle(joint_state_handle);

		hardware_interface::JointHandle joint_handle(joint_state_handle, &_joints[i].velocity_command);
		_velocity_joint_interface.registerHandle(joint_handle);
	}
	registerInterface(&_joint_state_interface);
	registerInterface(&_velocity_joint_interface);
}

void SmallBotHardwareInterface::frontLeftWheelAngleCallback(const std_msgs::Float64& msg) {
	_front_left_wheel_angle = msg.data;
}

void SmallBotHardwareInterface::frontRightWheelAngleCallback(const std_msgs::Float64& msg) {
	_front_right_wheel_angle = msg.data;
}

void SmallBotHardwareInterface::middleLeftWheelAngleCallback(const std_msgs::Float64& msg) {
	_middle_left_wheel_angle = msg.data;
}

void SmallBotHardwareInterface::middleRightWheelAngleCallback(const std_msgs::Float64& msg) {
	_middle_right_wheel_angle = msg.data;
}

void SmallBotHardwareInterface::rearLeftWheelAngleCallback(const std_msgs::Float64& msg) {
	_rear_left_wheel_angle = msg.data;
}

void SmallBotHardwareInterface::rearRightWheelAngleCallback(const std_msgs::Float64& msg) {
	_rear_right_wheel_angle = msg.data;
}

void SmallBotHardwareInterface::limitDifferentialSpeed(double& diff_speed_left_side, double& diff_speed_right_side) {
	double large_speed = std::max(std::abs(diff_speed_left_side), std::abs(diff_speed_right_side));
	if (large_speed > _max_wheel_angular_speed) {
		diff_speed_left_side *= _max_wheel_angular_speed / large_speed;
		diff_speed_right_side *= _max_wheel_angular_speed / large_speed;
	}
}

#endif // SMALLBOT_HARDWARE_INTERFACE_HPP_

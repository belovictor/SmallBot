#!/usr/bin/python

import rospy
import socket
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from math import pi
from smallbot_receiver import SmallBotReceiver


def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    return IP


class SmallBotRemote():
    DEFAULT_UDP_PORT = 12345

    MODE_IDLE = 0
    MODE_STAND = 1
    MODE_ANGLE = 2
    MODE_WALK = 3

    MAX_ROLL_DEG = 45
    MAX_YAW_DEG = 45
    MAX_PATCH_DEG = 45

    MAX_FORWARD_SPEED = 0.2
    MAX_STRAFE_SPEED = 0.1
    MAX_YAW_SPEED_DEG = 15

    def __init__(self):

        self._angle_cmd_msg = Vector3()
        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._vel_cmd_msg = Twist()
        self._vel_cmd_msg.linear.x = 0
        self._vel_cmd_msg.linear.y = 0
        self._vel_cmd_msg.linear.z = 0
        self._vel_cmd_msg.angular.x = 0
        self._vel_cmd_msg.angular.y = 0
        self._vel_cmd_msg.angular.z = 0

        self._walk_event_cmd_msg = Bool()
        self._walk_event_cmd_msg.data = True  # Mostly acts as an event driven action on receipt of a true message

        self._stand_event_cmd_msg = Bool()
        self._stand_event_cmd_msg.data = True

        self._idle_event_cmd_msg = Bool()
        self._idle_event_cmd_msg.data = True

        rospy.loginfo("Setting Up the Spot Micro Remote Control Node...")

        # Set up and title the ros node for this code
        rospy.init_node('spot_micro_remote_control')

        # Create publishers for commanding velocity, angle, and robot states
        self._ros_pub_angle_cmd = rospy.Publisher('/angle_cmd', Vector3, queue_size=1)
        self._ros_pub_vel_cmd = rospy.Publisher('/smallbot_drive_controller/cmd_vel', Twist, queue_size=1)
        self._ros_pub_walk_cmd = rospy.Publisher('/walk_cmd', Bool, queue_size=1)
        self._ros_pub_stand_cmd = rospy.Publisher('/stand_cmd', Bool, queue_size=1)
        self._ros_pub_idle_cmd = rospy.Publisher('/idle_cmd', Bool, queue_size=1)

        self._receiver = None

        rospy.loginfo("Remote control node publishers corrrectly initialized")

    def reset_all_motion_commands_to_zero(self):
        '''Reset body motion cmd states to zero and publish zero value body motion commands'''

        self._vel_cmd_msg.linear.x = 0
        self._vel_cmd_msg.linear.y = 0
        self._vel_cmd_msg.linear.z = 0
        self._vel_cmd_msg.angular.x = 0
        self._vel_cmd_msg.angular.y = 0
        self._vel_cmd_msg.angular.z = 0

        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

    def reset_all_angle_commands_to_zero(self):
        '''Reset angle cmd states to zero and publish them'''

        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

    def run(self):
        print('Run your SpotControl application and connect to ' + get_ip() + ':' + str(self.DEFAULT_UDP_PORT))
        self._receiver = SmallBotReceiver()
        # Publish all body motion commands to 0
        self.reset_all_motion_commands_to_zero()
        while not rospy.is_shutdown():
            data_string = self._receiver.receive()
            print(data_string)
            command_array = data_string.split(' ')
            if len(command_array) < 2:
                print('Command has less then 2 components, ignoring')
                continue
            if command_array[0] == 'LEFT' or command_array[0] == 'RIGHT':
                print('Joystick command received')
                if len(command_array) < 3:
                    print('Joystick commands should have 2 arguments at least')
                    continue
                axis_x = float(command_array[1])
                axis_y = float(command_array[2])
                if command_array[0] == 'LEFT':
                    self._vel_cmd_msg.linear.x = axis_y * self.MAX_FORWARD_SPEED * -1
                    self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                elif command_array[0] == 'RIGHT':
                    self._vel_cmd_msg.angular.z = pi / 180 * axis_x * self.MAX_YAW_SPEED_DEG * -1
                    self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s ' \
                        % (self._vel_cmd_msg.linear.x, self._vel_cmd_msg.linear.y,
                            self._vel_cmd_msg.angular.z * 180 / pi))
            elif command_array[0] == 'ACTIVE':
                print('Active command received')
                if command_array[1] == '1':
                    rospy.loginfo('Stand command issued from remote control.')
                else:
                    rospy.loginfo('Idle command issued from remote control.')
            elif command_array[0] == 'WALK':
                print('Walk command received')
                if command_array[1] == '1':
                    rospy.loginfo('Entering walk command mode.')
                else:
                    rospy.loginfo('Entering angle command mode.')
            else:
                print('Unsupported command received')

if __name__ == "__main__":
    smc = SmallBotRemote()
    smc.run()

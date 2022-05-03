#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import math
import rospy
from encoder import Encoder
from std_msgs.msg import Float64

enc1pin1 = 7
enc1pin2 = 13
enc2pin1 = 12
enc2pin2 = 16
enc3pin1 = 15
enc3pin2 = 19
enc4pin1 = 18
enc4pin2 = 22
enc5pin1 = 21
enc5pin2 = 23
enc6pin1 = 24
enc6pin2 = 26

# This is set depending on motor encoder parameters
PULSES_PER_REVOLUTION = 374.22

class SmallBotEncoder():
    def __init__(self, pinA, pinB, reverse=False):
        self._pinA = pinA
        self._pinB = pinB
        self._reverse = reverse
        self._encoder = Encoder(self._pinA, self._pinB)
        self._encoderPosition = 0
    def getAngle(self):
        angle = self._encoder.getValue() / PULSES_PER_REVOLUTION * math.pi * 2
        if self._reverse == True:
            return -1 * angle
        else:
            return angle

class Encoders():
    def __init__(self, update_rate = 100):
        rospy.loginfo("Setting up encoders node...")
        rospy.init_node('encoders')
        GPIO.setmode(GPIO.BOARD)
        self._update_rate = update_rate
        self._front_left_wheel_position = 0
        self._front_right_wheel_position = 0
        self._middle_left_wheel_position = 0
        self._middle_right_wheel_position = 0
        self._rear_left_wheel_position = 0
        self._rear_right_wheel_position = 0
        self._e1 = SmallBotEncoder(enc1pin1, enc1pin2, True)
        self._e2 = SmallBotEncoder(enc2pin1, enc2pin2, False)
        self._e3 = SmallBotEncoder(enc3pin1, enc3pin2, True)
        self._e4 = SmallBotEncoder(enc4pin1, enc4pin2, False)
        self._e5 = SmallBotEncoder(enc5pin1, enc5pin2, True)
        self._e6 = SmallBotEncoder(enc6pin1, enc6pin2, False)
        self._front_left_wheel_angle_pub = rospy.Publisher('/smallbot/front_left_wheel/angle', Float64, queue_size=1)
        self._front_right_wheel_angle_pub = rospy.Publisher('/smallbot/front_right_wheel/angle', Float64, queue_size=1)
        self._middle_left_wheel_angle_pub = rospy.Publisher('/smallbot/middle_left_wheel/angle', Float64, queue_size=1)
        self._middle_right_wheel_angle_pub = rospy.Publisher('/smallbot/middle_right_wheel/angle', Float64, queue_size=1)
        self._rear_left_wheel_angle_pub = rospy.Publisher('/smallbot/rear_left_wheel/angle', Float64, queue_size=1)
        self._rear_right_wheel_angle_pub = rospy.Publisher('/smallbot/rear_right_wheel/angle', Float64, queue_size=1)
        self._front_left_wheel_velocity_pub = rospy.Publisher('/smallbot/front_left_wheel/current_velocity', Float64, queue_size=1)
        self._front_right_wheel_velocity_pub = rospy.Publisher('/smallbot/front_right_wheel/current_velocity', Float64, queue_size=1)
        self._middle_left_wheel_velocity_pub = rospy.Publisher('/smallbot/middle_left_wheel/current_velocity', Float64, queue_size=1)
        self._middle_right_wheel_velocity_pub = rospy.Publisher('/smallbot/middle_right_wheel/current_velocity', Float64, queue_size=1)
        self._rear_left_wheel_velocity_pub = rospy.Publisher('/smallbot/rear_left_wheel/current_velocity', Float64, queue_size=1)
        self._rear_right_wheel_velocity_pub = rospy.Publisher('/smallbot/rear_right_wheel/current_velocity', Float64, queue_size=1)
        self._lastTime = time.time()
        rospy.loginfo("Encoders node initialized")
    def run(self):
        r = rospy.Rate(self._update_rate)
        try:
            while not rospy.is_shutdown():
                thisTime = time.time()
                elapsedTime = thisTime - self._lastTime
                self._lastTime = thisTime
                front_left_wheel_delta = self._e1.getAngle() - self._front_left_wheel_position
                front_right_wheel_delta = self._e2.getAngle() - self._front_right_wheel_position
                middle_left_wheel_delta = self._e1.getAngle() - self._middle_left_wheel_position
                middle_right_wheel_delta = self._e2.getAngle() - self._middle_right_wheel_position
                rear_left_wheel_delta = self._e1.getAngle() - self._rear_left_wheel_position
                rear_right_wheel_delta = self._e2.getAngle() - self._rear_right_wheel_position
                self._front_left_wheel_position = self._e1.getAngle()
                self._front_right_wheel_position = self._e2.getAngle()
                self._middle_left_wheel_position = self._e3.getAngle()
                self._middle_right_wheel_position = self._e4.getAngle()
                self._rear_left_wheel_position = self._e5.getAngle()
                self._rear_right_wheel_position = self._e6.getAngle()
                self._front_left_wheel_angle_pub.publish(self._e1.getAngle())
                self._front_right_wheel_angle_pub.publish(self._e2.getAngle())
                self._middle_left_wheel_angle_pub.publish(self._e3.getAngle())
                self._middle_right_wheel_angle_pub.publish(self._e4.getAngle())
                self._rear_left_wheel_angle_pub.publish(self._e5.getAngle())
                self._rear_right_wheel_angle_pub.publish(self._e6.getAngle())
                self._front_left_wheel_velocity_pub.publish(front_left_wheel_delta / elapsedTime)
                self._front_right_wheel_velocity_pub.publish(front_right_wheel_delta / elapsedTime)
                self._middle_left_wheel_velocity_pub.publish(middle_left_wheel_delta / elapsedTime)
                self._middle_right_wheel_velocity_pub.publish(middle_right_wheel_delta / elapsedTime)
                self._rear_left_wheel_velocity_pub.publish(rear_left_wheel_delta / elapsedTime)
                self._rear_right_wheel_velocity_pub.publish(rear_right_wheel_delta / elapsedTime)
                r.sleep()
        finally:
            GPIO.cleanup()

if __name__ == '__main__':
    encoders = Encoders(100)
    encoders.run()

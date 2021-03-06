#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, copy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from raspicat_basic.msg import LightSensorValues
import math
import random


import json
from json.decoder import WHITESPACE


class WallStop():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback_lightsensors)

        self.rotate = False

        self.FIXED_KEYS = ["止まれ", "動け"] # stop0, move1
        self.keys = []
        for key in self.FIXED_KEYS:
            self.keys.append(unicode(key, 'utf-8'))


        f = open('/home/daisha/~/Desktop/googleassis/shirei2.txt')
        s = f.read()
        # print(list(loads_iter(s)))
        order = list(self.loads_iter(s))[-1]
        f.close()
        comm = order["data"].encode('utf-8')

        self.state = 0
        if comm == self.keys[1].encode('utf-8'):
            self.state = 1


    def loads_iter(self, s):
        size = len(s)
        decoder = json.JSONDecoder()

        end = 0
        while True:
            idx = WHITESPACE.match(s[end:]).end()
            i = end + idx
            if i >= size:
                break
            ob, end = decoder.raw_decode(s, i)
            yield ob

    def callback_lightsensors(self,messages):
        self.sensor_values = messages

    def run(self):
        rate = rospy.Rate(10)
        data = Twist()

        flame = 0
        data.linear.x = 0.0
        data.angular.z = 0

        while not rospy.is_shutdown():
            flame += 1

            # print(self.sensor_values)

            print(flame)
            if flame == 50:
                data.angular.z = 1.0 + (random.random() - 0.5)/4.0
                if random.random() < 0.5:
                    data.angular.z *= -1.0

            if flame >= 100:
                data.angular.z = 0
                flame = 0

                # command data reading
            f = open('/home/daisha/~/Desktop/googleassis/shirei2.txt')
            s = f.read()
            # print(list(loads_iter(s)))
            order = list(self.loads_iter(s))[-1]
            f.close()
            comm = order["data"].encode('utf-8')
            print(comm)

            if comm == self.keys[0].encode('utf-8'):
                print("command>stop")
                data.linear.x = 0.0
                data.angular.z = 0


            if comm == self.keys[1].encode('utf-8'):
                print("command>move")
                data.linear.x = 0.1
                # data.angular.z = 0
            else:
                data.angular.z = 0

            if self.sensor_values.right_forward < 100:
                print("RF")
                data.linear.x = 0.0
                # data.angular.z = 0
            if self.sensor_values.left_forward < 100:
                print("LF")
                data.linear.x = 0.0
                # data.angular.z = 0
            if self.sensor_values.right_side < 100:
                print("RS")
                data.linear.x = 0.0
                # data.angular.z = 0
            if self.sensor_values.left_side < 100:
                print("LS")
                data.linear.x = 0.0
                # data.angular.z = 0





            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_stop')

    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()

    w = WallStop()
    w.run()
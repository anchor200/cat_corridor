#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, copy, os, socket
from std_msgs.msg import UInt16
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
        self.buzzer = rospy.Publisher('/buzzer', UInt16, queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback_lightsensors)

        self.rotate = False
        self.manual = False  # 命令に従うモードかどうか

        self.FIXED_KEYS = ["止まれ", "動け", "atsumare", "teishi", "detected"] # stop0, move1
        self.keys = []
        for key in self.FIXED_KEYS:
            self.keys.append(unicode(key, 'utf-8'))


        f = open('/home/daisha/~/Desktop/googleassis/shirei2.txt')
        s = f.read()
        # print(list(loads_iter(s)))
        order = list(self.loads_iter(s))[-1]
        f.close()
        comm = order["data"].encode('utf-8')

        self.event_lists = 0
        with open('/home/daisha/~/Desktop/googleassis/shirei3.txt') as f:
            s = f.read()
            order = list(self.loads_iter(s))[-1]
            self.event_lists = len(list(self.loads_iter(s)))


        self.state = 0
        self.event_proc = False



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
        data2 = Twist()
        pitch = 0

        flame = 0
        before_event = 0
        eventflame = 0
        data.linear.x = 0.0
        data.angular.z = 0
        comm = ""
        self.ratio = 1.0

        self.to_wall = 1000


        data2.angular.z = 0
        data2.linear.x = 0.0

        while not rospy.is_shutdown():
            flame += 1
            if self.state < 2:
                pitch = 0

            f = open('/home/daisha/~/Desktop/googleassis/shirei2.txt')
            s = f.read()
            # print(list(loads_iter(s)))
            if len(list(self.loads_iter(s))) >= 1:
                order = list(self.loads_iter(s))[-1]
                comm = order["data"].encode('utf-8')
            f.close()

            # print(comm)


            if comm == self.keys[0].encode('utf-8') and self.event_proc == False:
                if self.state != 0:
                    self.state = 0
                    print("command>stop")

            if comm == self.keys[1].encode('utf-8') and self.event_proc == False:
                if self.state != 1:
                    self.state = 1
                    print("command>move")


            with open('/home/daisha/~/Desktop/googleassis/shirei3.txt') as f:
                s = f.read()
                order = list(self.loads_iter(s))[-1]
                if self.event_lists < len(list(self.loads_iter(s))):
                    self.event_lists = len(list(self.loads_iter(s)))
                    comm2 = order["data"].encode('utf-8')
                    if comm2 == self.keys[2].encode('utf-8') and self.state != 2:
                        self.state = 2
                        before_event = self.state
                    if comm2 == self.keys[4].encode('utf-8') and self.state != 2:
                        self.state = 3
                        before_event = self.state

            if self.state == 2 or self.state == 3:
                if self.event_proc == False:
                    print("event started")
                    self.event_proc = True
                    eventflame = 40
                    if self.state == 3:
                        eventflame = 300
                else:
                    if eventflame > 0:
                        eventflame -= 1
                    if eventflame == 0:
                        print("event ended")
                        self.event_proc = False
                        self.state = before_event
                        flame = 49

            if self.state == 0:
                data.linear.x = 0.0
                data.angular.z = 0

            if self.state == 1 or self.state == 3:
                data.linear.x = 0.0
                """if flame < 10:
                    data.linear.x = 0.01 * self.ratio
                elif flame < 20:
                    data.linear.x = 0.03 * self.ratio
                elif flame < 40:
                    data.linear.x = 0.05 * self.ratio"""
                if flame < 50:
                    data.linear.x = 0.06 * self.ratio

                if flame == 50:
                    data.angular.z = (1.0 + (random.random() - 0.5)/4.0)/1.9 * self.ratio
                    if random.random() < 0.5:
                        data.angular.z *= -1.0

                if flame >= 80:
                    data.angular.z += 0.05 * (0.0 - data.angular.z)

                if flame >= 100:
                    data.angular.z = 0
                    flame = 0



            comm3 = ""
            with open('/home/daisha/~/Desktop/googleassis/effect.txt') as f:
                s = f.read()
                if len(list(self.loads_iter(s))) != 0:
                    order = list(self.loads_iter(s))[-1]
                    comm3 = order["data"].encode('utf-8')
            if comm3 == self.keys[3].encode('utf-8'):
                self.manual = True
                self.ratio *= 0.95
            else:
                self.manual = False
                if self.ratio < 1.0:
                    self.ratio *= 1.01
                if self.ratio > 1.0:
                    self.ratio = 1.0


            # print(self.manual)


            to_wall_buf = min(self.sensor_values.right_forward, self.sensor_values.left_forward, self.sensor_values.right_side, self.sensor_values.left_side)
            if to_wall_buf < 700:
                if self.to_wall - to_wall_buf > 0:
                    data.linear.x = 0.06 * to_wall_buf/900

            if to_wall_buf < 400:
                data.linear.x = 0.0

            self.to_wall = to_wall_buf


            print(data.linear.x)


            """if self.sensor_values.right_forward < 500:
                # print("RF")
                # data.linear.x = 0.0
                # data.angular.z = 0
            if self.sensor_values.left_forward < 500:
                # print("LF")
                # data.linear.x = 0.0
            if self.sensor_values.right_side < 500:
                # print("RS")
                # data.linear.x = 0.0
            if self.sensor_values.left_side < 500:
                # print("LS")
                # data.linear.x = 0.0"""

            if eventflame > 0: # events
                if self.state == 2:
                    data.linear.x = 0.0
                    data.angular.z = 2
                    if eventflame > 20:
                        data.angular.z = -2
                    if eventflame < 5:
                        data.angular.z = 0
                if self.state == 3:
                    # data.linear.x = 0.0
                    if eventflame > 200:
                        pitch = 2000
                    elif eventflame > 100:
                        pitch = 1000
                    elif eventflame > 0:
                        pitch = 441
                    self.buzzer.publish(pitch)
                    continue
                    # data.angular.z = 0


            # print(self.ratio)
            # print(data.linear.x)

            if self.state != 3:
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



#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, copy, os, socket


def get_line(self):
    line = ""
    self.sock.timeout(0.1)
    while not rospy.is_shutdown():
        try:
            v = self.sock.recv(1)
        except socket.timeout as e:
            print e
            return "stable"

        if v == '\n':
            return line
        line += v




rate = rospy.Rate(10)
while not rospy.is_shutdown():
    try:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(("localhost", 10500))
        break
    except:
        rate.sleep()
rospy.on_shutdown(self.sock.close)
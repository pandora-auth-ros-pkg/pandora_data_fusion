#!/usr/bin/env python
# Copyright <alert_handler_test.py>

import math

import unittest

import rospy

from state_manager.state_client import StateClient
from geometry_msgs.msg import Point
from pandora_common_msgs.msg import GeneralAlertMsg 
from pandora_arm_hardware_interface.msg import Co2Msg
from sensor_msgs.msg import Image

def distance(a, b):

    return math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2 )

def direction(a, b):
        
    dire = Point()
    norm = distance(a, b)
    dire.x = (b.x - a.x)/norm
    dire.y = (b.y - a.y)/norm
    dire.z = (b.z - a.z)/norm
    return dire

class TestBase(unittest.TestCase):

    def mockCallback(self, data):

        self.alertList.append(data)
        self.replied = True
        rospy.logdebug(self.alertList)

    @classmethod
    def connect(cls, processor):

        cls.state_changer = StateClient(False)
        rospy.sleep(0.1)
        cls.state_changer.transition_to_state(2)
        rospy.sleep(2)
        
        if processor is "co2":
            cls.mock_publisher = rospy.Publisher("/test/raw_input", Co2Msg)
        elif processor is "thermal":
            cls.mock_publisher = rospy.Publisher("/test/raw_input", Image)

    @classmethod
    def disconnect(cls):

        cls.mock_publisher.unregister()

    def setUp(self):

        self.mock_subscriber = rospy.Subscriber(
            "/test/alert_output", 
            GeneralAlertMsg, self.mockCallback)
        self.alertList = []
        self.replied = False

    def tearDown(self):

        self.mock_subscriber.unregister()


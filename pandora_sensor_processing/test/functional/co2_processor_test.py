#!/usr/bin/env python
# Copyright <alert_handler_static_test.py>

PKG = 'pandora_sensor_processing'
NAME = 'co2_processor_test'

import sys
import math

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

import test_base
from pandora_arm_hardware_interface.msg import Co2Msg
from pandora_common_msgs.msg import GeneralAlertMsg 

class Co2ProcessorTest(test_base.TestBase):
    
    def publish_raw(self):

        self.raw = Co2Msg()
        self.raw.header.frame_id = "aer"
        self.raw.header.stamp = rospy.get_rostime()
        self.raw.co2_percentage = 0.4

        self.mock_publisher.publish(self.raw)
        self.replied = False
        rospy.sleep(0.1)

    def expect_success(self, times):

        self.assertTrue(self.replied)
        self.assertEqual(len(self.alertList), times)
        self.assertAlmostEqual(self.alertList[times - 1].probability, 0.68171501)
        self.assertEqual(self.alertList[times - 1].yaw, 0)
        self.assertEqual(self.alertList[times - 1].pitch, 0)
        self.assertEqual(self.alertList[times - 1].header.frame_id, "aer")

    def expect_fail(self):

        self.assertFalse(self.replied)

    def test_mode_exploration(self):

        self.state_changer.transition_to_state(2)
        rospy.sleep(1)
        self.publish_raw()
        self.expect_fail()

    def test_mode_identification(self):

        self.state_changer.transition_to_state(3)
        rospy.sleep(1)
        self.publish_raw()
        self.expect_success(1)

    def test_toggle(self):

        self.state_changer.transition_to_state(2)
        rospy.sleep(1)
        self.publish_raw()
        self.expect_fail()
        self.state_changer.transition_to_state(2)
        rospy.sleep(1)
        self.publish_raw()
        self.expect_fail()
        self.state_changer.transition_to_state(3)
        rospy.sleep(1)
        self.publish_raw()
        self.expect_success(1)
        self.state_changer.transition_to_state(3)
        rospy.sleep(1)
        self.publish_raw()
        self.expect_success(2)

if __name__ == '__main__':

    rospy.sleep(10)
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    Co2ProcessorTest.connect("co2")
    rostest.rosrun(PKG, NAME, Co2ProcessorTest, sys.argv)
    Co2ProcessorTest.disconnect()


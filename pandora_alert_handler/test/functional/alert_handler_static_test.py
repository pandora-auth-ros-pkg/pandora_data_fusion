#!/usr/bin/env python
# Copyright <alert_handler_static_test.py>

PKG = 'pandora_alert_handler'
NAME = 'alert_handler_functional_static_test'

import sys

import unittest
import rostest

import rospy
import alert_delivery

from data_fusion_communications.srv import GetObjectsSrv
from data_fusion_communications.srv import GetObjectsSrvResponse
#from data_fusion_communications.srv import FlushQueuesSrv

class TestAlertHandlerStatic(unittest.TestCase):

    def setUp(self):
        
        self.deliveryBoy = alert_delivery.AlertDeliveryBoy()
        rospy.init_node(NAME, anonymous=True)
        self.get_objects = rospy.ServiceProxy('/data_fusion/get_objects', GetObjectsSrv, True)
        #self.flush_lists = rospy.ServiceProxy('/data_fusion/flush_queues', FlushQueuesSrv, True)

    def test_works(self):

        self.assertEqual(1+1, 2)

    def test_a_simple_alert(self):

        #self.flush_lists()
        rospy.wait_for_service('/data_fusion/get_objects')
        self.deliveryBoy.deliverHazmatOrder(0, 0, 1)
        rospy.sleep(1.2)
        for i in range(2):
            try:
                resp = self.get_objects()
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        pose = resp.hazmats.pop().pose
        self.assertAlmostEqual(pose.position.x, 0.99525856)
        self.assertAlmostEqual(pose.position.y, 0.52000838)
        self.assertAlmostEqual(pose.position.z, 1)
        self.assertAlmostEqual(pose.orientation.x, 0)
        self.assertAlmostEqual(pose.orientation.y, 0)
        self.assertAlmostEqual(pose.orientation.z, 0.70710679)
        self.assertAlmostEqual(pose.orientation.w, 0.70710679)

    def test_qr_hazmat_hole_tpa_coexist(self):

        #self.flush_lists()
        self.deliveryBoy.clearOrderList()
        self.deliveryBoy.getOrderListFromBoss('orders/mixed_order.in')
        # hole: yaw = -0.1 pitch = 0
        self.deliveryBoy.deliverNextOrder()
        rospy.sleep(1.2)
        for i in range(2):
            try:
                resp = self.get_objects()
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        self.assertEqual(len(resp.holes), 1)
        self.assertEqual(len(resp.hazmats), 1)
        self.assertEqual(len(resp.qrs), 0)
        self.assertEqual(len(resp.tpas), 0)
        # qr: yaw = 0 pitch = 0
        self.deliveryBoy.deliverNextOrder()
        rospy.sleep(1.2)
        for i in range(2):
            try:
                resp = self.get_objects()
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        self.assertEqual(len(resp.holes), 1)
        self.assertEqual(len(resp.hazmats), 1)
        self.assertEqual(len(resp.qrs), 1)
        self.assertEqual(len(resp.tpas), 0)
        # hazmat: yaw = 0.1 pitch = 0.012
        self.deliveryBoy.deliverNextOrder()
        rospy.sleep(1.2)
        for i in range(2):
            try:
                resp = self.get_objects()
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        self.assertEqual(len(resp.holes), 1)
        self.assertEqual(len(resp.hazmats), 1)
        self.assertEqual(len(resp.qrs), 1)
        self.assertEqual(len(resp.tpas), 0)
        # hole: yaw = 0 pitch = 0
        self.deliveryBoy.deliverNextOrder()
        rospy.sleep(1.2)
        for i in range(2):
            try:
                resp = self.get_objects()
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        self.assertEqual(len(resp.holes), 1)
        self.assertEqual(len(resp.hazmats), 1)
        self.assertEqual(len(resp.qrs), 1)
        self.assertEqual(len(resp.tpas), 0)
        # tpa: yaw = -0.1 pitch = -0.15
        self.deliveryBoy.deliverNextOrder()
        rospy.sleep(1.2)
        for i in range(2):
            try:
                resp = self.get_objects()
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        self.assertEqual(len(resp.holes), 1)
        self.assertEqual(len(resp.hazmats), 1)
        self.assertEqual(len(resp.qrs), 1)
        self.assertEqual(len(resp.tpas), 1)

    def test_kalman_filter_of_one_object(self):
        pass



if __name__ == '__main__':
  rospy.sleep(1)
  rostest.rosrun(PKG, NAME, TestAlertHandlerStatic, sys.argv)


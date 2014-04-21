#!/usr/bin/env python
# Copyright <alert_handler_static_test.py>

PKG = 'pandora_alert_handler'
NAME = 'alert_handler_static_test'

import sys
import math

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

import alert_delivery

from data_fusion_communications.srv import GetObjectsSrv
from data_fusion_communications.srv import GetObjectsSrvResponse
from std_srvs.srv import Empty
from geometry_msgs.msg import Point

def distance(a, b):

    return math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2 )

def direction(a, b):
        
    dire = Point()
    dire.x = b.x - a.x
    dire.y = b.y - a.y
    dire.z = b.z - a.z
    return dire

class AlertHandlerStaticTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        
        cls.deliveryBoy = alert_delivery.AlertDeliveryBoy()
        cls.get_objects = rospy.ServiceProxy('/data_fusion/get_objects', GetObjectsSrv, True)
        rospy.wait_for_service('/data_fusion/get_objects')
        cls.flush_lists = rospy.ServiceProxy('/data_fusion/flush_queues', Empty, True)
        rospy.wait_for_service('/data_fusion/flush_queues')
        cls.deliveryBoy.deliverHazmatOrder(0, 0, 1)
        cls.flush_lists()

    def test_works(self):

        self.assertEqual(1+1, 2)

    def test_a_simple_alert(self):

        self.flush_lists()
        self.deliveryBoy.deliverHazmatOrder(0, 0, 1)
        rospy.sleep(0.1)
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

    def test_objects_coexist(self):

        self.flush_lists()
        self.deliveryBoy.clearOrderList()
        self.deliveryBoy.getOrderListFromBoss('orders/mixed_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)
            try:
                outs.append(self.get_objects())
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        
        # hole: yaw = -0.1 pitch = 0
        self.assertEqual(len(outs[0].holes), 1)
        self.assertEqual(len(outs[0].hazmats), 0)
        self.assertEqual(len(outs[0].qrs), 0)
        self.assertEqual(len(outs[0].tpas), 0)
        # qr: yaw = 0 pitch = 0
        self.assertEqual(len(outs[1].holes), 1)
        self.assertEqual(len(outs[1].hazmats), 0)
        self.assertEqual(len(outs[1].qrs), 1)
        self.assertEqual(len(outs[1].tpas), 0)
        # hazmat: yaw = 0.1 pitch = 0.012
        self.assertEqual(len(outs[2].holes), 1)
        self.assertEqual(len(outs[2].hazmats), 1)
        self.assertEqual(len(outs[2].qrs), 1)
        self.assertEqual(len(outs[2].tpas), 0)
        # hole: yaw = 0 pitch = 0
        self.assertEqual(len(outs[3].holes), 1)
        self.assertEqual(len(outs[3].hazmats), 1)
        self.assertEqual(len(outs[3].qrs), 1)
        self.assertEqual(len(outs[3].tpas), 0)
        # tpa: yaw = -0.1 pitch = -0.15
        self.assertEqual(len(outs[4].holes), 1)
        self.assertEqual(len(outs[4].hazmats), 1)
        self.assertEqual(len(outs[4].qrs), 1)
        self.assertEqual(len(outs[4].tpas), 1)

    def test_kalman_filter_of_one_object(self):
        
        self.flush_lists()
        self.deliveryBoy.clearOrderList()
        self.deliveryBoy.getOrderListFromBoss('orders/one_kalman_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)
            try:
                outs.append(self.get_objects())
                # The order had only holes in it!
                self.assertEqual(len(outs[-1].holes), 1)
                self.assertEqual(len(outs[-1].hazmats), 0)
                self.assertEqual(len(outs[-1].qrs), 0)
                self.assertEqual(len(outs[-1].tpas), 0)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        position0 = outs[0].holes[0].pose.position
        position1 = outs[1].holes[0].pose.position
        position4 = outs[4].holes[0].pose.position
        position5 = outs[5].holes[0].pose.position
        position6 = outs[6].holes[0].pose.position
        
        # If measurement does not differ from the expected position, the updated
        # expected position will not change.
        self.assertEqual(position0.x, position1.x)
        self.assertEqual(position0.y, position1.y)
        self.assertEqual(position0.z, position1.z)
        # I expect that 3-4 alerts would bring the conviction high enough to be
        # recognised as a victim, but no less.
        self.assertEqual(len(outs[0].victimsToGo), 0)
        self.assertEqual(len(outs[1].victimsToGo), 0)
        for i in range(3, 12):
            if len(outs[i-1].victimsToGo) == 1:
                break
        rospy.logdebug("Victim found in %s-th alert!", str(i))
        self.assertTrue(i == 3 or i == 4)
        # If the measurement differs the same way, then the updated expected position
        # should draw closer to that measurement - towards the same direction.  That
        # means that expected position's distance from the initial expected position
        # should be greater as the different measurement insists.
        self.assertLess(distance(position0, position4), distance(position0, position5))
        #dir4 = direction(position0, position4)
        #dir5 = direction(position0, position5)
        #self.assertEqual(dir4.x, dir5.x)
        #self.assertEqual(dir4.y, dir5.y)
        #self.assertEqual(dir4.z, dir5.z)

        self.assertLess(distance(position0, position6), distance(position0, position5))

        # We consider that conviction of the object about its position is lot higher now.
        # I assume that its expected position will be around position0.  So:
        self.assertLess(distance(position0, outs[10].holes[0].pose.position), 0.05)

    def test_robustness_of_conviction(self):

        pass

        




if __name__ == '__main__':
  rospy.sleep(1)
  rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
  AlertHandlerStaticTest.setUpClass()
  rostest.rosrun(PKG, NAME, AlertHandlerStaticTest, sys.argv)


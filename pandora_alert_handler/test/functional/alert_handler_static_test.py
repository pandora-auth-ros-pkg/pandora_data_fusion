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
    norm = distance(a, b)
    dire.x = (b.x - a.x)/norm
    dire.y = (b.y - a.y)/norm
    dire.z = (b.z - a.z)/norm
    return dire

class AlertHandlerStaticTest(unittest.TestCase):
     
    deliveryBoy = alert_delivery.AlertDeliveryBoy()

    @classmethod
    def connect(cls):

        cls.get_objects = rospy.ServiceProxy('/data_fusion/get_objects', GetObjectsSrv, True)
        rospy.wait_for_service('/data_fusion/get_objects')
        cls.flush_lists = rospy.ServiceProxy('/data_fusion/flush_queues', Empty, True)
        rospy.wait_for_service('/data_fusion/flush_queues')
        cls.deliveryBoy.deliverHazmatOrder(0, 0, 1)
        rospy.sleep(0.05)
        cls.flush_lists()
        
    @classmethod
    def disconnect(cls):

        cls.get_objects.close()
        cls.flush_lists.close()

    def setUp(self):

        i = 0
        while(True):
            try:
                self.flush_lists()
                break
            except rospy.ServiceException as exc:
                if (i > 3):
                    raise rospy.ServiceException()
                rospy.logdebug("!< flush_lists service failed >! reconnecting and retrying...")
                i += 1
                self.connect()
        self.deliveryBoy.clearOrderList()
      
    def fillInfo(self, outs):

        i = 0
        while(True):
            try:
                outs.append(self.get_objects())
                break
            except rospy.ServiceException as exc:
                if (i > 3):
                    raise rospy.ServiceException()
                rospy.logdebug("!< get_objects service failed >! reconnecting and retrying...")
                i += 1
                self.connect()

    def test_works(self):

        self.assertEqual(1+1, 2)

    def test_a_simple_alert(self):

        self.deliveryBoy.deliverHazmatOrder(0, 0, 1)
        rospy.sleep(0.1)
        outs = []
        self.fillInfo(outs)
        pose = outs[0].hazmats.pop().pose

        self.assertAlmostEqual(pose.position.x, 0.99525856)
        self.assertAlmostEqual(pose.position.y, 0.52000838)
        self.assertAlmostEqual(pose.position.z, 1)
        self.assertAlmostEqual(pose.orientation.x, 0)
        self.assertAlmostEqual(pose.orientation.y, 0)
        self.assertAlmostEqual(pose.orientation.z, 0.70710679)
        self.assertAlmostEqual(pose.orientation.w, 0.70710679)

    def test_objects_coexist(self):

        self.deliveryBoy.getOrderListFromBoss('orders/mixed_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)
            self.fillInfo(outs)
        
        self.assertEqual(len(outs[0].holes), 1)
        self.assertEqual(len(outs[0].hazmats), 0)
        self.assertEqual(len(outs[0].qrs), 0)
        self.assertEqual(len(outs[0].tpas), 0)
        self.assertEqual(len(outs[1].holes), 1)
        self.assertEqual(len(outs[1].hazmats), 0)
        self.assertEqual(len(outs[1].qrs), 1)
        self.assertEqual(len(outs[1].tpas), 0)
        self.assertEqual(len(outs[2].holes), 1)
        self.assertEqual(len(outs[2].hazmats), 1)
        self.assertEqual(len(outs[2].qrs), 1)
        self.assertEqual(len(outs[2].tpas), 0)
        self.assertEqual(len(outs[3].holes), 1)
        self.assertEqual(len(outs[3].hazmats), 1)
        self.assertEqual(len(outs[3].qrs), 1)
        self.assertEqual(len(outs[3].tpas), 0)
        self.assertEqual(len(outs[4].holes), 1)
        self.assertEqual(len(outs[4].hazmats), 1)
        self.assertEqual(len(outs[4].qrs), 1)
        self.assertEqual(len(outs[4].tpas), 1)

    def test_kalman_filter_of_one_object(self):
        
        self.deliveryBoy.getOrderListFromBoss('orders/one_kalman_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)
            self.fillInfo(outs)
            # The order had only holes in it!
            self.assertEqual(len(outs[-1].holes), 1)
            self.assertEqual(len(outs[-1].hazmats), 0)
            self.assertEqual(len(outs[-1].qrs), 0)
            self.assertEqual(len(outs[-1].tpas), 0)
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
        dir4 = direction(position0, position4)
        dir5 = direction(position0, position5)
        self.assertAlmostEqual(dir4.x, dir5.x)
        self.assertAlmostEqual(dir4.y, dir5.y)
        self.assertAlmostEqual(dir4.z, dir5.z)

        self.assertLess(distance(position0, position6), distance(position0, position5))

        # We consider that conviction of the object about its position is lot higher now.
        # I assume that its expected position will be around position0.  So:
        self.assertLess(distance(position0, outs[10].holes[0].pose.position), 0.05)

    def test_robustness_of_conviction(self):

        self.deliveryBoy.getOrderListFromBoss('orders/frequent_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)
            self.fillInfo(outs)
            # The order had only holes in it!
            rospy.logdebug("That is the %d-th alert.", len(outs))
            self.assertEqual(len(outs[-1].holes), 1)
            self.assertEqual(len(outs[-1].hazmats), 0)
            self.assertEqual(len(outs[-1].qrs), 0)
            self.assertEqual(len(outs[-1].tpas), 0)
            if len(outs) > 1:
                self.assertEqual(distance(outs[-1].holes[0].pose.position, 
                  outs[-2].holes[0].pose.position), 0)
        position0 = outs[0].holes[0].pose.position

        # A measurement off will not throw away very much a stable object.
        self.deliveryBoy.deliverHoleOrder(0.13, 0, 1)
        self.fillInfo(outs)
        position1 = outs[-1].holes[0].pose.position
        distanceLessConviction = distance(position0, position1)
        self.assertLess(distanceLessConviction, 0.03)
        
        self.setUp()
        self.deliveryBoy.getOrderListFromBoss('orders/frequent_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)

        # That measurement off will throw away the object even less, if more
        # stable measurements have occured.
        self.deliveryBoy.deliverHoleOrder(0, 0, 1)
        self.fillInfo(outs)
        self.deliveryBoy.deliverHoleOrder(0.13, 0, 1)
        self.fillInfo(outs)
        position0 = outs[0].holes[0].pose.position
        position1 = outs[1].holes[0].pose.position
        self.assertLess(distance(position0, position1), distanceLessConviction)
        

    def test_2_objects_colliding(self):

        pass

    def test_kalman_resistance_to_gaussian(self):

        pass

if __name__ == '__main__':

    rospy.sleep(1)
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    AlertHandlerStaticTest.connect()
    rostest.rosrun(PKG, NAME, AlertHandlerStaticTest, sys.argv)
    AlertHandlerStaticTest.disconnect()


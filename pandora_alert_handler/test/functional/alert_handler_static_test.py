# Copyright <alert_handler_static_test.py>

PKG = 'pandora_alert_handler'
NAME = 'alert_handler_functional_static_test'

import sys, unittest
import rospy, rostest

class TestAlertHandlerStatic(unittest.TestCase):

  def test_works(self):
    self.assertEqual(1+1, 2)

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, TestAlertHandlerStatic, sys.argv)


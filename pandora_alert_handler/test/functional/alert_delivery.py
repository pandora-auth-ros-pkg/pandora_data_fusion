#!usr/bin/env python

from vision_communications.msg import HoleDirectionMsg
from vision_communications.msg import HolesDirectionsVectorMsg
from vision_communications.msg import HazmatAlertMsg
from vision_communications.msg import HazmatAlertsVectorMsg
from vision_communications.msg import QRAlertsVectorMsg
from vision_communications.msg import QRAlertMsg

class AlertDeliveryBoy:

    def __init__(self, frame_id):

        orderWaitingList = []
        
        self.hazmat_msg = HazmatAlertsVectorMsg()
        self.hazmat_msg.header.frame_id = frame_id
        
        self.qr_msg = QRAlertsVectorMsg()
        self.qr_msg.header.frame_id = frame_id
        
        self.hole_msg = HolesDirectionsVectorMsg()
        self.hole_msg.header.frame_id = frame_id
        
        self.hazmatDeliveryAddress = '/vision/hazmat_alert'
        self.hazmat_pub = rospy.Publisher(self.hazmatDeliveryAddress, 
                                          HazmatAlertsVectorMsg)
        self.qrDeliveryAddress = '/vision/qr_alert'
        self.qr_pub = rospy.Publisher(self.qrDeliveryAddress, 
                                      QRAlertsVectorMsg)
        self.holeDeliveryAddress = '/data_fusion/victim_fusion/'+
                                     'hole_direction_alert'
        self.hole_pub = rospy.Publisher(self.holeDeliveryAddress, 
                                        HolesDirectionsVectorMsg)

    def deliverHoleOrder(self, orderYaw, 
                        orderPitch, orderId, orderProbability = 1):
        self.hole_msg.holesDirections = []
        self.hole_msg.holesDirections.append(HoleDirectionMsg(yaw = orderYaw,
                                          pitch = orderPitch,
                                          probability = orderProbability,
                                          holeId = orderId))
        self.hole_pub.publish(self.hole_msg)

    def deliverHazmatOrder(self, orderYaw, 
                          orderPitch, orderPattern):
        self.hazmat_msg.hazmatAlerts = []
        self.hazmat_msg.hazmatAlerts.append(HazmatAlertMsg(yaw = orderYaw,
                                          pitch = holePitch,
                                          patternType = orderPattern))
        self.hazmat_pub.publish(self.hazmat_msg)

    def deliverQrOrder(self, orderYaw, 
                      orderPitch, orderContent):
        self.qr_msg.qrAlerts = []
        self.qr_msg.qrAlerts.append(QRAlertMsg(yaw = orderYaw,
                                          pitch = holePitch,
                                          QRcontent = orderContent))
        self.qr_pub.publish(self.qr_msg)

    def clearOrderList(self):
        self.orderWaitingList = []

    def getOrderList(self, 

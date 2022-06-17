#!/usr/bin/env python
import os
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import message_filters
import numpy as np
import copy

class Node:

    def __init__(self):

        rospy.init_node('test_projection')        

        sub1 = message_filters.Subscriber('/image', Image)
        # sub1 = message_filters.Subscriber('/image', CompressedImage)
        sub2 = message_filters.Subscriber('/depth', Image)
        self.i = 0
        self.image = None

        ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2], 1, 0.5) 
        ts.registerCallback(self.callback)
        print('node')
        rate = rospy.Rate(60)
        while True:
            if not self.image is None:
                cv2.imshow('image', self.image)
                cv2.waitKey(3)
                rate.sleep()         

    def callback(self,*args):
        cv_bridge = CvBridge()
        try:
            image = cv_bridge.imgmsg_to_cv2(args[0], desired_encoding=args[0].encoding)
            # image = cv_bridge.compressed_imgmsg_to_cv2(args[0], desired_encoding='passthrough')
            depth = cv_bridge.imgmsg_to_cv2(args[1], desired_encoding=args[1].encoding)
            depth = np.array(depth,dtype=np.uint16)
            depth[depth > 65535] = 65535

            depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)

            self.i += 1
            mask = np.dstack((depth,depth,depth))

            b=0.75
            cv2.addWeighted(mask, b, image, 1-b, 0, image)

            self.image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)   
        except Exception as e:
            print(e)          


if __name__ == '__main__':
    Node() 

    
       

#!/usr/bin/env python3
import rospy
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
import os
import cv2
from cv_bridge import CvBridge
import time
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

class Node:

    def __init__(self):

        rospy.init_node('images_publisher', anonymous=True)

        self.fdir = rospy.get_param('~save_directory',default='/tmp')
        self.frame_rate = rospy.get_param('~frame_rate',default=1) 
        self.compressed = rospy.get_param('~compressed',default=False) 
        self.step = rospy.get_param('~step',default=1) 

        self.camera_frame = rospy.get_param('~camera_frame') 
        self.map_frame = rospy.get_param('~map_frame',default='map')         

        self.br = tf2_ros.TransformBroadcaster()

        if self.compressed:
            pub = rospy.Publisher('/image', CompressedImage, queue_size=1)
        else:
            pub = rospy.Publisher('/image', Image, queue_size=1)            

        self.images_dir = os.path.join(self.fdir, 'rgb')
        self.poses_fname = os.path.join(self.fdir, 'poses.csv')
        self.images_files = os.listdir(self.images_dir)

        try:
            self.poses = np.loadtxt(self.poses_fname, delimiter=',')
        except Exception as e:
            print(e)
            return

        cv_bridge = CvBridge()

        for i in range(0,len(self.images_files),self.step):
            time.sleep(1/float(self.frame_rate))
            stamp = rospy.Time.now()
            p = self.poses[i,1:4]
            q = self.poses[i,4:]
            self.send_tf(self.map_frame,self.camera_frame,p,q,stamp)
            I = cv2.imread(os.path.join(self.images_dir,'{}.png'.format(i+1)))
            if self.compressed:
                msg = cv_bridge.cv2_to_compressed_imgmsg(I)
            else:
                msg = cv_bridge.cv2_to_imgmsg(I)                    
            msg.header.stamp = stamp
            msg.header.seq = i
            pub.publish(msg)

        rospy.spin()

    def send_tf(self,frame1,frame2,t,q,stamp):

        transformStamped = TransformStamped()
        transformStamped.header.stamp = stamp
        transformStamped.header.frame_id = frame1
        transformStamped.child_frame_id = frame2

        transformStamped.transform.translation.x = t[0]
        transformStamped.transform.translation.y = t[1]
        transformStamped.transform.translation.z = t[2]

        transformStamped.transform.rotation.x = q[0]
        transformStamped.transform.rotation.y = q[1]
        transformStamped.transform.rotation.z = q[2]
        transformStamped.transform.rotation.w = q[3]

        self.br.sendTransform(transformStamped)   

if __name__ == '__main__':
    Node()

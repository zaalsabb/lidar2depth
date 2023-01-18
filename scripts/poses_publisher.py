#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from lidar2depth.srv import GetPoses, GetPosesResponse

class Node:

    def __init__(self):

        rospy.init_node('poses_publisher', anonymous=True)

        f_poses = rospy.get_param('~poses',default=None)
        f_timestamps = rospy.get_param('~timestamps',default=None)

        try:
            self.poses = np.loadtxt(f_poses, delimiter=',')
            self.timestamps = np.loadtxt(f_timestamps)
        except Exception as e:
            print(e)
            return
        
        rospy.Service('get_poses', GetPoses, self.handle_req)
        rospy.spin()

    def handle_req(self,req):
        res = GetPosesResponse()
        for i in range(len(self.poses)):            
            t = self.timestamps[i]
            stamp = rospy.Time.from_sec(t)
            p = self.poses[i,1:4]
            q = self.poses[i,4:]
            msg = PoseStamped()
            msg.pose.position.x = p[0]
            msg.pose.position.y = p[1]             
            msg.pose.position.z = p[2]  
            msg.pose.orientation.x = q[0]             
            msg.pose.orientation.y = q[1]             
            msg.pose.orientation.z = q[2]             
            msg.pose.orientation.w = q[3]             
            msg.header.stamp = stamp
            msg.header.seq = i+1
            res.poses.append(msg)
        return res

if __name__ == '__main__':
    Node()

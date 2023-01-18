#!/usr/bin/env python
import rospy
from std_msgs.msg import Time
import numpy as np
from lidar2depth.srv import GetStamps, GetStampsResponse

class Node:

    def __init__(self):

        rospy.init_node('timestamps_publisher')

        f_timestamps = rospy.get_param('~timestamps',default=None)

        try:
            self.timestamps = np.loadtxt(f_timestamps)
        except Exception as e:
            print(e)
            return
        
        rospy.Service('get_timestamps', GetStamps, self.handle_req)
        rospy.spin()

    def handle_req(self,req):
        res = GetStampsResponse()
        for i in range(len(self.timestamps)):            
            t = self.timestamps[i]
            msg = Time()
            msg.data = rospy.Time.from_sec(t)
            res.timestamps.append(msg)
        return res

if __name__ == '__main__':
    Node()

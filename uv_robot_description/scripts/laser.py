#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def laser_callback(msg):
    
    regions = [
        min(min(msg.ranges[0:78]),20),
        min(min(msg.ranges[102:180]),20),
        min(min(msg.ranges[182:258]),20),
        min(min(msg.ranges[282:360]),20),
        min(min(msg.ranges[362:539]),20),
        min(min(msg.ranges[540:718]),20)
    ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('rplidar_scan')
    sub = rospy.Subscriber('/uvrobot/laser/scan', LaserScan, laser_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
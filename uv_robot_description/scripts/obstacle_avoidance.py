#! /usr/bin/env python

from __future__ import print_function
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

publ = None
pubr = None
pubb = None

moveBindings = {
        'i':(-1,0,1),
        'o':(-1,1,0),
        'j':(1,1,1),
        'l':(-1,-1,-1),
        'u':(-1,0,1),
        ',':(1,0,-1),
        '.':(0,1,-1),
        'm':(1,-1,0),  
        'O':(-1,1,0),
        'I':(-1,0,1),
        'J':(1,-2,1),
        'L':(-1,2,-1),
        'U':(0,-1,1),
        '<':(1,0,-1),
        '>':(0,1,-1),
        'M':(1,-1,0),  
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
    }


def laser_callback(msg):
    regions = {
        'r':    min(min(msg.ranges[0:78]), 20),
        'rf':   min(min(msg.ranges[102:180]), 20),
        'lf':   min(min(msg.ranges[182:258]), 20),
        'l':    min(min(msg.ranges[282:360]), 20),
        'lb':   min(min(msg.ranges[362:539]), 20),
        'rb':   min(min(msg.ranges[540:718]), 20)
    }
    take_action(regions)

def take_action(regions):

    speed = 4.0
    x = 0
    y = 0
    z = 0
    status = 0
    d = 1

    if regions['l'] > d and regions['lf'] > d and regions['rf'] > d and regions['r'] > d:
        state_description = 'Case 1 - Empty'
        key = 'i'
    elif regions['l'] > d and regions['lf'] > d and regions['rf'] > d and regions['r'] < d:
        state_description = 'Case 2 - R'
        key = 'i'
    elif regions['l'] > d and regions['lf'] > d and regions['rf'] < d and regions['r'] > d:
        state_description = 'Case 3 - RF'
        key = 'j'
    elif regions['l'] > d and regions['lf'] > d and regions['rf'] < d and regions['r'] < d:
        state_description = 'Case 4 - R RF'
        key = 'j'
    elif regions['l'] > d and regions['lf'] < d and regions['rf'] > d and regions['r'] > d:
        state_description = 'Case 5 - LF'
        key = 'l'
    elif regions['l'] > d and regions['lf'] < d and regions['rf'] > d and regions['r'] < d:
        state_description = 'Case 6 - LF R'
        key = 'l'
    elif regions['l'] > d and regions['lf'] < d and regions['rf'] < d and regions['r'] > d:
        state_description = 'Case 7 - LF RF'
        key = 'l'
    elif regions['l'] > d and regions['lf'] < d and regions['rf'] < d and regions['r'] < d:
        state_description = 'Case 8 - LF RF R'
        key = 'j'
    elif regions['l'] < d and regions['lf'] > d and regions['rf'] > d and regions['r'] > d:
        state_description = 'Case 9 - L'
        key = 'i'
    elif regions['l'] < d and regions['lf'] > d and regions['rf'] > d and regions['r'] < d:
        state_description = 'Case 10 - L R'
        key = 'i'
    elif regions['l'] < d and regions['lf'] > d and regions['rf'] < d and regions['r'] < d:
        state_description = 'Case 11 - L RF'
        key = 'j'
    elif regions['l'] < d and regions['lf'] > d and regions['rf'] < d and regions['r'] > d:
        state_description = 'Case 12 - L RF R'
        key = 'j'
    elif regions['l'] < d and regions['lf'] < d and regions['rf'] > d and regions['r'] < d:
        state_description = 'Case 13 - L LF'
        key = 'l'
    elif regions['l'] < d and regions['lf'] < d and regions['rf'] > d and regions['r'] > d:
        state_description = 'Case 14 - L LF R'
        key = 'l'
    elif regions['l'] < d and regions['lf'] < d and regions['rf'] < d and regions['r'] < d:
        state_description = 'Case 15 - L LF RF'
        key = 'l'
    else:
        if regions['lb'] > d and regions['rb'] > d:
            state_description = 'Case 16.1 - Back Clear'
            key = 'l'
        elif regions['lb'] > d and regions['rb'] < d:
            state_description = 'Case 16.1 - RB'
            key = 'j'
        elif regions['lb'] < d and regions['rb'] > d:
            state_description = 'Case 16.1 - LB'
            key = 'l'
        else:
            state_description = "Stuck"
            key = 'j'
    
    rospy.loginfo(state_description)

    if key in moveBindings.keys():
        x = moveBindings[key][0]
        y = moveBindings[key][1]
        z = moveBindings[key][2]

    else:
        x = 0
        y = 0
        z = 0
    
    vell = Float64()
    velb = Float64()
    velr = Float64()

    vell = x*speed
    velb = y*speed
    velr = z*speed
    
    publ.publish(vell)
    pubb.publish(velb)
    pubr.publish(velr)  



def main():

    global publ, pubr, pubb
    rospy.init_node('vel_Publisher')

    publ = rospy.Publisher('/fluoresce/left_joint_velocity_controller/command', Float64, queue_size=1)
    pubb = rospy.Publisher('/fluoresce/back_joint_velocity_controller/command', Float64, queue_size=1)
    pubr = rospy.Publisher('/fluoresce/right_joint_velocity_controller/command', Float64, queue_size=1)

    sub = rospy.Subscriber('/uvrobot/laser/scan', LaserScan, laser_callback)
    rospy.spin()

if __name__ == '__main__':

    main()
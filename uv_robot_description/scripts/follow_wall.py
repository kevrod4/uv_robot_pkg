#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_msgs.msg import Float64

import math

regions_ = {
    'r': 0,
    'rf': 0,
    'lf': 0,
    'l': 0,
    'lb': 0,
    'rb': 0
}

state_ = 0
found_wall_ = 0
state_dict_ = {
    0:'find the wall',
    1:'turn right',
    2:'follow the wall'
}

def laser_callback(msg):
    global regions_
    regions_ = {
        'r':    min(min(msg.ranges[0:78]), 20),
        'rf':   min(min(msg.ranges[102:180]), 20),
        'lf':   min(min(msg.ranges[182:258]), 20),
        'l':    min(min(msg.ranges[282:360]), 20),
        'lb':   min(min(msg.ranges[362:539]), 20),
        'rb':   min(min(msg.ranges[540:718]), 20)
    }
    take_action()

def change_state(state):
    global state_, state_dict_, found_wall_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():

    global regions_
    regions = regions_
    d = 1
    state_description = ''

    if regions['l'] > d and regions['lf'] > d and regions['rf'] > d and regions['r'] > d:
        state_description = 'Case 1 - Empty'
        change_state(0)
    elif regions['l'] > d and regions['lf'] > d and regions['rf'] > d and regions['r'] < d:
        state_description = 'Case 2 - R'
        change_state(0)
    elif regions['l'] > d and regions['lf'] > d and regions['rf'] < d and regions['r'] > d:
        state_description = 'Case 3 - RF'
        change_state(1)
    elif regions['l'] > d and regions['lf'] > d and regions['rf'] < d and regions['r'] < d:
        state_description = 'Case 4 - R RF'
        change_state(1)
    elif regions['l'] > d and regions['lf'] < d and regions['rf'] > d and regions['r'] > d:
        state_description = 'Case 5 - LF'
        change_state(1)
    elif regions['l'] > d and regions['lf'] < d and regions['rf'] > d and regions['r'] < d:
        state_description = 'Case 6 - LF R'
        change_state(1)
    elif regions['l'] > d and regions['lf'] < d and regions['rf'] < d and regions['r'] > d:
        state_description = 'Case 7 - LF RF'
        change_state(1)
    elif regions['l'] > d and regions['lf'] < d and regions['rf'] < d and regions['r'] < d:
        state_description = 'Case 8 - LF RF R'
        change_state(1)
    elif regions['l'] < d and regions['lf'] > d and regions['rf'] > d and regions['r'] > d:
        state_description = 'Case 9 - L'
        change_state(2)
    elif regions['l'] < d and regions['lf'] > d and regions['rf'] > d and regions['r'] < d:
        state_description = 'Case 10 - L R'
        change_state(2)
    elif regions['l'] < d and regions['lf'] > d and regions['rf'] < d and regions['r'] < d:
        state_description = 'Case 11 - L RF'
        change_state(1)
    elif regions['l'] < d and regions['lf'] > d and regions['rf'] < d and regions['r'] > d:
        state_description = 'Case 12 - L RF R'
        change_state(1)
    elif regions['l'] < d and regions['lf'] < d and regions['rf'] > d and regions['r'] < d:
        state_description = 'Case 13 - L LF'
        change_state(1)
    elif regions['l'] < d and regions['lf'] < d and regions['rf'] > d and regions['r'] > d:
        state_description = 'Case 14 - L LF R'
        change_state(1)
    elif regions['l'] < d and regions['lf'] < d and regions['rf'] < d and regions['r'] < d:
        state_description = 'Case 15 - L LF RF'
        change_state(1)
    else:
        state_description = 'Unknown State!'
    
def find_wall():
    global publ_, pubr_, pubb_, found_wall_

    vell = Float64()
    velb = Float64()
    velr = Float64()

    if found_wall_ == 0:  
        vell = -4
        velb = 0
        velr = 4
    else:   
        vell = 4
        velb = 4
        velr = 4

    publ_.publish(vell)
    pubb_.publish(velb)
    pubr_.publish(velr)
    

def turn_right():
    global publ_, pubr_, pubb_

    vell = Float64()
    velb = Float64()
    velr = Float64()

    vell = -4
    velb = -4
    velr = -4

    publ_.publish(vell)
    pubb_.publish(velb)
    pubr_.publish(velr)

def follow_the_wall():
    global publ_, pubr_, pubb_, found_wall_

    vell = Float64()
    velb = Float64()
    velr = Float64()
    found_wall_ = 1

    vell = -4
    velb = 0
    velr = 4

    publ_.publish(vell)
    pubb_.publish(velb)
    pubr_.publish(velr)

def main():

    global publ_, pubr_, pubb_
    rospy.init_node('wall_follow')

    publ_ = rospy.Publisher('/fluoresce/left_joint_velocity_controller/command', Float64, queue_size=1)
    pubb_ = rospy.Publisher('/fluoresce/back_joint_velocity_controller/command', Float64, queue_size=1)
    pubr_ = rospy.Publisher('/fluoresce/right_joint_velocity_controller/command', Float64, queue_size=1)

    sub = rospy.Subscriber('/uvrobot/laser/scan', LaserScan, laser_callback)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if state_ == 0:
            find_wall()
        elif state_ == 1:
            turn_right()
        elif state_ == 2:
            follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown State!')

        rate.sleep()

if __name__ == '__main__':
    main()

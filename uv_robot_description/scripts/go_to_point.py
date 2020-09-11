#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_msgs.msg import Float64

import math

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 1
desired_position_.y = 2
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.1

# publishers
publ = None
pubr = None
pubb = None

def fix_yaw(des_pos):
    global yaw_, pubb, pubr, publ, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    
    vell = Float64()
    velb = Float64()
    velr = Float64()  

    if math.fabs(err_yaw) > yaw_precision_:
        if err_yaw > 0:
            vell = 4
            velb = 4
            velr = 4
        else:
            vell = -4
            velb = -4
            velr = -4
    
    publ.publish(vell)
    pubb.publish(velb)
    pubr.publish(velr)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)


def go_straight_ahead(des_pos):
    global yaw_, publ, pubr, pubb, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    vell = Float64()
    velb = Float64()
    velr = Float64()
    
    if err_pos > dist_precision_:
        vell = -4
        velb = 0
        velr = 4
        publ.publish(vell)
        pubb.publish(velb)
        pubr.publish(velr)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)

def done():
    vell = Float64()
    velb = Float64()
    velr = Float64()
    vell = 0
    velb = 0
    velr = 0
    publ.publish(vell)
    pubb.publish(velb)
    pubr.publish(velr)

def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2] + 1.5708



def main():
    global publ, pubb, pubr
    
    rospy.init_node('go_to_point')
    
    publ = rospy.Publisher('/fluoresce/left_joint_velocity_controller/command', Float64, queue_size=1)
    pubb = rospy.Publisher('/fluoresce/back_joint_velocity_controller/command', Float64, queue_size=1)
    pubr = rospy.Publisher('/fluoresce/right_joint_velocity_controller/command', Float64, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if state_ == 0:
            fix_yaw(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            done()
            pass
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()

if __name__ == '__main__':
    main()
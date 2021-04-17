#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from math import atan2
from sensor_msgs.msg import LaserScan
import sys

x = 0.0
y = 0.0
theta = 0.0


def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    pub.publish(velocity_msg)

def go_straight():
    # get distance and linear velocity from command line
    global g1_y
    global g1_x
    global velocity_msg
    distance_to_drive = math.sqrt( ((g1_x)**2)+((g1_y)**2) )
    velocity_msg.linear.x = ((g1_x)**2)+((g1_y)**2)
    t_0 = rospy.Time.now().to_sec()

    distance_moved = 0.0

    while distance_moved < distance_to_drive:
        rospy.loginfo("TurtleBot is moving")
        pub.publish(velocity_msg)
        r.sleep()

        # time in sec in the loop
        t_1 = rospy.Time.now().to_sec()

        distance_moved = (t_1 - t_0) * abs(velocity_msg.linear.x)
        rospy.loginfo("distance moved: {d}".format(d=distance_moved))

    rospy.logwarn("Distance reached")
    velocity_msg.linear.x = 0.0
    pub.publish(velocity_msg)

def rotate():
    relative_angle_degree = 0
    velocity_msg.angular.z = theta
    rotated_angle = 0.0

    t0 = rospy.Time.now().to_sec()
    while True:
        rospy.loginfo("TurtleBot is rotating")
        pub.publish(velocity_msg)
        r.sleep()
        t1 = rospy.Time.now().to_sec()
        rospy.loginfo("t0: {t}".format(t=t0))
        rospy.loginfo("t1: {t}".format(t=t1))
        current_angle_degree = (t1 - t0) * theta

        rospy.loginfo("current angle: {a}".format(a=current_angle_degree))
        rospy.loginfo("angle to reach: {a}".format(a=relative_angle_degree))
        if abs(current_angle_degree) >= math.radians(abs(relative_angle_degree)):
            rospy.loginfo("reached")
            break
    # finally, stop the robot when the distance is moved
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

def get_goal():
    """Get goal arguments from the command line"""
    rospy.loginfo("sys.argv: {a}".format(a=len(sys.argv)))
    x = 0
    y = 0
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    return x, y

def sensor_callback(msg):
    thr1 = 0.5
    thr2 = 0.5


    front = msg.ranges[0]
    left = msg.ranges[15]
    right = msg.ranges[345]

    rospy.loginfo("Distance from obstacle (front): {f}".format(f=front))
    rospy.loginfo("Distance from obstacle (left): {l}".format(l=left))
    rospy.loginfo("Distance from obstacle (right): {r}".format(r=right))

    if front > thr1 and left >thr2 and right > thr2:
        velocity_msg.linear.x = 0.2 # go forward (linear velocity)
        velocity_msg.angular.z = 0.0 # do not rotate (angular velocity)
    else:
        velocity_msg.linear.x = 0.0 # stop
        velocity_msg.angular.z = -0.2 # rotate clockwise
        if front >thr1 and left > thr2 and right > thr2:
            velocity_msg.linear.x = 0.2
            velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)



def read_scan():
    rospy.Subscriber("scan", LaserScan, sensor_callback)
    rospy.spin()


rospy.init_node("move_to_goal")
sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
r = rospy.Rate(4)
velocity_msg = Twist()

goal = Point()
goal.x, goal.y = get_goal()

while not rospy.is_shutdown():
    global velocity_msg

    g1_x = goal.x - x
    g1_y = goal.y - y
    angle_to_goal = atan2(g1_y, g1_x)

    while read_scan():
        if (angle_to_goal - theta) > 0.1:
            rotate()
            go_straight()
        else:
            go_straight()
            rotate()


    pub.publish(velocity_msg)
    r.sleep()


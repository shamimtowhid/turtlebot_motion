#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose 
import random
import time
import math

position = Pose()
velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

def move(speed, distance, isforward):
    rate = rospy.Rate(10) # 5 messages per second
    msg = Twist()
    if isforward:
        msg.linear.x = abs(speed)
    else:
        msg.linear.x = -abs(speed)

    curr_distance = 0
    t0 = time.time() # time in second

    while True:
        velocity_publisher.publish(msg)
        t1 = time.time()
        curr_distance = speed * (t1-t0)
        if curr_distance > distance:
            break
        rate.sleep()
    msg.linear.x = 0.0
    velocity_publisher.publish(msg)

def callback(pose):
    global position
    position.x = pose.x
    position.y = pose.y
    position.theta = pose.theta
    position.linear_velocity = pose.linear_velocity
    position.angular_velocity = pose.angular_velocity

def rotate(angular_speed, angle_in_rad, isclockwise):
    rate = rospy.Rate(10)
    msg = Twist()
    if isclockwise:
        msg.angular.z = -abs(angular_speed)
    else:
        msg.angular.z = abs(angular_speed)
    t0 = time.time()
    current_angle = 0.0
    while True:
        velocity_publisher.publish(msg)
        t1 = time.time()
        current_angle = angular_speed * (t1-t0)
        if current_angle >= angle_in_rad:
            break
        rate.sleep()
    msg.angular.z = 0.0
    velocity_publisher.publish(msg)

def degrees_to_radian(angle_in_degree):
    return angle_in_degree * 3.1416 /180.0

def set_desired_orientation(desired_angle_in_radians):
    relative_angle = desired_angle_in_radians - position.theta
    if relative_angle < 0:
        clockwise = True
    else:
        clockwise = False
    rotate(abs(relative_angle), abs(relative_angle), clockwise)

def get_distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)
    
def move_to_goal(goal, tolerance):
    rate = rospy.Rate(10)
    msg = Twist()
    distance = get_distance(goal.x, goal.y, position.x, position.y)
    while True:
        msg.linear.x = 1.5 * (get_distance(goal.x, goal.y, position.x, position.y))
        msg.angular.z = 4 * (math.atan2(goal.y-position.y, goal.x-position.x) - position.theta)
        velocity_publisher.publish(msg)
        distance = get_distance(goal.x, goal.y, position.x, position.y)
        if distance < tolerance:
            break
        rate.sleep()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    velocity_publisher.publish(msg)
    rate.sleep()

def grid_motion():
    rate = rospy.Rate(0.5)
    goal_pos = Pose()
    goal_pos.x = 1.0
    goal_pos.y = 1.0
    goal_pos.theta = 0.0
    move_to_goal(goal_pos, 0.01)
    rate.sleep()

    set_desired_orientation(degrees_to_radian(0))
    rate.sleep()

    move(2, 9, True)
    rotate(degrees_to_radian(30), degrees_to_radian(90), False)
    rate.sleep()

    move(2, 9, True)
    rotate(degrees_to_radian(30), degrees_to_radian(90), False)
    rate.sleep()

    move(2, 1, True)
    rotate(degrees_to_radian(30), degrees_to_radian(90), False)
    rate.sleep()

    move(2, 9, True)
    rotate(degrees_to_radian(30), degrees_to_radian(90), True)
    rate.sleep()

    move(2, 1, True)
    rotate(degrees_to_radian(30), degrees_to_radian(90), True)
    rate.sleep()

    move(2, 9, True)
    rotate(degrees_to_radian(30), degrees_to_radian(90), False)
    rate.sleep()

def spiral_motion():
    rate = rospy.Rate(1)
    
    msg = Twist()
    rk = 0.5
    constant_speed = 4
    while True:
        rk += 0.5
        msg.linear.x = rk
        msg.angular.z = constant_speed
        velocity_publisher.publish(msg)
        if position.x >= 10.5 or position.y >= 10.5:
            break
        rate.sleep()


if __name__ == '__main__':
    try:

        rospy.init_node('motion', anonymous=True)
        rospy.Subscriber("/turtle1/pose", Pose, callback)

        rospy.loginfo('Enter 1 for Grid motion')
        rospy.loginfo('Enter 2 for Spiral motion')
        choice = input()
        if choice == 1:
            grid_motion()
        else:
            spiral_motion()

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Program Terminated')

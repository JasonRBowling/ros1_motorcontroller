#!/usr/bin/env python3

# from http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom and
# from http://andrewjkramer.net/arduino-odometry/
# http://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html
# https://answers.ros.org/question/12903/quaternions-with-python/
# https://gist.github.com/atotto/f275n4f75bedb6ea56e3e0264ec405dcf



from time import sleep
import numpy as np
import rospy
import math
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Twist, Point, Vector3
import tf

pub = None

max_speed = .32  # meters per sec
max_ang_speed = 2.0  # radians per sec
wheel_base = .220  # meters
wheel_diameter = 0.088  # meters
gear_ratio = 56  # gearbox is 1:56 and we want to convert wheel speed back to motor rpm

r_distance = 0.0
l_distance = 0.0
x_position = 0.0
y_position = 0.0
theta = 0.0

r_vel = 0.0
l_vel = 0.0


def motorControl_cb(new_cmd_vel):

    global pub1
    global pub2

    global max_speed
    global max_ang_speed
    global wheel_base
    global wheel_diameter
    global gear_ratio

    if new_cmd_vel.linear.x > max_speed:
        new_cmd_vel.linear.x = max_speed

    elif new_cmd_vel.linear.x < -max_speed:
        new_cmd_vel.linear.x = -max_speed

    if new_cmd_vel.angular.z > max_ang_speed:
        new_cmd_vel.angular.z = max_ang_speed
    elif new_cmd_vel.angular.z < -max_ang_speed:
        new_cmd_vel.angular.z = -max_ang_speed

    data = new_cmd_vel
    # Calculate wheel speeds in m/s
    #reference https://snapcraft.io/blog/your-first-robot-the-driver-4-5
    left_wheel_vel = data.linear.x - ((data.angular.z * wheel_base) / 2.0)
    right_wheel_vel = data.linear.x + ((data.angular.z * wheel_base) / 2.0)


    # convert the required wheel speeds in m/s into rpms
    c = math.pi * wheel_diameter  # wheel circumference in meters
    right_rpm = int((right_wheel_vel / c) * 60.0 * gear_ratio)
    left_rpm = int((left_wheel_vel / c) * 60.0 * gear_ratio)

    #rospy.loginfo("Commanded linear speed: %f", data.linear.x)
    #rospy.loginfo("LW vel: %f, RW vel: %f", left_wheel_vel, right_wheel_vel)
    #rospy.loginfo("LW rpm: %f, RW rpm: %f", left_rpm, right_rpm)
    #print()
    pub1.publish(right_rpm)
    pub2.publish(left_rpm)
    return

###these 20.0s should be changed to use actual time since update)###


def r_tick_cb(msg):
    global wheel_diameter
    global r_distance
    global r_vel

    ticks = msg.data
    r_distance = ((ticks / 32.0) / 56.0) * (math.pi * wheel_diameter)
    r_vel = r_distance * 20.0
    r_rpm_actual = (ticks / 32.0) * 20.0 * 60.0
    #print("r_vel: " + str(r_vel))

def l_tick_cb(msg):
    global wheel_diameter
    global l_distance
    global l_vel

    ticks = msg.data
    l_distance = ((ticks / 32.0) / 56.0) * (math.pi * wheel_diameter)
    l_vel = l_distance * 20.0
    l_rpm_actual = (ticks / 32.0) * 20.0 * 60.0
    #print("l_vel: " + str(l_vel))

    # we have both numbers of ticks - update odometry position
    #if (r_distance != 0.0) or (l_distance != 0.0):
    updateOdom()


def updateOdom():
    # motor controller publishes right ticks, then left, so call this after left tick

    global theta
    global l_distance
    global r_distance
    global wheel_base
    global x_position
    global y_position
    global odom_pub
    global odom_broadcaster

    dCenter = (l_distance + r_distance) / 2.0
    # calculate change in angle
    phi = (r_distance - l_distance) / wheel_base
    theta += phi
    # constrain _theta to the range 0 to 2 pi
    if (theta > 2.0 * math.pi):
        theta -= 2.0 * math.pi

    if (theta < 0.0):
        theta += 2.0 * math.pi

    x_position += dCenter * math.cos(theta)
    y_position += dCenter * math.sin(theta)

    # compute the overall motion of the robot since last update
    vec_v = (r_vel + l_vel) / 2.0
    ang_vel = (r_vel - l_vel) / wheel_base

    # decompose into x, y components of velocity vector
    vx = vec_v * math.cos(ang_vel)

    # calculate angular velocity
    vy = vec_v * math.sin(ang_vel)

    currentTime = rospy.get_rostime()
    #print("Theta: " + str(theta) + " rad\t" + str(math.degrees(theta)) + " deg" )
    # generate quaternion from yaw. Roll and pitch assumed to be zero.
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x_position, y_position, 0.), odom_quat, currentTime, "base_footprint", "odom")

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = currentTime
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(
        Point(x_position, y_position, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, ang_vel))

    #set the covariance. The diagonal cannot be zero.
    #used values and modified code from https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/
    #and https://answers.ros.org/question/64759/covariance-matrix-for-vo-and-odom/
    for i in range(0, 36):
       odom.pose.covariance[i] = 0.0

    odom.pose.covariance[0] = 0.01
    odom.pose.covariance[7] = 0.01
    odom.pose.covariance[14] = 0.01

    odom.pose.covariance[21] = 0.1
    odom.pose.covariance[28] = 0.1
    odom.pose.covariance[35] = 0.1

    # publish the message
    odom_pub.publish(odom)

    #print("Theta: " + str(theta))
    #print("Right Wheel vel: " + str(r_vel))
    #print("Left Wheel vel: " + str(l_vel))
    #print("x vel component: " + str(vx))
    #print("y vel component: " + str(vy))
    #print("ang vel: " + str(ang_vel))
    #print("stamp: " + str(currentTime))
    #print("Quaternion: " + str(odom_quat))
    #print("X position: " + str(x_position))
    #print("Y position: " + str(y_position))

    #print("\n")


def listener():
    global pub1
    global pub2
    global odom_pub
    global odom_broadcaster

    rospy.init_node('baseController', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, motorControl_cb)
    rospy.Subscriber('/rightMotorTicks', Int16, r_tick_cb)
    rospy.Subscriber('/leftMotorTicks', Int16, l_tick_cb)
    pub1 = rospy.Publisher('/rightMotor', Int16, queue_size=10)
    pub2 = rospy.Publisher('/leftMotor', Int16, queue_size=10)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    rospy.spin()

if __name__ == '__main__':
    listener()

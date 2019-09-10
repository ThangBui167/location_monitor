#!/usr/bin/env python
# nav_msgs is name of package and msg is namespace can be found from Odometry
# Odometry is msg class
# LAndmarkDistance la ten file trogn folder msg
import rospy
import math
import random
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from location_monitor.msg import LandmarkDistance
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist
import numpy as np
from math import radians
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np


def check_inside(x, y, maxdist):
    curentdistant = distance(x, y)
    if curentdistant < maxdist:
        return True
    else:
        return False


def planning_avoid(x, y, maxdist):
    current_distant = distance(x, y)
    d_min = maxdist - current_distant

    if d_min < 1:
        return "smaller than 1"

    else:
        return "bigger than 1"


def distance(x, y, ):
    return math.sqrt(x * x + y * y)


def on_velocity_changed(received_robot_speed):
    global robot_speed

    robot_speed = received_robot_speed
    print("Robot speed: ", robot_speed.linear.x, robot_speed.angular.z)


roll = pitch = yaw = 0.0
target = 100.0
kp = 1
count = 0.0
previous_state = ""
robot_speed = Twist()


def callback(msg):
    global roll, pitch, yaw, previous_state, count, robot_speed
    max_distant = 5

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    print("X position: ", x, "Y position: ", y, "distant : ", distance(x, y))
    print(check_inside(x, y, max_distant))
    print(planning_avoid(x, y, max_distant))

    cmd_vel = rospy.get_param('~vel_out_topic', '/cmd_vel_mux/input/teleop')
    pub_speed = rospy.Publisher(cmd_vel, Twist, queue_size=1)

    speed = Twist()

    if check_inside(x, y, max_distant):
        speed.linear.x = robot_speed.linear.x
        speed.angular.z = robot_speed.angular.z
        # if planning_avoid(x, y, arg) == "bigger than 0.5":
        #     count = yaw

        print("Degree Converted: ", yaw)

        if planning_avoid(x, y, max_distant) == "smaller than 1":

            if previous_state == "bigger than 1":
                count = yaw

            target_rad = target * math.pi / 180.0
            dif = yaw - count

            if count < 0.0 < yaw:
                dif = yaw + abs(count)

            if yaw < 0.0 < count:
                dif = 2 * math.pi - count + yaw

            if target_rad - dif > 0.5:
                speed.angular.z = kp * (target_rad - dif)
                speed.linear.x = 0.0

            if target_rad - dif < 0.2:
                speed.linear.x = 0.05
                speed.angular.z = 0.1

            print("Count: ", count)
            print("dif: ", target_rad - dif)
        previous_state = planning_avoid(x, y, max_distant)

    topic = '/markerArray'
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)

    center = [0, 0, 0]
    radius = 5
    points = list_points(center, radius)

    markerArray = MarkerArray()
    for i in points:
        marker = Marker()
        marker.header.frame_id = "odom"  # ten cua frame ID trong Rviz
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        marker.color.a = np.random.random_sample()
        marker.color.r = np.random.random_sample()
        marker.color.g = np.random.random_sample()
        marker.color.b = np.random.random_sample()

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.pose.orientation.w = 1.0
        marker.pose.position.x = i[0]
        marker.pose.position.y = i[1]
        marker.pose.position.z = i[2]

        markerArray.markers.append(marker)

        # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

    # rospy.loginfo("Add markers")  # ko can de dong nay
    publisher.publish(markerArray)
    pub_speed.publish(speed)


def list_points(center, radius):
    # Ham nay de tao ra cac diem vien cua vong tron, ve ca diem vien nay trong rviz de thay cai vong tron
    list_of_points = []
    # Chia vong tron thanh 100 phan nho de ve marker cho de nhin
    angles = np.linspace(0, 2 * math.pi, 100)
    for theta in angles:
        x = center[0] + math.cos(theta) * radius
        y = center[1] + math.sin(theta) * radius
        point = [x, y, 0]
        list_of_points.append(point)

    return list_of_points


def main():
    rospy.init_node('Case1')
    rospy.loginfo('Case1')
    rospy.Subscriber("/vel_in", Twist, on_velocity_changed)
    rospy.Subscriber("odom", Odometry, callback)
    rospy.spin()


if __name__ == '__main__':
    main()

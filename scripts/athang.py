#!/usr/bin/env python

# File nay dung de ve vong tron gioi han cua con turtlebot co the di

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np

topic = 've_vong_tron'  # thay cai nay bang odometry vo
publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)

rospy.init_node('athang')  # cai nay ben kia co r thi ko can de lai

center = [0.0, 0.0, 0.0]  # vi tri ban dau cua robot, cu de yen 0,0,0 khong sao ca
radius = 5  # cai nay la ban kinh vong trong chinh la max distance


def points(center, radius):
    # Ham nay de tao ra cac diem vien cua vong tron, ve ca diem vien nay trong rviz de thay cai vong tron
    list_of_points = []
    # Chia vong tron thanh 100 phan nho de ve marker cho de nhin
    angles = np.linspace(0, 2*math.pi, 100)
    for theta in angles:
        x = center[0] + math.cos(theta)*radius
        y = center[1] + math.sin(theta)*radius
        point = [x, y, 0]
        list_of_points.append(point)

    return list_of_points


rate = rospy.Rate(1)
points = points(center, radius)

while not rospy.is_shutdown():
    markerArray = MarkerArray()
    for i in points:
        marker = Marker()
        marker.header.frame_id = "/map"  # ten cua frame ID trong Rviz
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

    rospy.loginfo("Add markers")  # ko can de dong nay
    publisher.publish(markerArray)
    rate.sleep()

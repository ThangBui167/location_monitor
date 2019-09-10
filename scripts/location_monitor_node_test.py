#!/usr/bin/env python

from shapely.geometry import LineString, Point
import rospy
import math
import random
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from location_monitor.msg import LandmarkDistance
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist
import numpy as np
from math import radians
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def ray_intersect(robot_pos, edge_first_point, edge_second_point):
    # check if 2nd point Y >= 1st point Y
    assert edge_second_point[1] >= edge_first_point[1]

    p_list = list(robot_pos)
    if p_list[2] == edge_first_point[1] or p_list[2] == edge_second_point[1] or p_list[1] == edge_first_point[1] or \
            p_list[1] == edge_second_point[1]:
        # p_list[2] += 0.5
        return False

    # assign variables to be clearer
    rob_pos_x = p_list[1]
    point1_x = edge_first_point[0]
    point2_x = edge_second_point[0]

    rob_pos_y = p_list[2]
    point1_y = edge_first_point[1]
    point2_y = edge_second_point[1]

    if rob_pos_y < point1_y or rob_pos_y > point2_y:
        return False
    elif rob_pos_x >= max(point1_x, point2_x):
        return False
    else:
        if rob_pos_x < min(point1_x, point2_x):
            return True
        else:
            if point1_x != point2_x:
                m_red = (point2_y - point1_y) / (point2_x - point1_x)
            else:
                m_red = float('Inf')

            if point1_x != rob_pos_x:
                m_blue = (rob_pos_y - point1_y) / (rob_pos_x - point1_x)
            else:
                m_blue = float('Inf')

            if m_blue >= m_red:
                return True
            else:
                return False


def make_edges(_landmarks):
    edges = []
    for l in range(len(_landmarks)):
        if l == len(_landmarks) - 1:
            edge = [_landmarks[l], _landmarks[0]]
        else:
            edge = [_landmarks[l], _landmarks[l + 1]]

        try:
            assert edge[0][1] <= edge[1][1]
        except AssertionError:
            temp = edge[0]
            edge[0] = edge[1]
            edge[1] = temp
        finally:
            edges.append(edge)
    assert len(edges) == len(_landmarks)
    return edges


def check_inside(x_r, y_r, _landmarks):
    count = 0
    robot_coor = ("robot", x_r, y_r)
    edges = make_edges(_landmarks)
    for edge in edges:
        if ray_intersect(robot_coor, edge[0], edge[1]):
            count += 1
    if count % 2 == 1:
        return True
    else:
        return False


def planning_avoid(x_r, y_r, _landmarks):
    robot_coor = [x_r, y_r, 0]
    robot_coor = np.asarray(robot_coor)
    _landmarks = [np.asarray(e) for e in _landmarks]

    d_min = 1000
    for l in range(len(_landmarks)):
        if l == len(_landmarks) - 1:
            d = np.linalg.norm(np.cross(_landmarks[0] - _landmarks[l], _landmarks[l] - robot_coor)) / np.linalg.norm(
                _landmarks[0] - _landmarks[l])
        else:

            d = np.linalg.norm(np.cross(_landmarks[l + 1] - _landmarks[l], _landmarks[l] - robot_coor)) / np.linalg.norm(
                _landmarks[l + 1] - _landmarks[l])

        if d < d_min:
            d_min = d
    print(d_min)
    if d_min < 2:
        print("smaller than 2")
    else:
        print("dasdasd")


"""class LandmarkMonitor(object):

    def __init__(self, pub, landmarks):
        self.pub = pub
        self.landmarks = landmarks

    def position_callback(self,msg):
        x = msg.pose.pose.position.x
        y= msg.pose.pose.position.y
        return x, y

    def check_inside(self,x_r, y_r, landmarks):
        count = 0
        robot_coor = ("robot",x_r.position_callback, y_r.position_callback
                      )
        edges = make_edges(landmarks)
        for edge in edges:
            if ray_intersect(robot_coor, edge[0], edge[1]):
                count += 1
        if count % 2 == 1:
            return True
        else:
            return False

"""

roll = pitch = yaw = 0.0
target = 90
kp = 0.5
def callback(msg, arg):
    global roll, pitch, yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # print(arg)
    print(check_inside(x, y, arg))

    cmd_vel = '/cmd_vel_mux/input/teleop'
    pub_speed = rospy.Publisher(cmd_vel, Twist, queue_size=1)

    speed = Twist()

    if check_inside(x,y,arg):
        target_rad = target * math.pi / 180

        # if planning_avoid(x, y, arg) == "bigger than 2":
        speed.linear.x = 0.5
        speed.angular.z = 0.0
        #
        # elif planning_avoid(x, y, arg) == "smaller than 2":
        #     speed.angular.z = kp * (target_rad - yaw)
        #     speed.linear.x = 0.0

        pub_speed.publish(speed)


def main():
    rospy.init_node('location_monitor_node_test')
    rospy.loginfo('Publishing landmarks')
    landmarks = []
    landmarks.append((-2.0, -2.0, 0.0))
    landmarks.append((2.0, -2.0, 0.0))
    landmarks.append((2.0, 1.0, 0.0))
    landmarks.append((1.0, 3.0, 0.0))
    landmarks.append((-2.0, 1.0, 0.0))

    rospy.Subscriber("/odom", Odometry, callback, (landmarks))

    rospy.spin()
    # result = check_inside(0,0, landmarks)
    # print(result)

    # result1 = planning_avoid(3.5, 4, 0.1, 0.5, landmarks)


if __name__ == '__main__':
    main()


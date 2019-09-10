#!/usr/bin/env python

from shapely.geometry import LineString, Point
import rospy
import math
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Float32MultiArray
from move_base_msgs.msg import MoveBaseActionGoal
import numpy as np
from actionlib_msgs.msg import *


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

    # ########
    d_min = 10000
    for l in range(len(_landmarks)):
        if l == len(_landmarks) - 1:
            d = np.linalg.norm(np.cross(_landmarks[0] - _landmarks[l], _landmarks[l] - robot_coor)) / np.linalg.norm(
                _landmarks[0] - _landmarks[l])
        else:

            d = np.linalg.norm(np.cross(_landmarks[l + 1] - _landmarks[l], _landmarks[l] - robot_coor)) / np.linalg.norm(
                _landmarks[l + 1] - _landmarks[l])

        if d < d_min:
            d_min = d
    # print(d_min)
    if d_min < 1:
        print("smaller than 1")
        return "smaller than 1"
    else:
        print("bigger than 1")
        return "bigger than 1"


target = 100.0
kp = 2
count = 0.0
previous_state = ""
landmarks = []
velocity = []
check_ready = False
robot_speed = Twist()
current_goal = None
hasGoal = False


def on_velocity_changed(received_robot_speed):
    global robot_speed
    if check_ready:
        robot_speed = received_robot_speed
        print("Robot speed: ", robot_speed.linear.x, robot_speed.angular.z)


def callback(msg):
    global previous_state, count, kp, check_ready, landmarks, velocity, hasGoal

    if check_ready:

        x = msg.linear.x
        y = msg.linear.y

        yaw = msg.angular.z

        # Transfer

        print("WWWW: ", x, y, yaw)

        print(check_inside(x, y, landmarks))

        cmd_vel = rospy.get_param('~vel_out_topic', '/cmd_vel_mux/input/teleop')
        pub_speed = rospy.Publisher(cmd_vel, Twist, queue_size=1)
        speed = Twist()

        pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

        goal = GoalID()
        goal.id = ''

        if check_inside(x, y, landmarks):
            # speed.linear.x = 0.3
            # speed.angular.z = 0.0

            print("Degree Converted: ", yaw)

            if planning_avoid(x, y, landmarks) == "smaller than 1":

                if previous_state == "bigger than 1":
                    pub_cancel.publish(goal)
                    print('stoped')
                    count = yaw

                target_rad = target * math.pi / 180.0
                dif = yaw - count

                if count < 0.0 < yaw:
                    dif = yaw + abs(count)

                if yaw < 0.0 < count:
                    dif = 2 * math.pi - count + yaw

                if target_rad - dif > 0.3:
                    speed.angular.z = kp * (target_rad - dif)
                    speed.linear.x = 0.0

                if target_rad - dif < 0.3:
                    speed.linear.x = 0.05
                    speed.angular.z = 0.1

                print("Count: ", count)
                print("dif: ", target_rad - dif)
                pub_speed.publish(speed)
            else:
                speed = robot_speed
                pub_speed.publish(speed)

            previous_state = planning_avoid(x, y, landmarks)

    else:
        print("Not ready yet")


def get_parameters(msg):
    global check_ready, landmarks, velocity
    data = msg.data
    count1 = len(data) / 3

    if len(data) != 3 and not landmarks:
        velocity = [data[0], data[1], data[2]]

        for i in range(1, count1):
            landmark = (data[i*3], data[i*3 + 1], data[i*3 + 2])
            landmarks.append(landmark)

        check_ready = True



def main():

    rospy.Subscriber("/parameters", Float32MultiArray, get_parameters)
    rospy.Subscriber("/vel_in", Twist, on_velocity_changed)
    rospy.Subscriber("/map_position", Twist, callback)

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('Case2')
    rospy.loginfo('Running')

    main()


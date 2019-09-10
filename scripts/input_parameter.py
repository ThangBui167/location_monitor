#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import os

landmarks = []
markerArray = MarkerArray()
historyPickedPoints = []
pickedPoints = []
id_temp = 0

def picked_point(msg):
    global landmarks, markerArray, id_temp, pickedPoints, historyPickedPoints
    historyPickedPoints.append(msg)

    x = msg.point.x
    y = msg.point.y
    frame_id = msg.header.frame_id

    landmarks.append(x)
    landmarks.append(y)
    landmarks.append(0)

    if len(pickedPoints) < 1:
        pickedPoints.append(msg)
        return

    prev_point = pickedPoints.pop()

    #add current picked point:
    pickedPoints.append(msg)
    start_point = Point()  # start point
    start_point.x = prev_point.point.x
    start_point.y = prev_point.point.y
    start_point.z = 0

    end_point = Point()  # end point
    end_point.x = x
    end_point.y = y
    end_point.z = 0
    print("Picked point: {0}, {1}, {2}".format(x, y, 0))

    # Visualize in RVIZ
    marker = Marker()
    marker.header.frame_id = frame_id  # ten cua frame ID trong Rviz
    #marker.type = marker.SPHERE
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.points.append(start_point)
    marker.points.append(end_point)

    marker.color.a = 255
    marker.color.r = 255
    marker.color.g = 0
    marker.color.b = 0

    markerArray.markers.append(marker)
    # Renumber the marker IDs
    id_temp = 0
    for m in markerArray.markers:
        m.id = id_temp
        id_temp += 1

    # when we have 4 points marked,
    # we add the last line which take the first point and last point
    if len(historyPickedPoints) == 4:
        last_line_start_point = Point()  # start point
        last_line_start_point.x = historyPickedPoints[3].point.x
        last_line_start_point.y = historyPickedPoints[3].point.y
        last_line_start_point.z = 0

        last_line_end_point = Point()  # end point
        last_line_end_point.x = historyPickedPoints[0].point.x
        last_line_end_point.y = historyPickedPoints[0].point.y
        last_line_end_point.z = 0

        last_marker = Marker()
        last_marker.header.frame_id = frame_id  # ten cua frame ID trong Rviz
        # marker.type = marker.SPHERE
        last_marker.type = marker.LINE_STRIP
        last_marker.action = marker.ADD

        last_marker.scale.x = 0.2
        last_marker.scale.y = 0.2
        last_marker.scale.z = 0.2
        last_marker.points.append(last_line_start_point)
        last_marker.points.append(last_line_end_point)

        last_marker.color.a = 255
        last_marker.color.r = 255
        last_marker.color.g = 0
        last_marker.color.b = 0

        markerArray.markers.append(last_marker)
        # Renumber the marker IDs
        id_temp = 0
        for m in markerArray.markers:
            m.id = id_temp
            id_temp += 1


def talker():
    rospy.init_node('input_parameter', anonymous=True)
    pub1 = rospy.Publisher('parameters', Float32MultiArray, queue_size=10)
    pub2 = rospy.Publisher('map_position', Twist, queue_size=10)
    pub3 = rospy.Publisher('picked_points', MarkerArray, queue_size=100)
    rate = rospy.Rate(10)

    # Subscribe to get clicked point in Rviz
    rospy.Subscriber("/clicked_point", PointStamped, picked_point)

    data = Float32MultiArray()

    listener = tf.TransformListener()

    # Input parameters
    print('=======================')
    # x = float(input('Input x velocity: '))
    # y = float(input('Input y velocity: '))
    # z = float(input('Input z velocity: '))
    # frame_id = float(input('Input 1 for map or 2 for odom : '))
    # polygon_points = float(input('Input numbers of polygon (not less than 3): '))
    x = rospy.get_param('~input_x', 1)
    y = rospy.get_param('~input_y', 0)
    z = rospy.get_param('~input_z', 0)
    frame_id = rospy.get_param('~frame_id', 1)
    polygon_points = rospy.get_param('~polygon_points', 4)
    print("PARAMETERS: ", x, y, z, frame_id, polygon_points)

    # os.system('rosparam set /move_base/DWAPlannerROS/max_vel_x %s' % str(x))
    # os.system('rosparam set /move_base/DWAPlannerROS/max_vel_y %s' % str(y))

    while not rospy.is_shutdown():

        # Handle frame
        if frame_id == 1.0:
            frame = '/map'
        else:
            frame = '/odom'

        try:
            (trans, rot) = listener.lookupTransform(frame, '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        (roll, pitch, yaw) = euler_from_quaternion(rot)

        position = Twist()
        position.linear.x = trans[0]
        position.linear.y = trans[1]
        position.linear.z = trans[2]

        position.angular.x = roll
        position.angular.y = pitch
        position.angular.z = yaw

        # Handle polygon points
        if (len(landmarks) / 3) == polygon_points:
            # print(landmarks)
            data.data = [x, y, z] + landmarks
        else:
            data.data = [x, y, z]

        # Publish data
        pub1.publish(data)
        pub2.publish(position)
        pub3.publish(markerArray)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
#nav_msgs is name of package and msg is namespace can be found from Odometry
#Odometry is msg class
import rospy
import math
from nav_msgs.msg import Odometry
from __future__ import print_function
from shapely.geometry import LineString



landmarks = []
landmarks.append(("Cube", 0.31, 0.99));
landmarks.append(("Dumspter", 0.11, -2.42));
landmarks.append(("Cylinder", -1.14, -2.88));
landmarks.append(("Barrier", -2.59, -0.83));
landmarks.append(("Bookshelf", -0.09, 0.53));


def distance(x1, y1, x2, y2):

	xd = x1 -x2
	yd = y1-y2
	return math.sqrt(xd*xd +yd*yd)



def callback(msg):
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y

		closest_name = None
		closest_distance = None
		for landmark_name, landmark_x, landmark_y in landmarks:
			dist = distance(x, y, landmark_x, landmark_y)#compare robot posetions and landmark
			if closest_distance is None or dist < closest_distance:
				closest_name = landmark_name
				closest_distance = dist
		rospy.loginfo("closest: {}".format(closest_name)) # when robot at near will print name of that that object



def main():

	rospy.init_node('location_monitor')
	rospy.Subscriber("/odom", Odometry, callback)
	rospy.spin()

if __name__ == '__main__':
    main()

#!usr/bin/env python

#Libraries
import rospy, tf, sys, time, math, numpy

#Libraries
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

def readPoint():

#main function
if __name__ == '__main__':
	
	topic = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
	
	rospy.Subscriber('/clicked_point', PoseStamped, readPoint)

#!usr/bin/env python

#import libraries
import rospy, math, tf, time

#import ROS msg classes
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from Queue import PriorityQueue
from geometry_msgs.msg import PoseStamped, Point , PoseWithCovarianceStamped, Pose, Quaternion
from nav_msgs.srv import GetPlan

from pose2d import *
from astar_map import *
from node import *
from astar_planner import *

#CostMap read callBack Function
def readGlobalMap(msg):
	global rosmap
	rosmap = msg

def readLocalMap(msg):
	global localMap
	localMap = msg
	
#Service callBack Function
def planCallBack(msg):
	global rosmap
	#~ global frontierPub
	#~ global exploredPub
	#~ global freePub
	#~ global blockedPub
	#~ global wayPointsPub
	
	print "received request"
	
	navMap = AStarMap(rosmap, msg.start, msg.goal)
	print "created map"
	
	planner = AStarPlanner(AStarMap(rosmap), msg.start, msg.goal)
	
	return planner.wayPoints


#main function
if __name__ == '__main__':
	
	global frontierPub
	global exploredPub
	global freePub
	global blockedPub
	global wayPointsPub
	
	rospy.init_node('astar')
	
	#rospy.Subscriber("/map", OccupancyGrid, readMap)
	rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, readGlobalMap)

	frontierPub = rospy.Publisher("/grid_Frontier", GridCells, queue_size=1)
	exploredPub = rospy.Publisher("/grid_Explored", GridCells, queue_size=1)
	freePub = rospy.Publisher("/grid_Free", GridCells, queue_size=1)
	blockedPub = rospy.Publisher("/grid_Blocked", GridCells, queue_size=1)
	wayPointsPub = rospy.Publisher("/way_points", GridCells, queue_size=5)
    
	rospy.Service("astar_planner", GetPlan, planCallBack)
    
	rospy.sleep(1)
    
	rospy.spin()

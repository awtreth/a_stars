#!usr/bin/env python

#import libraries
import rospy, math, tf, time

#import ROS msg classes
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from Queue import PriorityQueue
from geometry_msgs.msg import PoseStamped, Point , PoseWithCovarianceStamped, Pose, Quaternion
from nav_msgs.srv import GetPlan
from map_msgs.msg import OccupancyGridUpdate


from pose2d import *
from astar_map import *
from node import *
from astar_planner import *

from std_srvs.srv import Empty

def clearCostMap():
	rospy.wait_for_service('/move_base/clear_costmaps')
	clear = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
	
	try:
		clear()
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

#CostMap read callBack Function
def readGlobalMap(msg):
	global rosmap
	rosmap = msg
	#~ print "got global"

def readUpdateMap(msg):
	global updateMap
	global hasUpdate
	global rosmap
	#~ print "got update"
	if msg.width == rosmap.info.width:
		updateMap = msg
		hasUpdate = True
	
#Service callBack Function
def planCallBack(msg):
	global rosmap
	global updateMap
	global hasUpdate
	#~ global frontierPub
	#~ global exploredPub
	#~ global freePub
	#~ global blockedPub
	#~ global wayPointsPub
	
	print "received request"
	hasUpdate = False
	while hasUpdate is False and not rospy.is_shutdown():
		clearCostMap()
		rospy.sleep(1)
	hasUpdate = False
	
	planner = AStarPlanner(AStarMap(rosmap,updateMap), msg.start, msg.goal)
	
	pubGridCells(planner)
	
	return planner.wayPoints

def pubGridCells(planner):
	global frontierPub
	global exploredPub
	global freePub
	global blockedPub
	global wayPointsPub
	global pathPub
	
	print "pub"
	
	blockedPub.publish(planner.map.getGridCell(CellState.BLOCKED))
	freePub.publish(planner.map.getGridCell(CellState.FREE))
	exploredPub.publish(planner.map.getGridCell(CellState.EXPLORED))
	frontierPub.publish(planner.map.getGridCell(CellState.FRONTIER))
	pathPub.publish(planner.getPathGridCells())
	wayPointsPub.publish(planner.getWayPointsGridCells())



#main function
if __name__ == '__main__':
	
	global frontierPub
	global exploredPub
	global freePub
	global blockedPub
	global wayPointsPub
	global pathPub
	
	rospy.init_node('astar')
	
	#rospy.Subscriber("/map", OccupancyGrid, readMap)
	rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, readGlobalMap)
	rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, readUpdateMap)

	frontierPub = rospy.Publisher("/AStarFrontier", GridCells, queue_size=1)
	exploredPub = rospy.Publisher("/AStarExplored", GridCells, queue_size=1)
	freePub = rospy.Publisher("/AStarFree", GridCells, queue_size=1)
	blockedPub = rospy.Publisher("/AStarObstacles", GridCells, queue_size=1)
	wayPointsPub = rospy.Publisher("/AStarWayPoints", GridCells, queue_size=1)
	pathPub = rospy.Publisher("/AStarPath", GridCells, queue_size=1)
    
	rospy.Service("astar_planner", GetPlan, planCallBack)
    
	rospy.sleep(1)
    
	rospy.spin()

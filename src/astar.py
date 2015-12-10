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

#CostMap read callBack Function
def readGlobalMap(msg):
	global rosmap
	rosmap = msg

def readLocalMap(msg):
	global localMap
	localMap = msg

#def mergeMaps(localmap, globalmap):
#	rosmap = OccupancyGrid()
#	
#	min_global_x = localmap.info.origin.position.x-globalmap.info.origin.position.x
#	min_global_y = localmap.info.origin.position.y-globalmap.info.origin.position.y
#	max_global_x = min_global_x+localmap.info.width
#	max_global_y = min_global_y+localmap.info.height


#Service callBack Function
def planCallBack(msg):
	global rosmap
	global frontierPub
	global exploredPub
	global freePub
	global blockedPub
	global wayPointsPub
	
	print "received request"
	
	navMap = AStarMap(rosmap, msg.start, msg.goal)
	print "created map"
	
	# Create the first node
	origin = Node(navMap.startPose)
	
	#PriorityQueue of nodes
	pq = PriorityQueue()
	currentNode = origin
	
	# Compute the AStar search algorithm
	while not currentNode.pos.equals(navMap.goalPose) and not rospy.is_shutdown():
		children = navMap.expand(currentNode)
		
		for node in children:
			pq.put((node.cost(), node)) #it is ranked by the cost
		
		if(pq.qsize() == 0): #no more nodes to be expanded
			print "didn't find a path"
			frontierPub.publish(navMap.getGridCell(CellState.FRONTIER))
			return Path() #return an empty Path
		
		# pop the "closest" node to the goal
		currentNode = pq.get()[1]

		# Publish the data in Rviz environment	
		#frontierPub.publish(navMap.getGridCell(CellState.FRONTIER))
	
	# Publish the data in Rviz environment	
	frontierPub.publish(navMap.getGridCell(CellState.FRONTIER))
	exploredPub.publish(navMap.getGridCell(CellState.EXPLORED))
	freePub.publish(navMap.getGridCell(CellState.FREE))
	blockedPub.publish(navMap.getGridCell(CellState.BLOCKED))
	
	#Get the path
	path = Path()
	path.header.frame_id = "map"
	
	# Since we store the parent node inside each node and the startNode has null parent
	# We can iterate through it as a LinkedList
	while currentNode.parent is not 0 and not rospy.is_shutdown():
		path.poses.append(currentNode.toPose(navMap.resolution, navMap.origin))
		currentNode = currentNode.parent #go to the next node

	del path.poses[-1] #take the startPoint off
	
	path.poses = list(reversed(path.poses))
	
	return path


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

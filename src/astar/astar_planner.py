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

class AStarPlanner(object):
	def __init__(self, astarMap, startPose, goalPose):
		self.map = astarMap
		self.startPose = self.poseToPose2D(startPose)
		self.goalPose = self.poseToPose2D(goalPose)
		self.run()
	
	#helper function that converts a ROS Pose to Pose2D (discrete dir)
	def poseToPose2D(self, rosPose):
		#startPose
		q = [rosPose.pose.orientation.x, rosPose.pose.orientation.y, rosPose.pose.orientation.z, rosPose.pose.orientation.w]#convert the msg Quaternion to a tuple
		angle = tf.transformations.euler_from_quaternion(q)[2]#get the yaw angle in radians
		direction = abs(round((angle-math.pi)/(2.*math.pi/Direction.N_DIRECTIONS)))%Direction.N_DIRECTIONS#convert to Direction
		
		#CONVERSION FROM METERS TO GRID POSITION IN RELATION TO THE MAP
		#goalPose
		x = round((rosPose.pose.position.x-self.map.origin.position.x)/self.map.resolution)
		y = round((rosPose.pose.position.y-self.map.origin.position.y)/self.map.resolution)
		
		return Pose2D(x, y, direction)#Create a customPose version of the goalPose
	
	#calculate the heuristic of the specified node
	def heuristic(self, node):
		pointDiff = self.goalPose.minus(node.pos) #get the difference from the goal
				
		#turn heuristic cost
		turn_cost = abs(node.pos.dir-self.goalPose.dir)
		if(turn_cost > Direction.N_DIRECTIONS/2): turn_cost = Direction.N_DIRECTIONS-turn_cost
		
		return abs(pointDiff.x) + abs(pointDiff.y) #+ turn_cost #abs(dx)+abs(dy) is for 90  #counts the moveForward with cost 1
	
	#calculate the g_n cost of the specified node
	def gCost(self, node):
		#The cost functions makes a trade-off between turn movements (slow) and the cost (as higher as you get close to obstacles)
		if(node.pos.dir is not node.parent.pos.dir): #turn movement
			return node.parent.g_n + 5 # avoid turns (5 is an intermediate cost for moveForward)
		else: #move ForwardMovement
			return node.parent.g_n + 1 + self.map.getCost(node.pos.x, node.pos.y)/10 # around 0 to 10
	
	
	def expandNode(self, node):

		all_children = node.expandAll()
		children = []
		
		for child in all_children:
			if(not self.map.wasPoseMarked(child.pos)):#outOfBounds and Blocked poses are marked by default
				self.map.markPose(child.pos)
				self.map.setGridState(child.pos.x, child.pos.y, CellState.FRONTIER)
				
				child.g_n = gCost(child)
				child.h_n = heuristic(child)
				
				children.append(child)
		
		self.map.setGridState(node.pos.x, node.pos.y, CellState.EXPLORED) #set the current node as EXPLORED
		return children
	
	
	def run(self):
		# Create the first node
		origin = Node(self.map.startPose)
		
		#PriorityQueue of nodes
		pq = PriorityQueue()
		currentNode = origin
		
		self.path = Path() #empty Path
		
		# Compute the AStar search algorithm
		while not currentNode.pos.equals(self.map.goalPose) and not rospy.is_shutdown():
			children = self.expand(currentNode)
			
			for node in children:
				pq.put((node.cost(), node)) #it is ranked by the cost
			
			if(pq.qsize() == 0): #no more nodes to be expanded
				print "didn't find a path"
				currentNode = origin
				break
				
			# pop the "closest" node to the goal
			currentNode = pq.get()[1]
		
		#create path
		path.header.frame_id = "map"
		
		# Since we store the parent node inside each node and the startNode has null parent
		# We can iterate through it as a LinkedList
		while currentNode.parent is not 0 and not rospy.is_shutdown():
			path.poses.append(currentNode.toPose(navMap.resolution, navMap.origin))
			currentNode = currentNode.parent #go to the next node
	
		#del path.poses[-1] #take the startPoint off
		
		path.poses = list(reversed(path.poses))
		
		return path
	
	def calcPathWayPoints(self, lastNode):
		#create path
		self.path = Path() #empty Path
		self.wayPoints = Path() #empty Path
		self.path.header.frame_id = "map"
		self.wayPoints.frame_id = "map"
		
		# Since we store the parent node inside each node and the startNode has null parent
		# We can iterate through it as a LinkedList
		while node.parent is not 0 and not rospy.is_shutdown():
			self.path.poses.append(node.toPose(self.map.resolution, self.map.origin))
			
			if(node.pos.dir != node.parent.pos.dir or node.pos.equals(self.goalPose)): #and not node.pos.equals(node.parent.pos)
				self.wayPoints.poses.append(self.path.poses[-1])
			
			node = node.parent #go to the next node
	
		self.path.poses.reverse()
		self.wayPoints.poses.reverse()
		
		def getPathGridCells(self):
			return pathToGridCells(self.path, self.map.resolution)
		
		def getWayPointsGridCells(self):
			return pathToGridCells(self.wayPoints, self.map.resolution)
	

def pathToGridCells(path, resolution):
	grid = GridCells()
	grid.header.frame_id = "map"
	grid.cell_width = resolution
	grid.cell_height = resolution
	
	for pose in path.poses:
		grid.cells.append(pose.position)
		
	return grid

#import libraries
import rospy, math, tf, time

#import ROS msg classes
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from Queue import PriorityQueue
from geometry_msgs.msg import PoseStamped, Point , PoseWithCovarianceStamped, Pose, Quaternion
from nav_msgs.srv import GetPlan

#import custom files
from pose2d import *
from astar_map import *
from node import *

class AStarPlanner(object):
	"""
	Find the shortest path in an AStarMap
	"""
	def __init__(self, astarMap, startPose, goalPose):
		""" Default constructor
		it does all the calculations
		
		astarMap -- expect already set AStarMap object
		startPose -- ros PoseStamped msg (usually, the robot's current pose)
		goalPose -- ros PoseStamped msg where we want to go
		"""
		self.map = astarMap
		self.startPose = self.poseToPose2D(startPose)
		self.goalPose = self.poseToPose2D(goalPose)
		self.run()
	
	def poseToPose2D(self, rosPose):
		"""
		Helper function that converts a ROS Pose to Pose2D (discrete dir)
		
		rosPose -- PoseStamped msg
		
		returns converted Pose2D object
		"""

		q = [rosPose.pose.orientation.x, rosPose.pose.orientation.y, rosPose.pose.orientation.z, rosPose.pose.orientation.w]#convert the msg Quaternion to a tuple
		angle = tf.transformations.euler_from_quaternion(q)[2]#get the yaw angle in radians
		direction = abs(round((angle-math.pi)/(2.*math.pi/Direction.N_DIRECTIONS)))%Direction.N_DIRECTIONS#convert to Direction
		
		#CONVERSION FROM METERS TO GRID POSITION IN RELATION TO THE MAP
		#goalPose
		x = round((rosPose.pose.position.x-self.map.origin.position.x)/self.map.resolution)
		y = round((rosPose.pose.position.y-self.map.origin.position.y)/self.map.resolution)
		
		return Pose2D(x, y, direction)#Create a customPose version of the goalPose
	
	def heuristic(self, node):
		"""
		Calculate the heuristic of the specified node. For now, it takes the shortest distance (straight line)
		
		node -- specified node
		
		returns the heuristic cost (number)
		"""
		pointDiff = self.goalPose.minus(node.pos) #get the difference from the goal
				
		#turn heuristic cost
		#turn_cost = abs(node.pos.dir-self.goalPose.dir)
		#if(turn_cost > Direction.N_DIRECTIONS/2): turn_cost = Direction.N_DIRECTIONS-turn_cost
		
		return math.hypot(pointDiff.x, pointDiff.y)
		#return abs(pointDiff.x) + abs(pointDiff.y) #+ turn_cost #abs(dx)+abs(dy) is for 90  #counts the moveForward with cost 1
	
	def gCost(self, node):
		"""
		Calculate the cost to reach the specified node. The cost is based on what move it came from
		moveForward: proportional to the costmap value (1 to 5 for now)
		turn: 1 (you can make it higher to avoid turn movements. Useful when it takes  a long time)
		
		node -- specified node
		
		returns the cost (number)
		"""
		#The cost functions makes a trade-off between turn movements (slow) and the cost (as higher as you get close to obstacles)
		if(node.pos.dir is not node.parent.pos.dir): #turn movement
			return node.parent.g_n + 1 # avoid turns (5 is an intermediate cost for moveForward)
		else: #move ForwardMovement
			return node.parent.g_n + 1 + self.map.getCost(node.pos.x, node.pos.y)/20 # around 0 to 10
	
	
	def expandNode(self, node):
		"""
		return the free nodes that were not visited yet (graph search) from node.expandAll() method
		"""
		all_children = node.expandAll()
		children = []
		
		for child in all_children:
			if(not self.map.wasPoseMarked(child.pos)):#outOfBounds and Blocked poses are marked by default
				self.map.markPose(child.pos)
				self.map.setGridState(child.pos.x, child.pos.y, CellState.FRONTIER)
				
				child.g_n = self.gCost(child)
				child.h_n = self.heuristic(child)
				
				children.append(child)
		
		#Usually it's better to comment this line to better visualize the frontiers on rviz 
		self.map.setGridState(node.pos.x, node.pos.y, CellState.EXPLORED) #set the current node as EXPLORED
		return children
	
	
	def run(self):
		"""
		basically calculate the shortest path with A* search algorithm
		"""

		origin = Node(self.startPose)
		
		#PriorityQueue of nodes
		pq = PriorityQueue()
		currentNode = origin
		
		# Compute the AStar search algorithm
		while not currentNode.pos.equals(self.goalPose) and not rospy.is_shutdown():
			children = self.expandNode(currentNode)
			
			for node in children:
				pq.put((node.cost(), node)) #it is ranked by the cost
			
			if(pq.qsize() == 0): #no more nodes to be expanded
				print "didn't find a path"
				currentNode = origin
				break
				
			# pop the "closest" node to the goal
			currentNode = pq.get()[1]
		
		self.calcPathWayPoints(currentNode) #lastNode
	
	def calcPathWayPoints(self, node):
		"""
		generate path and extract wayPoints from it
		
		wayPoints are defined as the pose that has a different direction from the previos one
		they are useful to send movement goals to the robot
		
		node -- lastNode (usually the goal node)
		"""
		#create path
		self.path = Path() #empty Path
		self.wayPoints = Path() #empty Path
		self.path.header.frame_id = "map"
		self.wayPoints.header.frame_id = "map"
		
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
		"""get path as ros gridCells (for rviz visualization)"""
		return pathToGridCells(self.path, self.map.resolution)
	
	def getWayPointsGridCells(self):
		"""get way points as ros gridCells (for rviz visualization)"""
		return pathToGridCells(self.wayPoints, self.map.resolution)
	

def pathToGridCells(path, resolution):
	"""
	Helper function that convertes a Path() ros msg to gridCells (better visualization)
	"""
	grid = GridCells()
	grid.header.frame_id = "map"
	grid.cell_width = resolution
	grid.cell_height = resolution
	
	for pose in path.poses:
		grid.cells.append(pose.pose.position)
		
	return grid

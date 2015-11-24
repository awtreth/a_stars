#!usr/bin/env python

#import libraries
import rospy, math, tf, time

#import ROS msg classes
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from Queue import PriorityQueue
from geometry_msgs.msg import PoseStamped, Point , PoseWithCovarianceStamped, Pose, Quaternion
from nav_msgs.srv import GetPlan

#DEFINITION OF ENUM
#def enum(*sequential, **named):
#    enums = dict(zip(sequential, range(len(sequential))), **named)
#    return type('Enum', (), enums)
def enum(**enums):
    return type('Enum', (), enums)

#CONSTANTS
#the best way that we thought to do this was with Enumeration, but Pythond 2.7 does not support it
#Map points states
CellState = enum(FREE=0, FRONTIER=1, EXPLORED=2, BLOCKED=3)

#Orientation
Direction = enum(WEST=0, NORTH=1, EAST=2, SOUTH=3, N_DIRECTIONS=4) #90degree turn (starting with WEST)
#Direction = enum(WEST=0, NORTHWEST=1, NORTH=2, NORTHEAST=3, EAST=4, SOUTHEAST=5, SOUTH=6, SOUTHWEST=7, N_DIRECTIONS=8) #90degree turn (starting with WEST)
#Direction = enum(NORTHWEST=0, NORTH=1, NORTHEAST=2, EAST=3, SOUTHEAST=4, SOUTH=5, SOUTHWEST=6, WEST=7, N_DIRECTIONS=8) #90degree turn (starting with NORTHWEST)


#Helper class
#It represents a 2D oriented Point, or pose
class Pose(object): #FIXME: actually, it is a Pose (point + orientation)
	
	#constructor
	def __init__(self, x = 0, y = 0, orientation = 0):
		self.x = x
		self.y = y
		self.dir = orientation #west, NORTH, east or south for 90degrees turn

	#distance
	def dist(self, otherPose):
		return math.sqrt((self.x-otherPose.x)**2 + (self.y-otherPose.y)**2)
		
	#this - other
	def minus(self, otherPose):
		return Pose(self.x-otherPose.x, self.y-otherPose.y)
	
	#this + other
	def plus(self, otherPose):
		return Pose(otherPose.x+self.x, otherPose.y+self.y)
	
	#FIXME: check if it is used
	def equals(self, otherPose):
		return self.x == otherPose.x and self.y == otherPose.y
		
	def equalPos(self, other):
		return self.x == other.x and self.y == other.y 
		
	def asString(self):
		return "(%d, "%self.x + "%d, "%self.y + "%d)"%self.dir
		
		

#keep tracks of the state of each cell in the map
class NavMap(object):
	
	def __init__(self, ocmap, startPose, goalPose, threshold = 99): #ocmap = OccupancyGrid #TODO: maybe we could reate
		self.resolution = ocmap.info.resolution
		self.width = ocmap.info.width #just to make it easier to see
		self.height = ocmap.info.height
		
		#where we store the state of the cells
		self.table = [CellState.FREE]*len(ocmap.data) #at first we all with FREE value
		
		q = [startPose.pose.orientation.x, startPose.pose.orientation.y, startPose.pose.orientation.z, startPose.pose.orientation.w]
		startDir = abs(round(((tf.transformations.euler_from_quaternion(q))[2]-math.pi)/(2.*math.pi/Direction.N_DIRECTIONS)))%Direction.N_DIRECTIONS
		
		#print math.degrees((tf.transformations.euler_from_quaternion(q))[2])
		
		q = [goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w]
		goalDir = abs(round(((tf.transformations.euler_from_quaternion(q))[2]-math.pi)/(2.*math.pi/Direction.N_DIRECTIONS)))%Direction.N_DIRECTIONS
		
		#print math.degrees((tf.transformations.euler_from_quaternion(q))[2])
		self.goalPose = Pose(round(goalPose.pose.position.x/self.resolution), round(goalPose.pose.position.y/self.resolution), goalDir)
		
		#print "startDir %d"%startDir
		#print "goalDir %d"%goalDir
		
		self.startPose = Pose(round(startPose.pose.position.x/self.resolution), round(startPose.pose.position.y/self.resolution), startDir)
		
		for i in range(len(self.table)):
			if(ocmap.data[i]>=threshold):
				self.table[i] = CellState.BLOCKED #set the obstacles

	#return the expanded nodes from the node parent
	def expand(self, parent): #parent is a node
		#TODO: we can check if parent is in FRONTIER (we can throw an exception)
		self.set(parent.pos.x, parent.pos.y, CellState.EXPLORED) #set the current node as EXPLORED
		children = [] #return list
		
		#Scan the neighborhood of the node 
		for i in range(-1,2): #from -1 to +1 (include +1)
			for j in range(-1,2):
				#try i == j == 0 to include diagonals
				
				#set the position
				pos = parent.pos.plus(Pose(j,i)) #we can have problems with the definition of (i,j) (row,col) and (x,y)
				#0  -1 -> 0 (west)
				#-1  0 -> 1 (NORTH)
				#0   1 -> 2 (east)
				#1   0 -> 3 (south)
				#for 90degree %FIXME: make it generic
				pos.dir = abs(-2*i+j+1)
				
				
				if not self.inBounds(pos.x,pos.y) or self.get(pos.x,pos.y)==CellState.BLOCKED: #if we don't check, we can get an error in the get method of the next condition
					continue #this point cannot be expanded
				else:# it is above, below or beside (90degree turn) or the position is not FREE
					if abs(i)==abs(j) or (self.get(pos.x, pos.y, pos.dir) is not CellState.FREE):
						continue #this point cannot be expanded
				
				
				self.set(pos.x, pos.y, CellState.FRONTIER, 2**(pos.dir+2)) #uptade the position in the table with FRONTIER
				
				pointDiff = self.goalPose.minus(pos) #get the difference from the goal
				
				#TODO: I think we can make the difference inside minus() method and use pointDiff.dir as part of the Heuristic
				turn_cost = abs(pos.dir-self.goalPose.dir)
				if(turn_cost > Direction.N_DIRECTIONS/2): turn_cost = Direction.N_DIRECTIONS-turn_cost
				h_n = abs(pointDiff.x) + abs(pointDiff.y) #+ turn_cost #abs(dx)+abs(dy) is for 90 degree turn
				#if((pointDiff.x is not 0) and pointDiff.y is not 0): h_n = h_n + 1
				#TODO: count the number of turns in the heuristic
				
				children.append(Node(pos, parent, h_n)) #add the node into the return array

		return children
	
	# get the state of a gridCell in the map
	def get(self, x, y, direction = -1):
		if(self.inBounds(x,y) ):
			if(direction is not -1):
				return self.table[int(y*self.width+x)]&(2**(direction+2))
			else:
				return self.table[int(y*self.width+x)]&3
		else:
			return 0 #TODO: throw Exception
	
	# set the state of a position
	def set(self, x, y, state, direction = -1):
		if(self.inBounds(x,y) ):
			idx = int(y*self.width+x)
			if(direction is not -1):
				self.table[idx] |= direction
			
			if(self.table[idx]&CellState.EXPLORED is not CellState.EXPLORED):
				self.table[idx] -= self.table[idx]&3
				self.table[idx] |= state 
		else:
			return 0 #TODO: throw Exception
	
	
	# Check if a point is in bounds
	def inBounds(self,x,y):
		if(x >= 0 and x < self.width and y >= 0 and y < self.height):
			return True
		else:
			return False
	
	# Return the GridCells ROS msg of the cells with state "value"
	def getGridCell(self, value):
		grid = GridCells()
		grid.header.frame_id = "map"
		grid.cell_width = self.resolution
		grid.cell_height = self.resolution
		#grid.cells = []
		
		for y in range(0,self.height):
			for x in range(0,self.width):
				if(self.get(x,y) == value):
					point = Point()
					point.x = x*self.resolution
					point.y = y*self.resolution
					point.z = 0
					grid.cells.append(point)
		
		
		return grid
		
	
# It represents a possible state of the robot
class Node(object):
	
	# pos - OrientedPoint (x,y,dir)
	# parent - reference to the node that it came from
	# h_n - heuristic function
	def __init__(self, pos, parent = 0 , h_n = 0):
		if(parent): #if it doesn't have a parent (usually the startPoint)
			self.pos = pos
			self.parent = parent
			self.h_n = h_n
			turn_cost = abs(self.pos.dir-parent.pos.dir)
			if(turn_cost > Direction.N_DIRECTIONS/2): turn_cost = Direction.N_DIRECTIONS-turn_cost
			self.g_n = parent.g_n + 1 + turn_cost
		else:
			self.pos = pos
			self.parent = 0 #check if it is right, because parent is of tyep Node
			self.h_n = 0
			self.g_n = 0
	# to compute AStar
	def cost(self):
		return self.g_n + self.h_n
		
		
def readMap(msg):
	global rosmap
	rosmap = msg
	
def readLocalCostMap(msg):
	global localCostMap
	localCostMap = msg


#Helper function that convert a node to PoseStaped ROS msg
def nodeToPose(node, resolution):
	
	pose = PoseStamped()
	pose.header.frame_id = "map"
	point = Point()
	point.x = node.pos.x*resolution
	point.y = node.pos.y*resolution
	point.z = 0
	quatTuple = tf.transformations.quaternion_from_euler(0,0,math.pi-node.pos.dir*(2*math.pi/Direction.N_DIRECTIONS));
	pose.pose.orientation = Quaternion(quatTuple[0],quatTuple[1], quatTuple[2], quatTuple[3])
	pose.pose.position = point
	
	return pose
	

def planCallBack(msg):
	global rosmap
	global frontierPub
	global exploredPub
	global freePub
	global blockedPub
	global wayPointsPub
	
	print "received request"
	
	navMap = NavMap(rosmap, msg.start, msg.goal)
	print "created map"
	
	# Create the first node
	origin = Node(navMap.startPose)
	
	#PriorityQueue of nodes
	pq = PriorityQueue()
	currentNode = origin
	
	while not currentNode.pos.equalPos(navMap.goalPose) and not rospy.is_shutdown():
		children = navMap.expand(currentNode)
		
		for node in children:
			pq.put((node.cost(), node)) #it is ranked by the cost
		
		if(pq.qsize() == 0):
			print "didn't find a path"
			return Path()
		
		# pop the "closest" node to the goal
		currentNode = pq.get()[1]
		#print currentNode.g_n
		#print currentNode.h_n
		#print currentNode.cost()
	
		#print currentNode.pos.asString()
		#raw_input("");
		# Publish the data in Rviz environment	
		#frontierPub.publish(navMap.getGridCell(CellState.FRONTIER))
		#exploredPub.publish(navMap.getGridCell(CellState.EXPLORED))
		#freePub.publish(navMap.getGridCell(CellState.FREE))
		#blockedPub.publish(navMap.getGridCell(CellState.BLOCKED))
	
	# Publish the data in Rviz environment	
	frontierPub.publish(navMap.getGridCell(CellState.FRONTIER))
	exploredPub.publish(navMap.getGridCell(CellState.EXPLORED))
	freePub.publish(navMap.getGridCell(CellState.FREE))
	blockedPub.publish(navMap.getGridCell(CellState.BLOCKED))
	
	#Get the path
	path = Path()
	path.header.frame_id = "map"
	path.poses.append(msg.goal)
	path.poses.append(nodeToPose(currentNode, navMap.resolution))
	print "("+repr(currentNode.pos.x)+", "+repr(currentNode.pos.y) + ", " + repr(currentNode.pos.dir) + ')'
	
	# Since we store the parent node inside each node and the startNode has null parent
	# We can iterate through it as a LinkedList
	while currentNode.parent is not 0 and not rospy.is_shutdown():
		# Filter to get the wayPoints
		if(currentNode.pos.dir is not currentNode.parent.pos.dir): #if we change the dir
			# the wayPose is the position of the parent and the orientation of the child
			pose = Pose(currentNode.parent.pos.x, currentNode.parent.pos.y, currentNode.pos.dir)
			print "("+repr(currentNode.parent.pos.x)+", "+repr(currentNode.parent.pos.y) + ", " + repr(currentNode.pos.dir) + ')'
			newnode = Node(pose)
			path.poses.append(nodeToPose(newnode, navMap.resolution))
		
		currentNode = currentNode.parent #go to the next node
	
	del path.poses[-1] #take the startPoint off
	
	path.poses = list(reversed(path.poses))
	
	#create WayPoints GridCell
	wayPoints = GridCells()
	wayPoints.header.frame_id = "map"
	wayPoints.cell_width = navMap.resolution
	wayPoints.cell_height = navMap.resolution
	
	#Create path from the goal to the startPoint
	for pose in path.poses :
		wayPoints.cells.append(pose.pose.position)
	
	#print wayPoints.cells
	
	#TODO: revert path (to make it from start to the goal"
	
	#publish wayPoints and path
	wayPointsPub.publish(wayPoints)
	
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
	rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, readMap)

	frontierPub = rospy.Publisher("/grid_Frontier", GridCells, queue_size=1)
	exploredPub = rospy.Publisher("/grid_Explored", GridCells, queue_size=1)
	freePub = rospy.Publisher("/grid_Free", GridCells, queue_size=1)
	blockedPub = rospy.Publisher("/grid_Blocked", GridCells, queue_size=1)
	wayPointsPub = rospy.Publisher("/way_points", GridCells, queue_size=5)
    
	rospy.Service("astar_planner", GetPlan, planCallBack)
    
	rospy.sleep(1)
    
	rospy.spin()

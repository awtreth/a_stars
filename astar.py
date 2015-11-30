#!usr/bin/env python

#import libraries
import rospy, math, tf, time

#import ROS msg classes
from nav_msgs.msg import OccupancyGrid, GridCells, Path
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
		
	#this - other (just count x and y dimensions)
	def minus(self, otherPose):
		return Pose(self.x-otherPose.x, self.y-otherPose.y)
	
	#this + other (just count x and y dimensions)
	def plus(self, otherPose):
		return Pose(otherPose.x+self.x, otherPose.y+self.y)
	
	#consider all dimensions
	def equals(self, otherPose):
		return self.x == otherPose.x and self.y == otherPose.y and self.dir == otherPose.dir
	
	#check just x and y
	def equalPos(self, other):
		return self.x == other.x and self.y == other.y 
	
	#Use it to print out
	def asString(self):
		return "(%d, "%self.x + "%d, "%self.y + "%d)"%self.dir


#keep tracks of the state of each cell in the map
class NavMap(object):
	
	def __init__(self, ocmap, startPose, goalPose, threshold = 99): #ocmap = OccupancyGrid (costmap)
		self.resolution = ocmap.info.resolution
		self.width = ocmap.info.width #just to make it easier to see
		self.height = ocmap.info.height
		self.origin = ocmap.info.origin #Pose (of ROS)
		
		#where we store the state of the cells
		self.states = [CellState.FREE]*len(ocmap.data) #at first we all with FREE value
		self.poseMarks = [0]*len(ocmap.data) #store the state of each node
		self.costmap = ocmap.data[:] #simple copy of the costmap
		
		#DISCRETIZATION OF goalPose AND startPose ANGLES (OBS: consider that costmap is aligned in ang=0) FIXME
		#goalPose
		q = [startPose.pose.orientation.x, startPose.pose.orientation.y, startPose.pose.orientation.z, startPose.pose.orientation.w]#convert the msg Quaternion to a tuple
		angle = tf.transformations.euler_from_quaternion(q)[2]#get the yaw angle in radians
		startDir = abs(round((angle-math.pi)/(2.*math.pi/Direction.N_DIRECTIONS)))%Direction.N_DIRECTIONS#convert to Direction
		
		#startPose
		q = [goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w]#convert the msg Quaternion to a tuple
		angle = tf.transformations.euler_from_quaternion(q)[2]#get the yaw angle in radians
		goalDir = abs(round((angle-math.pi)/(2.*math.pi/Direction.N_DIRECTIONS)))%Direction.N_DIRECTIONS#convert to Direction
		
		#CONVERSION FROM METERS TO GRID POSITION IN RELATION TO THE MAP
		#OBS: consider that costmap is aligned in ang=0) FIXME
		
		#goalPose
		x = round((goalPose.pose.position.x-self.origin.position.x)/self.resolution)
		y = round((goalPose.pose.position.y-self.origin.position.y)/self.resolution)
		self.goalPose = Pose(x, y, goalDir)#Create a customPose version of the goalPose
		
		#startPose
		x = round((startPose.pose.position.x-self.origin.position.x)/self.resolution)
		y = round((startPose.pose.position.y-self.origin.position.y)/self.resolution)
		self.startPose = Pose(x, y, startDir)#Create a customPose version of the goalPose
		
		print "startPose: " + self.startPose.asString()
		print "goalPose: " + self.goalPose.asString()
		
		#create the state table
		for i in range(len(self.states)):
			if(ocmap.data[i]>=threshold):
				self.states[i] = CellState.BLOCKED #set the obstacles
			else:
				self.states[i] = CellState.FREE

	#return the expanded nodes from the node parent
	def expand(self, parent): #parent is a node
		#TODO: we can check if parent is in FRONTIER (we can throw an exception)
		
		all_children = []
		children = [] #return list
		
		#print "parent " + parent.pos.asString()
		
		all_children.append(parent.moveForward())
		#print "moveForward " + all_children[-1].pos.asString()
		all_children.append(parent.turnLeft())
		#print "turnLeft " + all_children[-1].pos.asString()
		all_children.append(parent.turnRight())
		#print "turnRight " + all_children[-1].pos.asString()
		
		for child in all_children:
			if(not self.wasPoseMarked(child.pos)):#outOfBounds and Blocked poses are marked by default
				self.markPose(child.pos)
				self.setGridState(child.pos.x, child.pos.y, CellState.FRONTIER)
				
				pointDiff = self.goalPose.minus(child.pos) #get the difference from the goal
				turn_cost = abs(child.pos.dir-self.goalPose.dir)
				if(turn_cost > Direction.N_DIRECTIONS/2): turn_cost = Direction.N_DIRECTIONS-turn_cost
				abs(pointDiff.x) + abs(pointDiff.y) #+ turn_cost #abs(dx)+abs(dy) is for 90 
				
				child.h_n = abs(pointDiff.x) + abs(pointDiff.y) #+ turn_cost #abs(dx)+abs(dy) is for 90 
				child.g_n = parent.g_n+1
				
				children.append(child)
		
		self.setGridState(parent.pos.x, parent.pos.y, CellState.EXPLORED) #set the current node as EXPLORED
		return children
	
	# get the state of a gridCell in the map
	def getGridState(self, x, y):
		if(self.inBounds(x,y)): return self.states[int(y*self.width+x)]
		else: return 0 #TODO: throw Exception
	
	# set the state of a position
	def setGridState(self, x, y, state):
		if(self.inBounds(x,y)): self.states[int(y*self.width+x)] = state
		else: pass #TODO: throw Exception
	
	# check if a pose was already checked
	def wasPoseMarked(self, pose):
		if(self.inBounds(pose.x,pose.y) and self.getGridState(pose.x, pose.y) is not CellState.BLOCKED):
			if(self.poseMarks[int(pose.y*self.width+pose.x)] & int(2**pose.dir)): return True
			else: return False
		else:
			#print pose.asString()
			#print self.getGridState(pose.x,pose.y)
			#print "outOfBounds in wasPoseMarked"
			return True #TODO: throw Exception
	
	# set the state of a position
	def markPose(self, pose):
		if(self.inBounds(pose.x,pose.y)): self.poseMarks[int(pose.y*self.width+pose.x)] |= int(2**pose.dir)
		else:
			print "outOfBounds in markPOse"
			pass #TODO: throw Exception
	
	# get the cost of a gridCell in the map
	def getCost(self, x, y):
		if(self.inBounds(x,y)): return self.costmap[int(y*self.width+x)]
		else: return 0 #TODO: throw Exception
	
	# set the cost of a position
	def setCost(self, x, y, newcost):
		if(self.inBounds(x,y) ): self.costmap[int(y*self.width+x)] = newcost
		else: pass #TODO: throw Exception
	
	# Check if a point is in bounds
	def inBounds(self,x,y):
		if(x >= 0 and x < self.width and y >= 0 and y < self.height): return True
		else: return False
	
	# Return the GridCells ROS msg of the cells with state "value"
	def getGridCell(self, value):
		grid = GridCells()
		grid.header.frame_id = "map"
		grid.cell_width = self.resolution
		grid.cell_height = self.resolution
		
		for y in range(0,self.height):
			for x in range(0,self.width):
				if(self.getGridState(x,y) == value):
					point = Point()
					point.x = x*self.resolution+self.origin.position.x
					point.y = y*self.resolution+self.origin.position.y
					point.z = 0
					grid.cells.append(point)
		
		return grid
		
	def nodeToPose(self, node):
		pose = PoseStamped()
		pose.header.frame_id = "map"
		point = Point()
		point.x = node.pos.x*self.resolution + self.origin.position.x
		point.y = node.pos.y*self.resolution + self.origin.position.y
		point.z = 0
		quatTuple = tf.transformations.quaternion_from_euler(0,0,math.pi-node.pos.dir*(2*math.pi/Direction.N_DIRECTIONS));
		pose.pose.orientation = Quaternion(quatTuple[0],quatTuple[1], quatTuple[2], quatTuple[3])
		pose.pose.position = point
		
		return pose
		
	
# It represents a possible state of the robot
class Node(object):
	
	# pos - OrientedPoint (x,y,dir)
	# parent - reference to the node that it came from
	# h_n - heuristic function
	def __init__(self, pos, parent = 0 , g_n = 0, h_n = 0):
		if(parent): #if it doesn't have a parent (usually the startPoint)
			self.pos = pos
			self.parent = parent
			self.h_n = h_n
			self.g_n = g_n
		else:
			self.pos = pos
			self.parent = 0 #check if it is right, because parent is of tyep Node
			self.h_n = 0
			self.g_n = 0
	# to compute AStar
	def cost(self):
		return self.g_n + self.h_n
	
	def turnLeft(self):
		pos = Pose(self.pos.x, self.pos.y, (self.pos.dir-1)%Direction.N_DIRECTIONS)
		return Node(pos, self)
		
	def turnRight(self):
		pos = Pose(self.pos.x, self.pos.y, (self.pos.dir+1)%Direction.N_DIRECTIONS)
		return Node(pos, self)
		
	def moveForward(self): #TODO: warning: this is valid just for 90degrees turn
		if(Direction.N_DIRECTIONS is 4):
			if(int(self.pos.dir%2) is 0): #even(west or east)
				#print "even"
				pos = Pose(self.pos.x+(self.pos.dir-1),self.pos.y, self.pos.dir)
				return Node(pos,self)
			else: #odd number
				#print "odd"
				pos = Pose(self.pos.x,self.pos.y-(self.pos.dir-2), self.pos.dir)
				return Node(pos,self)
		else:
			print "not defined behavior in moveForward for " + repr(Direction.N_DIRECTIONS) + "directions"
			return self

#CostMap read callBack Function
def readMap(msg):
	global rosmap
	rosmap = msg

#Service callBack Function
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
	
	while not currentNode.pos.equals(navMap.goalPose) and not rospy.is_shutdown():
		children = navMap.expand(currentNode)
		
		for node in children:
			#print "put " + node.pos.asString()
			pq.put((node.cost(), node)) #it is ranked by the cost
		
		if(pq.qsize() == 0):
			print "didn't find a path"
			frontierPub.publish(navMap.getGridCell(CellState.FRONTIER))
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
	#path.poses.append(msg.goal)
	#path.poses.append(nodeToPose(currentNode, navMap.resolution))
	#print "("+repr(currentNode.pos.x)+", "+repr(currentNode.pos.y) + ", " + repr(currentNode.pos.dir) + ')'
	
	# Since we store the parent node inside each node and the startNode has null parent
	# We can iterate through it as a LinkedList
	while currentNode.parent is not 0 and not rospy.is_shutdown():
		path.poses.append(navMap.nodeToPose(currentNode))
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
	rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, readMap)

	frontierPub = rospy.Publisher("/grid_Frontier", GridCells, queue_size=1)
	exploredPub = rospy.Publisher("/grid_Explored", GridCells, queue_size=1)
	freePub = rospy.Publisher("/grid_Free", GridCells, queue_size=1)
	blockedPub = rospy.Publisher("/grid_Blocked", GridCells, queue_size=1)
	wayPointsPub = rospy.Publisher("/way_points", GridCells, queue_size=5)
    
	rospy.Service("astar_planner", GetPlan, planCallBack)
    
	rospy.sleep(1)
    
	rospy.spin()

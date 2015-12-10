from pose2d import *
from node import *

import tf, math

#import ROS msg classes
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import PoseStamped, Point , Pose, Quaternion

#CONSTANTS
#the best way that we thought to do this was with Enumeration, but Pythond 2.7 does not support it
#Map points states
CellState = enum(FREE=0, FRONTIER=1, EXPLORED=2, BLOCKED=3)

#keep tracks of the state of each cell in the map
class AStarMap(object):
	
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
		#goalPose
		x = round((goalPose.pose.position.x-self.origin.position.x)/self.resolution)
		y = round((goalPose.pose.position.y-self.origin.position.y)/self.resolution)
		self.goalPose = Pose2D(x, y, goalDir)#Create a customPose version of the goalPose
		
		#startPose
		x = round((startPose.pose.position.x-self.origin.position.x)/self.resolution)
		y = round((startPose.pose.position.y-self.origin.position.y)/self.resolution)
		self.startPose = Pose2D(x, y, startDir)#Create a customPose version of the goalPose
		
		print "startPose: " + self.startPose.asString()
		print "goalPose: " + self.goalPose.asString()
		
		#create the state table
		for i in range(len(self.states)):
			if(ocmap.data[i]>=threshold):
				self.states[i] = CellState.BLOCKED #set the obstacles
			else:
				self.states[i] = CellState.FREE #unknown places (-1) are also marked as FREE

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
				
				#turn heuristic cost
				turn_cost = abs(child.pos.dir-self.goalPose.dir)
				if(turn_cost > Direction.N_DIRECTIONS/2): turn_cost = Direction.N_DIRECTIONS-turn_cost
				
				#compute the heuristic
				child.h_n = abs(pointDiff.x) + abs(pointDiff.y) #+ turn_cost #abs(dx)+abs(dy) is for 90  #counts the moveForward with cost 1
				
				#The cost functions makes a trade-off between turn movements (slow) and the cost (as higher as you get close to obstacles)
				if(child.pos.dir is not parent.pos.dir): #turn movement
					child.g_n = parent.g_n + 5 # avoid turns (5 is an intermediate cost for moveForward)
				else: #move ForwardMovement
					child.g_n = parent.g_n + 1 + self.getCost(child.pos.x, child.pos.y)/10 # around 0 to 10
				
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
	
	

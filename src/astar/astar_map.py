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
	
	def __init__(self, ocmap, updateMap, threshold = 90): #ocmap = OccupancyGrid (costmap)
		self.resolution = ocmap.info.resolution
		self.width = ocmap.info.width #just to make it easier to see
		self.height = ocmap.info.height
		self.origin = ocmap.info.origin #Pose (of ROS)
		self.threshold = threshold
		
		#where we store the state of the cells
		self.states = [CellState.FREE]*len(ocmap.data) #at first we all with FREE value
		self.poseMarks = [0]*len(ocmap.data) #store the state of each node
		self.costmap = list(ocmap.data) #simple copy of the costmap
		self.updateCostMap(updateMap)
		print "updated the map"
		#create the state table
		for i in range(len(self.states)):
			if(self.costmap[i]>=self.threshold or self.costmap[i] is -1):
				self.states[i] = CellState.BLOCKED #set the obstacles
			else:
				self.states[i] = CellState.FREE #unknown places (-1) are also marked as FREE

	def updateCostMap(self,update):
		for y in range(0,update.height):
			for x in range(0,update.width):
				self.setCost(x+update.x,y+update.y,update.data[y*update.width+x])

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

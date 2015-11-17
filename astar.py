#!usr/bin/env python

import rospy, math

from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from Queue import PriorityQueue
#import geometry_msgs
from geometry_msgs.msg import PoseStamped, Point , PoseWithCovarianceStamped
#from enum import Enum

#CONSTANTS
#Map points states
free = 0
frontier = 1
explored = 2
blocked = 3
#Orientation
west = 0
north = 1
east = 2
south = 3


#Helper class
class Point(object): #FIXME: actually, it is a Pose (point + orientation)
	def __init__(self, x = 0, y = 0, orientation = 0):
		self.x = x
		self.y = y
		self.dir = orientation

	#distance
	def dist(self, otherPoint):
		return math.sqrt(self.x**2 + self.y**2)
	#this - other
	def minus(self, otherPoint):
		return Point(self.x-otherPoint.x, self.y-otherPoint.y)
	#this + other
	def plus(self, otherPoint):
		return Point(otherPoint.x+self.x, otherPoint.y+self.y)
	
	def equals(self, otherPoint):
		return self.x == otherPoint.x and self.y == otherPoint.y
		
		

#keep tracks of the state of each cell in the map
class NavMap(object):
	
	def __init__(self, ocmap, goalPoint): #ocmap = OccupancyGrid global_cost_map
		self.resolution = ocmap.info.resolution
		self.width = ocmap.info.width #just to make it easier to see
		self.height = ocmap.info.height
		
		#where we store the state of the cells
		self.table = [free]*len(ocmap.data)
		self.goalPoint = goalPoint
		
		for i in range(len(self.table)):
			if(ocmap.data[i]>90):
				self.table[i] = blocked

	#return the expanded nodes from the node parent
	def expand(self, parent): #parent is a node
		#TODO: we can check if parent is in frontier (we can throw an exception)
		self.set(parent.pos.x, parent.pos.y, explored)
		children = [] #return list
		for i in range(-1,2):
			for j in range(-1,2):
				#with abs(i)==abs(j) we just consider turns of 90 degreees
				#try i == j == 0 to include diagonals
				pos = parent.pos.plus(Point(i,j)) #we can have problems with the definition of (i,j) (row,col) and (x,y) FIXME
				#FIXME: not good code
				if(not self.inBounds(pos)):
					continue
				else:
					#print "value = %d"%self.get(pos.x, pos.y)
					if(abs(i)==abs(j) or self.get(pos.x, pos.y) is not free):
						continue
				
				#0  -1 -> 0 (west)
				#-1  0 -> 1 (north)
				#0   1 -> 2 (east)
				#1   0 -> 3 (south)
				pos.dir = abs(2*i+j+1)
				
				self.set(pos.x, pos.y, frontier)
				
				pointDiff = self.goalPoint.minus(parent.pos)
				
				h_n = abs(pointDiff.x) + abs(pointDiff.y) #h_n = abs(dx) + abs(dy) #for 90 degree turn
				
				children.append(Node(pos, parent, h_n))

		return children
	
	
	def get(self, x, y):
		if(self.inBounds(Point(x,y)) ):
			return self.table[int(y*self.width+x)]
		else:
			return 0 #TODO: throw Exception
			
	def set(self, x, y, value):
		if(self.inBounds(Point(x,y)) ):
			self.table[int(y*self.width+x)] = value
		else:
			return 0 #TODO: throw Exception
	
	def inBounds(self,point):
		if(point.x > 0 and point.x < self.width and point.y > 0 and point.y < self.height):
			return True
		else:
			return False
			
	def getGridCell(self, value):
		grid = GridCells()
		grid.header.frame_id = "map"
		grid.cell_width = self.resolution
		grid.cell_height = self.resolution
		#grid.cells = []
		
		for y in range(self.height):
			for x in range(self.width):
				if(self.get(x,y) == value):
					point = Point()
					point.x = x*self.resolution
					point.y = y*self.resolution
					point.z = 0
					grid.cells.append(point)
		
		
		return grid
		
	
	
class Node(object):
	
	
	def __init__(self, pos, parent = 0 , h_n = 0):
		if(parent):
			self.pos = pos
			self.parent = parent
			self.h_n = h_n
			self.g_n = parent.g_n + 1 + (self.pos.dir+parent.pos.dir)%2
		else:
			self.pos = pos
			self.parent = 0 #check if it is right, because parent is of tyep Node
			self.h_n = 0
			self.g_n = 0

	def cost(self):
		return self.g_n + self.h_n
		

def globCostCallBack(data):
	global globMapGrid
	global startPathPlanning
	global resolution
	globMapGrid = data
	resolution = globMapGrid.info.resolution
	startPathPlanning = True

	print "Got the map"

def readGoal(msg):
	global goalPoint
	global newGoal
	goalPoint = Point(msg.pose.position.x, msg.pose.position.y)
	newGoal = True
	print "got Goal"
	
def readStart(msg):
	global startPoint
	global newStart
	startPoint = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)
	newStart = True
	print "got start"
	
	#print goalPoint.x
	#print goalPoint.y	

#~ def readOdom(odom):
	#~ global startPoint
	#~ startPoint = Point(odom.pose.pose.position.x, odom.pose.pose.position.y)
	#~ 
	#print startPoint.x
	#print startPoint.y

	

def run():
	global goalPoint
	global newGoal
	global globMapGrid
	global startPathPlanning
	global startPoint
	global resolution
	global newStart
	
	startPathPlanning = False
	newGoal = False
	newStart = False
	
	
	rospy.init_node('lab3')
	
	globCostSub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globCostCallBack)
	globPlanPub = rospy.Publisher("/aStarPath", Path, queue_size=1)
	markerSub = rospy.Subscriber('/aStarNavGoal', PoseStamped, readGoal)
	startSub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readStart)

	frontierPub = rospy.Publisher("/grid_Frontier", GridCells, queue_size=1)
	exploredPub = rospy.Publisher("/grid_Explored", GridCells, queue_size=1)
	freePub = rospy.Publisher("/grid_Free", GridCells, queue_size=1)
	blockedPub = rospy.Publisher("/grid_Blocked", GridCells, queue_size=1)
	pathPub = rospy.Publisher("/grid_path", GridCells, queue_size=1)
    
    
	sleeper = rospy.Duration(1)
	rospy.sleep(sleeper)
    
   
    
	while not rospy.is_shutdown():
		
		if (startPathPlanning and newGoal and newStart):
			startPoint = Point(round(startPoint.x/resolution), round(startPoint.y/resolution))
			goalPoint = Point(round(goalPoint.x/resolution), round(goalPoint.y/resolution))
			
			#startPoint.x = 100
			#startPoint.y = 80
			
			
			mmap = NavMap(globMapGrid, goalPoint)
			print "map created"
			

			#~ print startPoint.x
			#~ print startPoint.y
			#~ 
			origin = Node(startPoint)
			#~ 
			pq = PriorityQueue()
			currentNode = origin
			
			while not currentNode.pos.equals(goalPoint) and not rospy.is_shutdown():
				children = mmap.expand(currentNode)
				for node in children:
					pq.put((node.cost(), node))
				
				if(pq.qsize() == 0):
					print "didn't find a path"
					break
				currentNode = pq.get()[1]
				
			
			frontierPub.publish(mmap.getGridCell(frontier))
			exploredPub.publish(mmap.getGridCell(explored))
			freePub.publish(mmap.getGridCell(free))
			blockedPub.publish(mmap.getGridCell(blocked))
			
			path = GridCells()
			path.header.frame_id = "map"
			path.cell_width = resolution
			path.cell_height = resolution
			
			while currentNode.parent is not 0 and not rospy.is_shutdown():
				point = Point()
				point.x = currentNode.pos.x*resolution
				point.y = currentNode.pos.y*resolution
				point.z = 0
				path.cells.append(point)
				currentNode = currentNode.parent
			
			pathPub.publish(path)
			#~ print"expanded"
			#~ 
			#~ print "%d"%origin.pos.x + " %d"%origin.pos.y
			#~ print "a"
			#~ for node in children:
				#~ print "%d"%node.pos.x + " %d"%node.pos.y
			
			
			
			
			#startPathPlanning = False
			newGoal = False
			newStart = False
    	
	rospy.spin()
    
if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInteruptException:
		pass





 		

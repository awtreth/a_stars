#!usr/bin/env python

import rospy, math

from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from Queue import PriorityQueue
from geometry_msgs.msg import PoseStamped
#from enum import Enum

#Helper class
class Point(object):
	def __init__(self, x, y):
		self.x = x
		self.y = y

	#distance
	def dist(self, otherPoint):
		return math.sqrt(self.x**2 + self.y**2)
	#this - other
	def minus(self, otherPoint):
		return Point(self.x-otherPoint.x, self.y-otherPoint.y)
	#this + other
	def plus(self, otherPoint):
		return Point(otherPoint.x+self.x, otherPoint.y+self.y)
		
		
		
free = 0
frontier = 1
explored = 2
blocked = 3

#keep tracks of the state of each cell in the map
class NavMap(object):
	
	def __init__(self, ocmap, goalPoint): #ocmap = OccupancyGrid global_cost_map
		
		
		self.width = ocmap.info.width #just to make it easier to see
		self.height = ocmap.info.height


		#where we store the state of the cells
		self.table = [[free]*self.width]*self.height #At first we fill it all with free
		
		self.goalPoint = goalPoint
		
		for row in range(self.height):
			for col in range(self.width):
				#print row*self.width+col
				if(ocmap.data[row*self.width+col] > 90): #ocmap.data[] is a simple array
					#print self.table[row][col]
					self.table[row][col] = blocked
					#print ocmap.data[row*self.width+col]
				else:
					self.table[row][col] = free
				# else it keeps with free
		

	#return the expanded nodes from the node parent
	def expand(parent): #parent is a node
		#TODO: we can check if parent is in frontier (we can throw an exception)
		table[parent.pos.y][parent.pos.x] = explored
		
		children = [] #return list
		for i in range(-1,1):
			for j in range(-1,1):
				#with abs(i)==abs(j) we just consider turns of 90 degreees
				#try i == j == 0 to include diagonals
				
				pos = parent.plus(Point(i,j)) #we can have problems with the definition of (i,j) (row,col) and (x,y) FIXME
				
				#FIXME: not good code
				if(not self.inBounds(pos)):
					continue
				else:
					if(abs(i)==abs(j) or table[pos.y][pos.x] != free):
						continue
				
				table[pos.y][pos.x] = frontier
				pointDiff = self.goalPoint.minus(parent.pos)
				h_n = abs(pointDiff.x) + abs(pointDiff.y) #h_n = abs(dx) + abs(dy) #for 90 degree turn
				
				children.append(Node(pos, parent, h_n))

		return children
	
	
	def inBounds(self,point):
		if(point.x > 0 and point.x < self.width and point.y > 0 and point.y < self.height):
			return True
		else:
			return False
	
	
class Node(object):
	
	
	def __init__(self, pos):
		self.pos = pos
		self.parent = 0 #check if it is right, because parent is of tyep Node
		self.h_n = 0
		self.g_n = 0
	
	def __init__(self, pos, parent, h_n):
		self.pos = pos
		self.parent = parent
		self.h_n = h_n
		self.g_n = parent.g_n+1

	def cost():
		return self.g_n + self.h_n
		

def globCostCallBack(data):
	global globMapGrid
	global startPathPlanning
	
	globMapGrid = data
	startPathPlanning = True

	print "Got the map"

def readGoal(msg):
	global goalPoint
	global newGoal
	goalPoint = Point(msg.pose.position.x, msg.pose.position.y)
	newGoal = True
	
	#print goalPoint.x
	#print goalPoint.y	

def readOdom(odom):
	global startPoint
	startPoint = Point(odom.pose.pose.position.x, odom.pose.pose.position.y)
<<<<<<< HEAD
	#print startPoint.x
	#print startPoint.y
=======
	print startPoint.x
	print startPoint.y
	
#def pathServClient(self, #start, #goal):
	#all our path stuff
#	rospy.loginfo('waiting for service')
#	rospy.wait_for_service('Path')
#	try:
#		path = rospy.ServiceProxy('Path', Path, True)#(name, service_class, persistent=True)
		
#		globPlanPub = rospy.Publisher("/aStarPath", Path, queue_size=1)
#	except rospy.ServiceException as e:
#		self.fail("No path found: %s"%e)

#	rospy.spin()
>>>>>>> bac456c0ddde1a3632a870f4e6c3a4b4636a2c1b

def run():
	global goalPoint
	global newGoal
	global globMapGrid
	global startPathPlanning
	global startPoint
	
	startPathPlanning = False
	newGoal = False
	
	
	rospy.init_node('lab3')
	
	globCostSub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globCostCallBack)
	globPlanPub = rospy.Publisher("/aStarPath", Path, queue_size=1)
	markerSub = rospy.Subscriber('/aStarNavGoal', PoseStamped, readGoal)
	odomSub = rospy.Subscriber('/odom', Odometry, readOdom)

	frontierPub = rospy.Publisher("/grid_Frontier", GridCells, queue_size=1)
	exploredPub = rospy.Publisher("/grid_explored", GridCells, queue_size=1)
    
    
	sleeper = rospy.Duration(1)
	rospy.sleep(sleeper)
    
    #initialize shit
    
	while not rospy.is_shutdown():
		
		if (startPathPlanning and newGoal):
			
			#print globMapGrid
			
			map = NavMap(globMapGrid, goalPoint)
			print "map created"
			print startPoint
			origin = Node(startPoint)
			print "origin created"
			map.expand(origin)
			print"expanded"
			startPathPlanning = False
			newGoal = False
    	
	rospy.spin()
    
if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInteruptException:
		pass





 		

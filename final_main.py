#!usr/bin/env python

#import libraries
import rospy, math, tf, time

#import ROS msg classes
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import PoseStamped, Point , PoseWithCovarianceStamped, Pose, Quaternion
from nav_msgs.srv import GetPlan
from actionlib_msgs.msg import GoalStatusArray

#Represents a simple 2D point to compute the frontier
class Point2D(object):
    def __init__(self,x=0,y=0):
        self.x = x
        self.y = y

class CustomMap(object):
	#Possible states for each position of the map
	FREE = 0
	UNKNOWN = 2
	OBSTACLE = 1
	
	#Default Constructor
	def __init__(self, globalMap, localMap, threshold = 90):
		
		self.setupPubTopics()

		self.width = globalMap.info.width
		self.height = globalMap.info.height
		self.mmap = [[self.UNKNOWN]*self.width]*self.height
		#it assumes that the orientation of the globalCostMap doesn't change
		self.origin = Point2D(globalMap.info.origin.position.x, globalMap.info.origin.position.y)
		self.resolution = globalMap.info.resolution

	#create the state table
		for y in range(0,self.height):
			offset = y*self.width
			for x in range(0,self.width):
				idx = y*self.width+x
				if globalMap.data[idx] >= threshold:
					#print "obstacle " + repr(x) + ' ' + repr(y)
					self.set(x,y,self.OBSTACLE) #set the obstacles
				else:
					#print "free " + repr(x) + ' ' + repr(y)
					self.set(x,y,self.FREE)
				#if (idx%100 is 0):
				#	raw_input('press Enter')
		
		#print self.mmap
		self.update(localMap)

	# Update the map based on a new localMap
	# Keep the globalMap as it was before
	def update(self, localMap):
		pass

	#Get the state of the specified position
	def get(self,x,y):
		if self.inBounds(x,y):
			return self.mmap[y][x]
		else:
			print "Not inBounds in CustomMap::get method" #TODO: throw exception

	#Set the state of the specified position
	def set(self,x,y, value):
		if self.inBounds(x,y):
			self.mmap[y][x] = value #TODO: check if value is valid
		else:
			print "Not inBounds in CustomMap::set method" #TODO: throw exception
    
	#Check if certain position is in bounds
	def inBounds(self,x,y):
		if(x >= 0 and x < self.width and y >= 0 and y < self.height): return True
		else: return False

	#Return the frontier (list of Points)
	def getFrontier(self):
		frontier = []

		for y in range(self.height):
			for x in range(self.width):
				if(self.get(x,y) == self.UNKNOWN):
					frontier.append(self.getFreeNeighborPoints(x,y))

		return frontier
		
	#helper function
	def getFreeNeighborPoints(self, x, y):
		points = []
		offsets = ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1))
		
		for offset in offsets:
			xx = x+offset[0]
			yy = y+offset[1]
			if self.get(xx, yy) is self.FREE:
				points.append(Point2D(xx,yy))
		
		return points
	
	def setupPubTopics(self):
		self.unknownPub = rospy.Publisher("/unknown_area", GridCells, queue_size=1)
		self.freePub = rospy.Publisher("/free_area", GridCells, queue_size=1)
		self.obstaclePub = rospy.Publisher("/obstacle_area", GridCells, queue_size=1)

	def publishMap(self):
		self.unknownPub.publish(self.getGridCell(self.UNKNOWN))
		self.freePub.publish(self.getGridCell(self.FREE))
		self.obstaclePub.publish(self.getGridCell(self.OBSTACLE))
	
	
	def printMap(self):
		for y in range(self.height):
			line = []
			for x in range(self.width):
				line.append(self.get(x,y))
			
			print line
	
	
	
	# Return the GridCells ROS msg of the cells with state "value"
	def getGridCell(self, value):
		
		#self.printMap()
		
		grid = GridCells()
		grid.header.frame_id = "map"
		grid.cell_width = self.resolution
		grid.cell_height = self.resolution
		
		for y in range(0,self.height):
			for x in range(0,self.width):
				if(self.get(x,y) == value):
					
					point = Point()
					point.x = x*self.resolution+self.origin.x
					point.y = y*self.resolution+self.origin.y
					point.z = 0
					grid.cells.append(point)
					
		return grid

class FrontierProcessor(object):
	def __init__(self):
		self.setupPubTopics()

	def computeCentroid(self, frontier):
		pass

	def publishFrontier(self, frontier):
		pass

	def publishCentroid(self, centroid):
		pass

	def setupPubTopics(self):
		pass



#Callback function that gets robot Status
def readStatus(rosStatus):
	global status
	status = rosStatus.status_list.status

#Callback function that gets the globalCostMap
def readGlobalMap(rosmap):
	global globalMap
	globalMap = rosmap
	#print "global"

#Callback function that gets the localCostMap
def readLocalMap(rosmap):
	global localMap
	localMap = rosmap
	#print "local"

#Main Function
if __name__ == '__main__':
	global status
	global localMap
	global globalMap

	rospy.init_node('final_main')

	#rospy.Subscriber('/move_base/status', GoalStatusArray, readStatus)
	rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, readGlobalMap)
	rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, readLocalMap)


	rospy.sleep(1)

	mmap = CustomMap(globalMap, localMap)    
	
	
	while True:
		mmap.publishMap()
		rospy.sleep(1)
	rospy.spin()



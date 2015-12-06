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
		self.mmap = [self.UNKNOWN]*len(globalMap.data)
		#it assumes that the orientation of the globalCostMap doesn't change
		self.origin = Point2D(globalMap.info.origin.position.x, globalMap.info.origin.position.y)
		self.resolution = globalMap.info.resolution

	#create the state table
		for i in range(0,len(self.mmap)):
			if globalMap.data[i] >= threshold:
				self.mmap[i] = self.OBSTACLE #set the obstacles
			elif globalMap.data[i] < 0:
				self.mmap[i] = self.UNKNOWN
			else:
				self.mmap[i] = self.FREE
		
		self.update(localMap)

	# Update the map based on a new localMap
	# Keep the globalMap as it was before
	def update(self, localMap):
		pass

	#Get the state of the specified position
	def get(self,x,y):
		if self.inBounds(x,y):
			return self.mmap[int(y*self.width+x)]
		else:
			print "Not inBounds in CustomMap::get method" #TODO: throw exception

	#Set the state of the specified position
	def set(self,x,y, value):
		if self.inBounds(x,y):
			self.mmap[int(y*self.width+x)] = value #TODO: check if value is valid
		else:
			print "Not inBounds in CustomMap::set method" #TODO: throw exception
    
	#Check if certain position is in bounds
	def inBounds(self,x,y):
		if(x >= 0 and x < self.width and y >= 0 and y < self.height): return True
		else: return False

	#Return the frontier (list of Points)
	def getFrontier(self):
		frontier = []
		offsets = ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1))

		for y in range(self.height):
			for x in range(self.width):
				if(self.get(x,y) == self.UNKNOWN):
					#create a helper function
					for offset in offsets:
						xx = x+offset[0]
						yy = y+offset[1]
						if(self.inBounds(xx,yy)):
							if self.get(xx, yy) is self.FREE:
								frontier.append(Point2D(xx,yy))

		return frontier
	
	def setupPubTopics(self):
		self.unknownPub = rospy.Publisher("/unknown_area", GridCells, queue_size=1)
		self.freePub = rospy.Publisher("/free_area", GridCells, queue_size=1)
		self.obstaclePub = rospy.Publisher("/obstacle_area", GridCells, queue_size=1)

	def publishMap(self):
		self.unknownPub.publish(self.getGridCell(self.UNKNOWN))
		self.freePub.publish(self.getGridCell(self.FREE))
		self.obstaclePub.publish(self.getGridCell(self.OBSTACLE))	
	
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
		sumx = 0
		sumy = 0
		
		for point in frontier:
			sumx += point.x
			sumy += point.y
		
		return Point2D(sumx/len(frontier), sumy/len(frontier))

	def publishFrontier(self, frontier, resolution, origin):
		self.frontierPub.publish(self.toGridCells(frontier,resolution, origin))

	def publishCentroid(self, centroid,resolution,origin):
		centroidd = []
		centroidd.append(centroid)
		self.centroidPub.publish(self.toGridCells(centroidd,resolution,origin)) 

	def toGridCells(self, frontier, resolution, origin):
		grid = GridCells()
		grid.header.frame_id = "map"
		grid.cell_width = resolution
		grid.cell_height = resolution
		
		for pt in frontier:
			point = Point()
			point.x = pt.x*resolution+origin.x
			point.y = pt.y*resolution+origin.y
			point.z = 0
			grid.cells.append(point)
		
		return grid

	def setupPubTopics(self):
		self.frontierPub = rospy.Publisher("/map_frontier", GridCells, queue_size=1)
		self.centroidPub = rospy.Publisher("/centroid", GridCells, queue_size=1)



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
	
	frontierProcessor = FrontierProcessor()

	mmap = CustomMap(globalMap, localMap)    
	mmap.publishMap()

	frontier = mmap.getFrontier()

	frontierProcessor.publishFrontier(frontier,mmap.resolution,mmap.origin)
	frontierProcessor.publishCentroid(frontierProcessor.computeCentroid(frontier), mmap.resolution, mmap.origin)

	rospy.spin()



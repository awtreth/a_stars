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
	UNKNOWN = -1
	OBSTACLE = 1
	
	#Default Constructor
	def __init__(self, globalMap, localMap, threshold = 99):
		
		self.setupPubTopics()

		self.width = globalMap.info.width
		self.height = globalMap.info.height
		self.map = [self.height][self.width]

	#create the state table
		for row in range(self.height):
			offset = row*self.width
			for col in range(self.width):
				idx = offset+col
				if(globalMap.data[idx] >= threshold):
					self.map[row][col] = self.OBSTACLE #set the obstacles
				elif globalMap.data[idx] == -1:
						self.map[row][col] = self.UNKNOWN
				else:
					self.map[row][col] = self.FREE
		
		self.update(localMap)

	def setupPubTopics(self):
		pass



	# Update the map based on a new localMap
	# Keep the globalMap as it was before
	def update(self, localMap):
		pass

	#Get the state of the specified position
	def get(self,x,y):
		if inBounds(x,y):
			return self.map[y][x]
		else:
			print "Not inBounds in CustomMap::get method" #TODO: throw exception

	#Set the state of the specified position
	def set(self,x,y, value):
		if inBounds(x,y):
			self.map[y][x] = value #TODO: check if value is valid
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
	  
		
	def publishMap(self):
		pass

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

#Callback function that gets the localCostMap
def readLocalMap(rosmap):
	global localMap

#Main Function
if __name__ == '__main__':
	global status

	rospy.Subscriber('/move_base/status', GoalStatusArray, readStatus)
	rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, readGlobalMap)
	rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, readLocalMap)

	rospy.sleep(1)

	mmap = CustomMap(globalMap, localMap)    


	rospy.spin()



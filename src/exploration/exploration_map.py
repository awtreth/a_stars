from point2d import *
from frontier import *
from nav_msgs.msg import GridCells
from map_msgs.msg import OccupancyGridUpdate

import Queue, rospy, math

class ExplorationMap(object):
	#Possible states for each position of the map
	FREE = 0
	UNKNOWN = 2
	OBSTACLE = 1
	FREE_MARKED = 3
	
	#Default Constructor
	def __init__(self, globalMap, updateMap, threshold = 99):
		
		self.width = globalMap.info.width
		self.height = globalMap.info.height
		self.mmap = list(globalMap.data)
		#it assumes that the orientation of the globalCostMap doesn't change
		self.origin = Point2D(globalMap.info.origin.position.x, globalMap.info.origin.position.y)
		self.resolution = globalMap.info.resolution
		self.knownPoints = [] #FIXME: change to knownFreePoints

		if(updateMap is not 0):
			self.update(updateMap)

		#create the state table
		for i in range(0,len(self.mmap)):
			if self.mmap[i] < 0:
				self.mmap[i] = self.UNKNOWN
			elif self.mmap[i] >= threshold:
				self.mmap[i] = self.OBSTACLE #set the obstacles
			else:
				self.knownPoints.append(Point2D(int(math.fmod(i,self.width)),int(i/self.width)))
				self.mmap[i] = self.FREE

	def update(self,update):
		for y in range(0,update.height):
			for x in range(0,update.width):
				self.mmap[(y+update.y)*self.width+(x+update.x)] = update.data[y*update.width+x]

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

	#Return all connected frontiers
	#actually it's not "get". It calculates
	def getAllFrontiers(self):
		bigFrontier = self.getGlobalFrontier()
		
		frontiers = []
		
		while len(bigFrontier.points) > 0 and not rospy.is_shutdown():
			frontier = Frontier()
			self.dfs(bigFrontier.get(0), frontier,[])
			bigFrontier.removePoints(frontier.points)
			frontiers.append(frontier)
		
		return frontiers

	#Return the global frontier (they are not connected)
	def getGlobalFrontier(self):  #now old
		
		frontier = Frontier()
		
		for pt in self.knownPoints:
			if self.hasSpecificNeighboor(pt.x,pt.y,self.UNKNOWN) == True:
				frontier.addPoint(pt)
		
		return frontier

	#return the neighboors that has the "state" value
	def getSpecificNeighboors(self,x,y,state):
		specificNeighboors = []
		
		neighboors = Point2D(x,y).getNeighboors()
		
		for pt in neighboors:
			if self.inBounds(pt.x, pt.y):
				if(self.get(pt.x, pt.y) == state):
					specificNeighboors.append(pt)
					
		return specificNeighboors
	
	#return True if the specified has a neighboor with the specified state
	def hasSpecificNeighboor(self,x,y,state):
		neighboors = Point2D(x,y).getNeighboors()
		
		for pt in neighboors:
			if self.inBounds(pt.x, pt.y):
				if self.get(pt.x, pt.y) == state:
					return True
					
		return False

	#Breadth-First Search. Return the closest Free point that has an unknown neighboor
	#For now it's not feasable for big maps
	def findClosestUnknown(self, initPoint):
		
		q = Queue.Queue()
		q.put(initPoint)
		marked = []
		marked.append(initPoint)
		
		while not q.empty() and not rospy.is_shutdown():
			pt = q.get()
			neighboors = pt.getNeighboors()
			for neighboor in neighboors:
				if(neighboor not in marked):
					marked.append(neighboor)
					value = self.get(neighboor.x, neighboor.y)
					if value is self.FREE:
						q.put(neighboor)
					elif value is self.UNKNOWN:
						return pt #return the free point
		
		print "DONE!"
		return Point2D()

	#Try to find the first free point that has an unknown neighboor and then performs dfs to find connected frontier points
	def getClosestFrontier(self,initPoint):
		print "bfs"
		newInitPoint = self.findClosestUnknown(initPoint)
		print dfs
		frontier = Frontier()
		self.dfs(newInitPoint, frontier, [])
		return frontier
	
	#performs DepthFirstSearch to find frontier points
	#assumes that the first point is a frontier point
	def dfs(self, point, frontier, marked = []):
		
		frontier.addPoint(point)
		marked.append(point);
		neighboors = self.getSpecificNeighboors(point.x, point.y, self.FREE)
		
		for neighboor in neighboors:
			if not neighboor.isInList(marked):
				if self.hasSpecificNeighboor(neighboor.x, neighboor.y, self.UNKNOWN) == True:
					self.dfs(neighboor, frontier, marked)
				else: marked.append(neighboor)
	
	# Return the GridCells ROS msg of the cells with state "value"
	def getGridCell(self, value):
		
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


from point2d import *
from frontier import *
from nav_msgs.msg import GridCells
from map_msgs.msg import OccupancyGridUpdate

import Queue, rospy

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

		if(updateMap is not 0):
			self.update(updateMap)

	#create the state table
		for i in range(0,len(self.mmap)):
			if self.mmap[i] >= threshold:
				self.mmap[i] = self.OBSTACLE #set the obstacles
			elif self.mmap[i] < 0:
				self.mmap[i] = self.UNKNOWN
			else:
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

	#Return the frontier
	def getFrontier(self):  #now old
		
		frontier = Frontier()
		
		for y in range(self.height):
			for x in range(self.width):
				if(self.get(x,y) == self.UNKNOWN):
					frontier.addPoints(self.getFreeNeighboors(x,y))

		return frontier

	def getFreeNeighboors(self, x, y):
		freeNeighboors = []
		
		neighboors = Point2D(x,y).getNeighboors()
		
		for pt in neighboors:
			if(self.get(pt.x, pt.y) is self.FREE):
				freeNeighboors.append(pt)
		
		#~ for pt in freeNeighboors:
			#~ print pt.toString()
		
		return freeNeighboors


	def findClosestUnkown(self, initPoint):
		
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
	
	def dfsFree(self, initPoint):
		q = []
		q.append(initPoint)
		marked = []
		marked.append(initPoint)
		
		
		while len(q)>0 and not rospy.is_shutdown():
			pt = q.pop()
			neighboors = pt.getNeighboors()
			for neighboor in neighboors:
				if(neighboor not in marked):
					marked.append(neighboor)
					value = self.get(neighboor.x, neighboor.y)
					if value is self.FREE:
						q.append(neighboor)
					elif value is self.UNKNOWN:
						return pt #return the free point
		
		print "DONE!"
		return Point2D()

	def getClosestFrontier(self,initPoint):
		print "bfs"
		newInitPoint = self.findClosestUnkown(initPoint)
		#newInitPoint = self.dfsFree(initPoint)
		print "dfs"
		frontier = Frontier()
		self.dfs(newInitPoint, frontier, [])
		print "done"
		return frontier
	
	def dfs(self, point, frontier, marked = []):
		frontier.addPoint(point)
		marked.append(point);
		neighboors = self.getFreeNeighboors(point.x, point.y)
		
		for neighboor in neighboors:
			if not neighboor.isInList(marked):
				if self.hasUnknownNeighboors(neighboor):
					self.dfs(neighboor, frontier, marked)
				else: marked.append(neighboor)
	
	def hasUnknownNeighboors(self, pt):
		neighboors = pt.getNeighboors()
		
		for neighboor in neighboors:
			if self.get(neighboor.x, neighboor.y) is self.UNKNOWN:
				return True
		
		return False
	
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


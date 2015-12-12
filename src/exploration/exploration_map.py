from point2d import *
from frontier import *
from nav_msgs.msg import GridCells

class ExplorationMap(object):
	#Possible states for each position of the map
	FREE = 0
	UNKNOWN = 2
	OBSTACLE = 1
	FREE_MARKED = 3
	
	#Default Constructor
	def __init__(self, globalMap, threshold = 90):
		
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

	def cells(self):
		return self.mmap

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
	def getFrontierOld(self):  #now old
		
		frontier = Frontier()
		
		for y in range(self.height):
			for x in range(self.width):
				if(self.get(x,y) == self.UNKNOWN):
					frontier.addPoints(self.getFreeNeighboors(x,y))

		return frontier

	def getFreeNeighboors(self, x, y):
		neighboors = []
		offsets = ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1))
		
		for offset in offsets:
			xx = x+offset[0]
			yy = y+offset[1]
			if(self.inBounds(xx,yy)):
				if self.get(xx, yy) is self.FREE:
					neighboors.append(Point2D(xx,yy))
		
		return neighboors
		
		
	#width first search
	
	class OrderedSetQueue(Queue.Queue):
	def _init(self, maxsize):
		self.queue = OrderedSet()
	def _put(self, item):
		self.queue.add(item)
	def _get(self):
		return self.queue.pop()


#helper for getFrontier
	def frontExpand(self, x , y):
		neighboors = []
		offsets = ((-1,0),(0,-1),(0,1),(1,0))
		
		self.set(x , y , self.FREE_MARKED)
		
		for offset in offsets:
			xx = x+offset[0]
			yy = y+offset[1]
			if(self.inBounds(xx,yy)):
				if self.get(xx, yy) is self.UNKOWN:
					return Point2d(x,y)
					
				if self.get(xx, yy) is self.FREE:
					neighboors.append(Point2D(xx,yy))
		
		return neighboors
	
	#Takes the x y of robot returns point2d of nearest frontier
	def getFrontier(self, x , y)
		startPoint = Point2(x,y)
		que = OrderedSetQueue(1000)
		que.put(startPoint)
		while not rospy.is_shutdown():
			
			fromQue = que.get()
			
			expanded = frontExpand(fromQue.x , fromQue.y)
			
			if self.get(expanded.x , expanded.y) is self.UNKNOWN
				return expanded
				
			else:
				for point in expanded:
					que.put(point)
				
			
			
			
	
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


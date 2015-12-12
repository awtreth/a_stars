from point2d import *
from frontier import *
from nav_msgs.msg import GridCells

class ExplorationMap(object):
	#Possible states for each position of the map
	FREE = 0
	UNKNOWN = 2
	OBSTACLE = 1
	
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
	def getFrontier(self):
		
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


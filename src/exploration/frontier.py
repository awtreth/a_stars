from point2d import *
from nav_msgs.msg import GridCells
import rospy

class Frontier(object):
	def __init__(self, allPoints = []):
		self.points = []
		self.size = 0
		self.addAllPoints(allPoints)
	
	def get(self, pos):
		return self.points[pos]
	
	def getLast(self):
		return self.points[-1]
	
	def remove(self, pt):
		self.points.remove(pt)
	
	def removePoints(self, pts):
		for pt in pts:
			self.remove(pt)
	
	def copy(self):
		return Frontier(self.points)
	
	def isConnectedTo(self, point):
		for pt in self.points:
			if pt.isConnectedTo(point):
				return True
		return False
	
	def addPoint(self, point):
		self.points.append(point)
		self.size+=1
	
	def addPoints(self, points):
		for point in points:
			self.addPoint(point)
	
	def addAllPoints(self, points):
		self.clear()
		self.addPoints(points)
	
	def clear(self):
		del self.points[:]
		self.size = 0
	
	def closestPointTo(self, to):
		minDist = 99999999999999999
		p = Point2D(to.x, to.y)
		
		for point in self.points:
			dist = point.distTo(to)
			if(dist < minDist):
				minDist = dist
				p.x = point.x
				p.y = point.y
		
		return p
	
	#Return the smallest possible list of Frontiers where each frontier has connected points
	def split(self):
		pass

	def getMiddlePoint(self):
		return self.points[int(self.size/2)]

	def centroid(self):
		
		sumx = 0
		sumy = 0
		
		for point in self.points:
			sumx += point.x
			sumy += point.y

		return Point2D(sumx/self.size, sumy/self.size) # it can throw divisionByZero exception
		
	def toGridCell(self, resolution, origin):
		grid = GridCells()
		grid.header.frame_id = "map"
		grid.cell_width = resolution
		grid.cell_height = resolution
		
		for point in self.points:
			grid.cells.append(point.toRosPoint(resolution,origin))
		
		return grid

	def toString(self):
		string = ""
		for pt in self.points:
			string += pt.toString()
		
		return string


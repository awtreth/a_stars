import tf
from math import hypot
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point, PoseStamped, Quaternion

#Represents a simple 2D point to compute the frontier
class Point2D(object):
	def __init__(self,x=0,y=0):
		self.x = x
		self.y = y
		
		
	def __eq__(self, other):
		return self.equals(other)

	def __ne__(self, other):
		return not self.equals(other)

	def distTo(self, otherPoint):
		return hypot(self.x-otherPoint.x, self.y-otherPoint.y)

	def isConnectedTo(self, otherPoint):
		if int(self.distTo(otherPoint)) >= 2: return False
		return True
	
	def equals(self,otherPoint):
		if self.x == otherPoint.x and self.y == otherPoint.y: return True
		else: return False
	
	
	def getNeighboors(self):
		offsets = ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1))
		
		points = []
		
		for offset in offsets:
			points.append(Point2D(self.x+offset[0], self.y+offset[1]))
			
		return points

	def toRosPoint(self, resolution, origin):
		
		point = Point()
		point.x = self.x*resolution+origin.x
		point.y = self.y*resolution+origin.y
		point.z = 0

		return point
	
	def toGridCell(self, resolution, origin):
		grid = GridCells()
		grid.header.frame_id = "map"
		grid.cell_width = resolution
		grid.cell_height = resolution
		grid.cells.append(self.toRosPoint(resolution,origin))
		
		return grid
	
	def fromPoseStamped(self, pose, resolution, origin):
		self.x = int((pose.pose.position.x- origin.x)/resolution)
		self.y = int((pose.pose.position.y - origin.y)/resolution)
		
		return self

	#angle: euler angle in radians
	def toPoseStamped(self, resolution, origin, angle = 0):
		pose = PoseStamped()
		pose.header.frame_id = "map"
		
		pose.pose.position = self.toRosPoint(resolution, origin)
		quatmsg = Quaternion
		(x,y,z,w) = tf.transformations.quaternion_from_euler(0,0,angle)
		pose.pose.orientation.x = x
		pose.pose.orientation.y = y
		pose.pose.orientation.z = z
		pose.pose.orientation.w = w
		
		return pose
		
	def toString(self):
		return '('+repr(self.x)+','+repr(self.y)+')'

	def isInList(self, checkList):
		for pt in checkList:
			if self.equals(pt): return True
		return False

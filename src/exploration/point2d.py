import tf
from math import hypot
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point, PoseStamped, Quaternion

#Represents a simple 2D point to compute the frontier
class Point2D(object):
	def __init__(self,x=0,y=0):
		self.x = x
		self.y = y

	def distTo(self, otherPoint):
		return hypot(self.x-otherPoint.x, self.y-otherPoint.y)

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

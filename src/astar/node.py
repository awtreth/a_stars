import tf, math

from pose2d import *
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# It represents a possible state of the robot
class Node(object):
	
	# pos - OrientedPoint (x,y,dir)
	# parent - reference to the node that it came from
	# h_n - heuristic function
	def __init__(self, pos, parent = 0 , g_n = 0, h_n = 0):
		if(parent): #if it doesn't have a parent (usually the startPoint)
			self.pos = pos
			self.parent = parent
			self.h_n = h_n
			self.g_n = g_n
		else:
			self.pos = pos
			self.parent = 0 #check if it is right, because parent is of tyep Node
			self.h_n = 0
			self.g_n = 0
			
	# to compute AStar
	def cost(self):
		return self.g_n + self.h_n
	
	def turnLeft(self):
		pos = Pose2D(self.pos.x, self.pos.y, (self.pos.dir-1)%Direction.N_DIRECTIONS)
		return Node(pos, self)
		
	def turnRight(self):
		pos = Pose2D(self.pos.x, self.pos.y, (self.pos.dir+1)%Direction.N_DIRECTIONS)
		return Node(pos, self)
		
	def moveForward(self): #TODO: warning: this is valid just for 90degrees turn
		if(Direction.N_DIRECTIONS is 4):
			if(int(self.pos.dir%2) is 0): #even(west or east)
				#print "even"
				pos = Pose2D(self.pos.x+(self.pos.dir-1),self.pos.y, self.pos.dir)
				return Node(pos,self)
			else: #odd number
				#print "odd"
				pos = Pose2D(self.pos.x,self.pos.y-(self.pos.dir-2), self.pos.dir)
				return Node(pos,self)
		else:
			print "not defined behavior in moveForward for " + repr(Direction.N_DIRECTIONS) + "directions"
			return self

	def expandAll(self):
		all_children = []
		
		all_children.append(self.moveForward())
		all_children.append(self.turnLeft())
		all_children.append(self.turnRight())
		
		return all_children

	#Convert a node (in nCells and Direction) to a Pose(in meters and degrees)
	def toPose(self, resolution, origin):
		pose = PoseStamped()
		pose.header.frame_id = "map"
		point = Point()
		point.x = self.pos.x*resolution + origin.position.x
		point.y = self.pos.y*resolution + origin.position.y
		point.z = 0
		quatTuple = tf.transformations.quaternion_from_euler(0,0,math.pi-self.pos.dir*(2*math.pi/Direction.N_DIRECTIONS));
		pose.pose.orientation = Quaternion(quatTuple[0],quatTuple[1], quatTuple[2], quatTuple[3])
		pose.pose.position = point
		
		return pose

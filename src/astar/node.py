import tf, math

from geometry_msgs.msg import PoseStamped, Point, Quaternion

#custom imports
from pose2d import *

class Node(object):
	"""
	It represents a possible state of the robot
	"""
	
	def __init__(self, pos, parent = 0 , g_n = 0, h_n = 0):
		"""Default Constructor
		pos -- OrientedPoint (x,y,dir)
		parent -- reference to the node that it came from
		g_n -- actual cost to reach it node
		h_n -- heuristic from this node to the goal
		"""
		
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
		"""Returns the resulting node from a turnLeft action
		The turn angle is based on the number of directions in Direction Enum
		"""
		pos = Pose2D(self.pos.x, self.pos.y, (self.pos.dir-1)%Direction.N_DIRECTIONS)
		return Node(pos, self)
		
	def turnRight(self):
		"""Returns the resulting node from a turn right action
		The turn angle is based on the number of directions in Direction Enum
		"""
		pos = Pose2D(self.pos.x, self.pos.y, (self.pos.dir+1)%Direction.N_DIRECTIONS)
		return Node(pos, self)
		
	def moveForward(self, nDirs = Direction.N_DIRECTIONS): #TODO: warning: this is valid just for 90degrees turn
		"""Returns the resulting node from a move forward action
		
		nDirs -- number of directions
		"""
		if(nDirs is 4):
			if(int(self.pos.dir%2) is 0): #even(west or east)
				pos = Pose2D(self.pos.x+(self.pos.dir-1),self.pos.y, self.pos.dir)
				return Node(pos,self)
			else: #odd number (north or south)
				pos = Pose2D(self.pos.x,self.pos.y-(self.pos.dir-2), self.pos.dir)
				return Node(pos,self)
		elif(nDirs is 8):
			pos = Pose2D(self.pos.x,self.pos.y, self.pos.dir)
			
			if pos.dir in (Direction.WEST,Direction.NORTHWEST,Direction.SOUTHWEST):#west
				pos.x-=1
			elif pos.dir in (Direction.EAST, Direction.NORTHEAST, Direction.SOUTHEAST):#east
				pos.x+=1
			if pos.dir in (Direction.NORTHWEST, Direction.NORTH, Direction.NORTHEAST):#north
				pos.y+=1
			elif pos.dir in (Direction.SOUTHWEST, Direction.SOUTH, Direction.SOUTHEAST):#south
				pos.y-=1
			
			return Node(pos,self)
		else:
			print "not defined behavior in moveForward for " + repr(Direction.N_DIRECTIONS) + "directions"
			return self

	def expandAll(self):
		"""Returns a list of the reachable nodes from self
		
		reachable nodes: resulting nodes of moveForward, turnLeft and turnRight methods
		"""
		all_children = []
		
		all_children.append(self.moveForward())
		all_children.append(self.turnLeft())
		all_children.append(self.turnRight())
		
		return all_children

	def toPose(self, resolution, origin):
		"""Convert a node (in nCells and Direction units) to a Pose(in meters and degrees)
			resolution -- ros map resolution (in meters)
			origin -- ros map origin (in meters)
		"""
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

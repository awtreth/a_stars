import math

#DEFINITION OF ENUM
#def enum(*sequential, **named):
#    enums = dict(zip(sequential, range(len(sequential))), **named)
#    return type('Enum', (), enums)
def enum(**enums):
    return type('Enum', (), enums)


#Orientation
#Direction = enum(WEST=0, NORTH=1, EAST=2, SOUTH=3, N_DIRECTIONS=4) #90degree turn (starting with WEST)
Direction = enum(WEST=0, NORTHWEST=1, NORTH=2, NORTHEAST=3, EAST=4, SOUTHEAST=5, SOUTH=6, SOUTHWEST=7, N_DIRECTIONS=8) #45degree turn (starting with WEST)
#Direction = enum(NORTHWEST=0, NORTH=1, NORTHEAST=2, EAST=3, SOUTHEAST=4, SOUTH=5, SOUTHWEST=6, WEST=7, N_DIRECTIONS=8) #90degree turn (starting with NORTHWEST)

#Helper class
class Pose2D(object):
	"""
	2D oriented Point with discrete orientation (based on Direction)
	"""
	
	#constructor
	def __init__(self, x = 0, y = 0, orientation = 0):
		self.x = x
		self.y = y
		self.dir = orientation #west, NORTH, east or south for 90degrees turn

	#distance
	def dist(self, otherPose):
		"""Calculates the distance to otherPose"""
		return math.sqrt((self.x-otherPose.x)**2 + (self.y-otherPose.y)**2)
		
	#this - other (just count x and y dimensions)
	def minus(self, otherPose):
		return Pose2D(self.x-otherPose.x, self.y-otherPose.y)
	
	#this + other (just count x and y dimensions)
	def plus(self, otherPose):
		return Pose2D(otherPose.x+self.x, otherPose.y+self.y)
	
	#consider all dimensions
	def equals(self, otherPose):
		return self.x == otherPose.x and self.y == otherPose.y and self.dir == otherPose.dir
	
	#check just x and y
	def equalPos(self, other):
		return self.x == other.x and self.y == other.y 
	
	#Use it to print out
	def asString(self):
		return "(%d, "%self.x + "%d, "%self.y + "%d)"%self.dir

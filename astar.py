#!usr/bin/env python

#import libraries
import rospy, math, tf

#import ROS msg classes
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from Queue import PriorityQueue
from geometry_msgs.msg import PoseStamped, Point , PoseWithCovarianceStamped, Pose, Quaternion

#CONSTANTS
#the best way that we thought to do this was with Enumeration, but Pythond 2.7 does not support it
#Map points states
free = 0
frontier = 1
explored = 2
blocked = 3
#Orientation
west = 0
north = 1
east = 2
south = 3


#Helper class
#It represents a 2D oriented Point, or pose
class Point(object): #FIXME: actually, it is a Pose (point + orientation)
	
	#constructor
	def __init__(self, x = 0, y = 0, orientation = 0):
		self.x = x
		self.y = y
		self.dir = orientation #west, north, east or south for 90degrees turn

	#distance
	def dist(self, otherPoint):
		return math.sqrt(self.x**2 + self.y**2)
		
	#this - other
	def minus(self, otherPoint):
		return Point(self.x-otherPoint.x, self.y-otherPoint.y)
	
	#this + other
	def plus(self, otherPoint):
		return Point(otherPoint.x+self.x, otherPoint.y+self.y)
	
	#FIXME: check if it is used
	def equals(self, otherPoint):
		return self.x == otherPoint.x and self.y == otherPoint.y
		
		

#keep tracks of the state of each cell in the map
class NavMap(object):
	
	def __init__(self, ocmap, goalPoint): #ocmap = OccupancyGrid
		self.resolution = ocmap.info.resolution
		self.width = ocmap.info.width #just to make it easier to see
		self.height = ocmap.info.height
		
		#where we store the state of the cells
		self.table = [free]*len(ocmap.data) #at first we all with free value
		self.goalPoint = goalPoint
		
		for i in range(len(self.table)):
			if(ocmap.data[i]>90):
				self.table[i] = blocked #set the obstacles

	#return the expanded nodes from the node parent
	def expand(self, parent): #parent is a node
		#TODO: we can check if parent is in frontier (we can throw an exception)
		self.set(parent.pos.x, parent.pos.y, explored) #set the current node as explored
		children = [] #return list
		
		#Scan the neighborhood of the node 
		for i in range(-1,2): #from -1 to +1 (include +1)
			for j in range(-1,2):
				#try i == j == 0 to include diagonals
				
				#set the position
				pos = parent.pos.plus(Point(i,j)) #we can have problems with the definition of (i,j) (row,col) and (x,y) FIXME
				
				if(not self.inBounds(pos)): #if we don't check, we can get an error in the get method of the next condition
					continue #this point cannot be expanded
				else:	# it is above, below or beside (90degree turn) or the position is not free
					if(abs(i)==abs(j) or self.get(pos.x, pos.y) is not free):
						continue #this point cannot be expanded
				
				#0  -1 -> 0 (west)
				#-1  0 -> 1 (north)
				#0   1 -> 2 (east)
				#1   0 -> 3 (south)
				pos.dir = abs(2*i+j+1)
				
				self.set(pos.x, pos.y, frontier) #uptade the position in the table with FRONTIER
				
				pointDiff = self.goalPoint.minus(parent.pos) #get the difference from the goal
				
				h_n = abs(pointDiff.x) + abs(pointDiff.y) #h_n = abs(dx) + abs(dy) #for 90 degree turn
				#TODO: count the number of turns in the heuristic
				
				children.append(Node(pos, parent, h_n)) #add the node into the return array

		return children
	
	# get the state of a gridCell in the map
	def get(self, x, y):
		if(self.inBounds(Point(x,y)) ):
			return self.table[int(y*self.width+x)]
		else:
			return 0 #TODO: throw Exception
	
	# set the state of a position
	def set(self, x, y, value):
		if(self.inBounds(Point(x,y)) ):
			self.table[int(y*self.width+x)] = value
		else:
			return 0 #TODO: throw Exception
	
	# Check if a point is in bounds
	def inBounds(self,point):
		if(point.x > 0 and point.x < self.width and point.y > 0 and point.y < self.height):
			return True
		else:
			return False
	
	# Return the GridCells ROS msg of the cells with state "value"
	def getGridCell(self, value):
		grid = GridCells()
		grid.header.frame_id = "map"
		grid.cell_width = self.resolution
		grid.cell_height = self.resolution
		#grid.cells = []
		
		for y in range(self.height):
			for x in range(self.width):
				if(self.get(x,y) == value):
					point = Point()
					point.x = x*self.resolution
					point.y = y*self.resolution
					point.z = 0
					grid.cells.append(point)
		
		
		return grid
		
	
# It represents a possible state of the robot
class Node(object):
	
	# pos - OrientedPoint (x,y,dir)
	# parent - reference to the node that it came from
	# h_n - heuristic function
	def __init__(self, pos, parent = 0 , h_n = 0):
		if(parent): #if it doesn't have a parent (usually the startPoint)
			self.pos = pos
			self.parent = parent
			self.h_n = h_n
			self.g_n = parent.g_n + 1 + (self.pos.dir+parent.pos.dir)%2
		else:
			self.pos = pos
			self.parent = 0 #check if it is right, because parent is of tyep Node
			self.h_n = 0
			self.g_n = 0
	# to compute AStar
	def cost(self):
		return self.g_n + self.h_n
		
#get the maap
def globCostCallBack(data):
	global globMapGrid
	global startPathPlanning
	global resolution
	globMapGrid = data
	resolution = globMapGrid.info.resolution
	startPathPlanning = True

	print "Got the map"

#get the goal point
def readGoal(msg):
	global goalPoint
	global newGoal
	goalPoint = Point(msg.pose.position.x, msg.pose.position.y)
	newGoal = True
	print "got Goal"

#get the start point
def readStart(msg):
	global startPoint
	global newStart
	startPoint = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)
	newStart = True
	print "got start"
	
	#print goalPoint.x
	#print goalPoint.y	

#Helper function that convert a node to PoseStaped ROS msg
def nodeToPose(node):
	global resolution #this is not a good programming practice, but it works FIXME
	
	pose = PoseStamped()
	pose.header.frame_id = "map"
	point = Point()
	point.x = node.pos.x*resolution
	point.y = node.pos.y*resolution
	point.z = 0
	quatTuple = tf.transformations.quaternion_from_euler(0,0,node.pos.dir*math.pi/2);
	pose.pose.orientation = Quaternion(quatTuple[0],quatTuple[1], quatTuple[2], quatTuple[3])
	pose.pose.position = point
	
	return pose
	
class driver(object) 
	#This function accepts two wheel velocities and a time interval.
	def spinWheels(u1, u2, time):
   		global pub

    		lin_vel = (u1+u2)/2	#Determines the linear velocity of base based on the wheels
    		ang_vel = (u1-u2)/base	#Determines the angular velocity of base on the wheels.
   		twist_msg = Twist();	#Creates two messages: 
    		stop_msg = Twist();
    		twist_msg.linear.x = lin_vel	#Populate messages with data.
    		twist_msg.angular.z = ang_vel
   		stop_msg.linear.x = 0
    		stop_msg.angular.z = 0
    		#While the specified amount of time has not elapsed, send Twist messages.
    		now = rospy.Time.now().secs
    		while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
       			pub.publish(twist_msg)
    			pub.publish(stop_msg)

	#This function consumes linear and angular velocities
	#and creates a Twist message.  This message is then published.
	def publishTwist(lin_vel, ang_vel):
    		global pub

    		twist_msg = Twist();		#Create Twist Message
    		twist_msg.linear.x = lin_vel	#Populate message with data
    		twist_msg.angular.z = ang_vel
    		pub.publish(twist_msg)		#Send Message

	def driveStraight(speed, distance):
    		global odom_list
   		global pose 

   		x0 = pose.pose.position.x	#Set origin
    		y0 = pose.pose.position.y
    		done = False
    		while (not done and not rospy.is_shutdown()):
        		x1 = pose.pose.position.x
        		y1 = pose.pose.position.y
        		d = math.sqrt(math.pow(x1-x0, 2) + math.pow(y1-y0, 2))	#Distance formula
 	
 			if (d >= distance):	# if d is greaterthan or equal to distance val, 
						# make op done. 
           			done = True
            			publishTwist(speed, 0)
			else:			# else publish the movement!
	    			publishTwist(speed, 0)

	#Accepts an angle and makes the robot rotate around it.
	def rotate(angle):
    		global odom_list
    		global pose

    		#This node was created using Coordinate system transforms and numpy arrays.
   		#The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    		transformer = tf.TransformerROS()	
    		rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],	#Create goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

    		#Get transforms for frames
    		odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(3.0))
    		(trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    		T_o_t = transformer.fromTranslationRotation(trans, rot)
   		R_o_t = T_o_t[0:3,0:3]

    		#Setup goal matrix
    		goal_rot = numpy.dot(rotation, R_o_t)
    		goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                    [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                    [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                    [0,             0,             0,             1]])

    		#Continues creating and matching coordinate transforms.
    		done = False
    		while (not done and not rospy.is_shutdown()):
        		(trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        		state = transformer.fromTranslationRotation(trans, rot)
        		within_tolerance = abs((state - goal_o)) < .75
        		if ( within_tolerance.all() ):
            			spinWheels(0,0,0)
            			done = True
        		else:
            			if (angle > 0):
                			spinWheels(.1,-.1,.1)
            			else:
                			spinWheels(-.1,.1,.1)
	def readOdom(msg):
    		global pose
    		global odom_tf

    		pose = msg.pose
    		geo_quat = pose.pose.orientation
  		#This next segment takes the x, y, and z orientations of the robot current pose
    		odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
       		(pose.pose.orientation.x,
		pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),
		rospy.Time.now(),"base_footprint","odom")

#main function
def run():
	global goalPoint
	global newGoal
	global globMapGrid
	global startPathPlanning
	global startPoint
	global resolution
	global newStart
	
	startPathPlanning = False
	newGoal = False
	newStart = False
	
	
	rospy.init_node('lab3')
	
	globCostSub = rospy.Subscriber("/map", OccupancyGrid, globCostCallBack)
	globPlanPub = rospy.Publisher("/aStarPath", Path, queue_size=1)
	markerSub = rospy.Subscriber('/aStarNavGoal', PoseStamped, readGoal)
	startSub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readStart)

	frontierPub = rospy.Publisher("/grid_Frontier", GridCells, queue_size=1)
	exploredPub = rospy.Publisher("/grid_Explored", GridCells, queue_size=1)
	freePub = rospy.Publisher("/grid_Free", GridCells, queue_size=1)
	blockedPub = rospy.Publisher("/grid_Blocked", GridCells, queue_size=1)
	pathPub = rospy.Publisher("/grid_path", GridCells, queue_size=1)
    
    
	sleeper = rospy.Duration(1)
	rospy.sleep(sleeper)
    
	
    
	while not rospy.is_shutdown():
		
		# Trigger to start AStar
		if (startPathPlanning and newGoal and newStart):
			#convert the points from meters to nGridCells
			startPoint = Point(round(startPoint.x/resolution), round(startPoint.y/resolution))
			goalPoint = Point(round(goalPoint.x/resolution), round(goalPoint.y/resolution))

			#create our NavMap based on the read map and the goalPoint
			mmap = NavMap(globMapGrid, goalPoint)
			print "map created"
			
			# Create the first node
			origin = Node(startPoint)
			
			#PriorityQueue of nodes
			pq = PriorityQueue()
			currentNode = origin
			
			while not currentNode.pos.equals(goalPoint) and not rospy.is_shutdown():
				children = mmap.expand(currentNode)
				
				for node in children:
					pq.put((node.cost(), node)) #it is ranked by the cost
				
				if(pq.qsize() == 0):
					print "didn't find a path"
					break
				
				# pop the "closest" node to the goal
				currentNode = pq.get()[1]
			
			# Publish the data in Rviz environment	
			frontierPub.publish(mmap.getGridCell(frontier))
			exploredPub.publish(mmap.getGridCell(explored))
			freePub.publish(mmap.getGridCell(free))
			blockedPub.publish(mmap.getGridCell(blocked))
			
			#Get the path
			path = Path()
			path.header.frame_id = "map"
			path.poses.append(nodeToPose(currentNode))
			
			# Since we store the parent node inside each node and the startNode has null parent
			# We can iterate through it as a LinkedList
			while currentNode.parent is not 0 and not rospy.is_shutdown():
				# Filter to get the wayPoints
				if(currentNode.pos.dir is not currentNode.parent.pos.dir): #if we change the dir
					# the wayPose is the position of the parent and the orientation of the child
					point = Point(currentNode.parent.pos.x, currentNode.parent.pos.y, currentNode.pos.dir)
					newnode = Node(point)
					path.poses.append(nodeToPose(newnode))
				
				currentNode = currentNode.parent #go to the next node
			
			#create WayPoints GridCell
			wayPoints = GridCells()
			wayPoints.header.frame_id = "map"
			wayPoints.cell_width = resolution
			wayPoints.cell_height = resolution
			
			#Create path from the goal to the startPoint
			for pose in path.poses :
				wayPoints.cells.append(pose.pose.position)
			
			#TODO: revert path (to make it from start to the goal"
			
			#publish wayPoints and path
			pathPub.publish(wayPoints)
			globPlanPub.publish(path)
			
			#reset global variables
			#startPathPlanning = False
			newGoal = False
			newStart = False
    	
	rospy.spin()
    
if __name__ == '__main__':
	#try:
	run()
	#except rospy.ROSInteruptException:
		#pass





 		

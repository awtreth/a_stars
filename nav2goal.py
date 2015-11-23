#!/usr/bin/env python

#Libraries
import rospy, tf, sys, time, math, numpy

#Message Types
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

base = .23

#This function consumes linear and angular velocities
#and creates a Twist message.  This message is then published.
def publishTwist(lin_vel, ang_vel):
    global pub

    twist_msg = Twist();		#Create Twist Message

    twist_msg.linear.x = lin_vel	#Populate message with data
    twist_msg.angular.z = ang_vel
    pub.publish(twist_msg)		#Send Message
 
#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pub
    

    lin_vel = (u1+u2)/2			#Determines the linear velocity of base based on the wheels
    ang_vel = (u1-u2)/base		#Determines the angular velocity of base on the wheels.

    twist_msg = Twist();			#Creates two messages: 
    stop_msg = Twist();

    twist_msg.linear.x = lin_vel		#Populate messages with data.
    twist_msg.angular.z = ang_vel
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
    
    #While the specified amount of time has not elapsed, send Twist messages.
    now = rospy.Time.now().secs
    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        pub.publish(twist_msg)
    pub.publish(stop_msg)
    

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose 	

    x0 = pose.pose.position.x	#Set origin
    y0 = pose.pose.position.y

    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specifyed 
    done = False
    while (not done and not rospy.is_shutdown()):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        d = math.sqrt(math.pow(x1-x0, 2) + math.pow(y1-y0, 2))	#Distance formula
        if (d >= distance):
            done = True
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)

    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    #global pose
    global lst

    #This node was created using Coordinate system transforms and numpy arrays.
    #The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    transformer = tf.TransformerROS()	
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],	#Create goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

    #Get transforms for frames
    lst.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = lst.lookupTransform('odom', 'base_footprint', rospy.Time(0))
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
        (trans, rot) = lst.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .275
        if ( within_tolerance.all() ):
            spinWheels(0,0,0)
            done = True
        else:
            if (angle > 0):
                spinWheels(.1,-.1,.1)
            else:
                spinWheels(-.1,.1,.1)


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
	
	global lst #Transform Listener defined in Main
	global br #Transform Broadcaster defined in Main

	#goal is of the type geometry_msgs.msg.PoseStamped
	quat = goal.pose.orientation #quaternion 
	pos = goal.pose.position
	
	#The message has fields x,y,z,(w); but the sendTransform Functions receives tuples
	goal_position = (pos.x, pos.y, 0)
	goal_orientation = (quat.x, quat.y, quat.z, quat.w)	

	#Create the goal frame, related to the map
	goalTime = rospy.Time().now()	
	br.sendTransform(goal_position,goal_orientation,goalTime, "goal", "map")
	
	#Wait until the following transformation is possible
	lst.waitForTransformFull("base_footprint", rospy.Time(0), "goal", goalTime, "map", rospy.Duration(1))
	#TODO: Exception handler
	(trans, rot) = lst.lookupTransformFull("base_footprint", rospy.Time(0), "goal", goalTime, "map") #get the transform between odom and goal
	
	#angle between the first orientation and the final one
	totalAngle = tf.transformations.euler_from_quaternion(rot)[2]
	angle1 = math.atan2(trans[1], trans[0]) #first turn angle
	angle2 = from360to180(totalAngle - angle1) #second turn angle
	
	distance = math.hypot(trans[0], trans[1]) #distance between points (hypo calculates the hypotenuse)
	
	print "spin1 %f"%math.degrees(angle1)
	rotate(angle1)

	print "move %f"%distance
	driveStraight(.35,distance)
	
	print "spin2 %f"%math.degrees(angle2)
	rotate(angle2)

	print "done!"

#this function covert an angle from -360~360 to -180~180
#Examples: 190 -> -170 and -190 -> 170
def from360to180(angle):
	if(abs(angle)>math.pi):
		return angle%math.copysign(math.pi,-angle) #the module of python is different from C/C++ and Java (be careful)
	
	return angle#else

#gets the yaw angle from a quaternionMsg
def yawFromQuatMsg(quat):
	q = [quat.x, quat.y, quat.z, quat.w]
	return (tf.transformations.euler_from_quaternion(q))[2]

def readNavGoal(msg):
	navToPose(msg)
	

#Odometry Callback function.
def readOdom(msg):
    global pose
    global br

    pose = msg.pose
    geo_quat = pose.pose.orientation
  
    br.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
        (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),rospy.Time.now(),"base_footprint","odom")


# This is the program's main function
if __name__ == '__main__':
	
	# Change this node name to include your user name
	rospy.init_node('navToPose')
	
	global pub
	
	#Set Topics
	pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1) # Publisher for commanding robot motion
	rospy.Subscriber('/myGoal', PoseStamped, readNavGoal)
	rospy.Subscriber('/odom', Odometry, readOdom, queue_size=1)

	global lst
	global br	
	br = tf.TransformBroadcaster()
	lst = tf.TransformListener()
	
	time.sleep(1)

	rospy.spin()

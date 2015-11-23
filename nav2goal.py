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

#This function consumes linear and angular velocities
#and creates a Twist message.  This message is then published.
def publishTwist(lin_vel, ang_vel):
    global pub

    twist_msg = Twist();		#Create Twist Message

    twist_msg.linear.x = lin_vel	#Populate message with data
    twist_msg.angular.z = ang_vel
    pub.publish(twist_msg)		#Send Message

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
	br.sendTransform(goal_position,goal_orientation,rospy.Time().now(), "goal", "map")
	
	#Wait until the following transformation is possible
	lst.waitForTransform("base_footprint", "goal", rospy.Time(), rospy.Duration(2))
	#TODO: Exception handler
	(trans, rot) = lst.lookupTransform("base_footprint", "goal", rospy.Time(0)) #get the transform between odom and goal
	
	#angle between the first orientation and the final one
	#totalAngle = tf.transformations.euler_from_quaternion(rot)[2]
	baseGoalAngle = math.atan2(trans[1], trans[0]) #first turn angle
	
	#Wait until the following transformation is possible
	lst.waitForTransform("map", "base_footprint", rospy.Time(), rospy.Duration(2))
	#TODO: Exception handler
	(trans, rot) = lst.lookupTransform("map", "base_footprint", rospy.Time(0)) #get the transform between odom and goal
	
	baseAngle = tf.transformations.euler_from_quaternion(rot)[2]
	
	turn1angle = from360to180(baseAngle + baseGoalAngle)
	turn2angle = yawFromQuatMsg(goal.pose.orientation)
	
	
	print "turn1angle = " + repr(math.degrees(turn1angle))
	turnTo(turn1angle, math.radians(5))
	moveTo(goal.pose.position.x, goal.pose.position.y, .1)
	print "turn2angle = " + repr(math.degrees(turn2angle))
	turnTo(turn2angle, math.radians(5))
	print "done"


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


def turnTo(goalAngle, tolerance):
	global lst
	global br
	
	constant = .7
	max_vel = 1
	min_vel = .2

	
	turnToTime = rospy.Time.now()
	br.sendTransform((1,1,1),tf.transformations.quaternion_from_euler(0,0,goalAngle),turnToTime, "turn_to", "map")
	#raw_input("")
	
	while not rospy.is_shutdown():
		#Wait until the following transformation is possible
		lst.waitForTransformFull("base_footprint", rospy.Time(0), "turn_to", turnToTime, "map", rospy.Duration(2))
		#TODO: Exception handler
		(trans, rot) = lst.lookupTransformFull("base_footprint", rospy.Time(0), "turn_to", turnToTime,"map") #get the transform between odom and goal
		
		currentDiff = tf.transformations.euler_from_quaternion(rot)[2]
		
		#print "currentDiff = " + repr(currentDiff)
		
		if(abs(currentDiff) <= tolerance):
			publishTwist(0,0);
			print "break turn"
			break
		
		w = currentDiff*constant
		#print "w = " + repr(w)
		abs_w = abs(w)
		if(abs_w > max_vel): w = math.copysign(max_vel,w)
		elif(abs_w < min_vel): w = math.copysign(min_vel,w)
		
		publishTwist(0,w);
	

def moveTo(x,y, tolerance):
	global lst
	global br
	
	moveToTime = rospy.Time.now()
	#Create the goal frame, related to the map
	br.sendTransform((x,y,0),(0,0,0,1),moveToTime, "move_to", "map")
	
	kp_w = .5
	max_w = .3
	kp_vel = .5
	max_vel = .4
	min_vel = .15
	
	while not rospy.is_shutdown():
		lst.waitForTransformFull("base_footprint", rospy.Time(0), "move_to", moveToTime, "map", rospy.Duration(2))
		#TODO: Exception handler
		(trans, rot) = lst.lookupTransformFull("base_footprint", rospy.Time(0), "move_to", moveToTime,"map") #get the transform between odom and goal
		dist = math.hypot(trans[0], trans[1])
		#print "dist = " + repr(dist)
		
		if(dist <= tolerance):
			print "break move"
			publishTwist(0,0)
			break
		
		vel = dist*kp_vel
		#print "vel1 = " + repr(vel)
		if(vel > max_vel): vel = max_vel
		elif(vel < min_vel): vel = min_vel
		#print "vel2 = " + repr(vel)
		
		angle = math.atan2(trans[1], trans[0]) #first turn angle
		#print "angle = " + repr(angle)
		w = kp_w*angle
		#print "w1 = " + repr(w)
		if(abs(w) > max_w): w = math.copysign(max_w,w)
		#print "w2 = " + repr(w)
		
		publishTwist(vel,w)
		
	
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

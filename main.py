import rospy, math, tf, time 

#import ROS msg classes
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import PoseStamped, Point , PoseWithCovarianceStamped, Pose, Quaternion
from nav_msgs.srv import GetPlan

#Call Backs
def readNavGoal(msg):
	global goalPose
	global newGoal
	goalPose = msg
	newGoal = True

def readLocalMap(msg):
	global localMap
	global br
	localMap = msg
	
	#goal is of the type geometry_msgs.msg.PoseStamped
	quat = localMap.info.origin.orientation #quaternion 
	pos = localMap.info.origin.position
	
	#The message has fields x,y,z,(w); but the sendTransform Functions receives tuples
	origin_position = (pos.x, pos.y, 0)
	origin_orientation = (quat.x, quat.y, quat.z, quat.w)
	
	br.sendTransform(origin_position,origin_orientation,rospy.Time.now(), "local_map", "map")

#gets the yaw angle from a quaternionMsg
def yawFromQuatMsg(quat):
	q = [quat.x, quat.y, quat.z, quat.w]
	return (tf.transformations.euler_from_quaternion(q))[2]

def inLocalMap(globalPose):
	global localMap
	global lst
	
	#Create the goal frame, related to the map
	#br.sendTransform(origin_position,origin_orientation,rospy.Time.now(), "local_map", "map")
	
	x = globalPose.pose.position.x
	y = globalPose.pose.position.y
	
	globalPose.header.stamp = localMap.header.stamp
	#startPose.header.frame_id = "base_footprint"
	
	transformedPose = lst.transformPose("local_map", globalPose)
	
	resolution = localMap.info.resolution
	width = localMap.info.width
	height = localMap.info.height

	x = round(transformedPose.pose.position.x/resolution)
	y = round(transformedPose.pose.position.y/resolution)
	
	print "x:"+repr(x)+" y:"+repr(y)
	print "w:"+repr(width)+" h:"+repr(height)
	
	if(x<width and x>=0 and y<height and y>=0): return True
	else: return False


if __name__ == '__main__':
	global goalPose
	global newGoal
	global localMap
	
	global br
	global lst
	
	newGoal = False
	
	rospy.init_node('nav')
	
	rospy.Subscriber('/globalGoal', PoseStamped, readNavGoal)
	rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, readLocalMap)
	
	br = tf.TransformBroadcaster()
	lst = tf.TransformListener()

	while not rospy.is_shutdown():
		if(newGoal is True):
			lst.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
			#(trans, rot) = lst.lookupTransform('map', 'base_footprint', rospy.Time(0))
			
			startPose = PoseStamped()
			startPose.header.stamp = rospy.Time.now()
			startPose.header.frame_id = "base_footprint"
			startPose = lst.transformPose("map", startPose)
			
			rospy.wait_for_service('astar_planner')
			getPath = rospy.ServiceProxy('astar_planner', GetPlan)
			
			try:
				path = getPath(startPose, goalPose,0.)
			except rospy.ServiceException as exc:
				print("Service did not process request: " + str(exc))
			
			finalPlan = Path()
			
			lastPose = path.plan.poses[0]
			
			for pose in path.plan.poses:
				#~ if(not inLocalMap(pose)):
					#~ finalPlan.poses.append(lastPose)
					#~ break
					
				if(abs(yawFromQuatMsg(pose.pose.orientation)-yawFromQuatMsg(lastPose.pose.orientation)) > .1): #if we change the dir
					newPose = PoseStamped()
					newPose.pose.orientation = pose.pose.orientation
					newPose.pose.position = lastPose.pose.position
					finalPlan.poses.append(newPose)
				lastPose = pose
			
			#if(finalPlan.poses[-1].pose.position is not goalPose.pose.position):
			finalPlan.poses.append(goalPose)
			
			#now we have the path
			for pose in finalPlan.poses:
				rospy.wait_for_service('nav2goal')
				askMovement = rospy.ServiceProxy('nav2goal', GetPlan) #we are reusing the GetPlan srv msg to save time
				ans = askMovement(startPose,pose,0.)
			
			newGoal = False
			
			
		
		rospy.sleep(1)

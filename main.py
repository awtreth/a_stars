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

if __name__ == '__main__':
	global goalPose
	global newGoal
	newGoal = False
	
	rospy.init_node('nav')
	
	rospy.Subscriber('/myGoal', PoseStamped, readNavGoal)
	
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
			print "passed"
			
			try:
				path = getPath(startPose, goalPose,0.)
			except rospy.ServiceException as exc:
				print("Service did not process request: " + str(exc))
				
			newGoal = False
			
			
		
		rospy.sleep(1)

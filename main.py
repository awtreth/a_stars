import rospy, math, tf, time 

#import ROS msg classes
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry 
from Queue import PriorityQueue
from geometry_msgs.msg import PoseStamped, Point , PoseWithCovarianceStamped, Pose, Quaternion
from nav_msgs.srv import GetPlan




#Call Backs

def readNavGoal(msg):
	global nav
	global goal
	goal = msg
	nav = True


def readOdom(msg):
	global odom
	global start
	global br

	start = odomToPoseStamped(msg.pose)
	odom = True


def run():
	global start #poseStamped
	global goal  #poseStamped
	global path #path
	global nav
	global odom
	
	odom = False
	nav = False
	
	
	
	rospy.init_node('lab3Main')
	
	
	rospy.Subscriber('/odom', Odometry, readOdom, queue_size=1)
	rospy.Subscriber('/myGoal', PoseStamped, readNavGoal)
	
	
	sleeper = rospy.Duration(1)
	rospy.sleep(sleeper)
	
	while not odom and not nav and not rospy.is_shutdown():
		pass
	
	#send things to service 
	#get back path
	rospy.wait_for_service('astar_planner')
	getPath = rospy.ServiceProxy('astar_planner', GetPlan)
	try:
		path = getPath(start, goal)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
		
		
	#navTo goal service
	rospy.wait_for_service('navToGoal')
	navToGoal = rospy.ServiceProxy('navToGoal', GetPlan)
	
	
	pathCount = 0 
	while pathCount < path.length() -1 and not rospy.is_shutdown(): #may need fixing
		
		try:
			navToGoal(path[pathCount], path[pathCount + 1 ])
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		
	
	
	
def odomToPoseStamped(odom):
	ps = poseStamped()
	
	ps.header.frame_id = odom.header.frame_id
	ps.pose = odom.pose.pose






__name__ == '__main__':
	#try:
	run()
	#except rospy.ROSInteruptException:
		#pass

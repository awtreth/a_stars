#!usr/bin/env python

#custom imports
from exploration_ros_input import *
from exploration_map import *
import point2d, exploration_map, frontier, exploration_ros_input

#import libraries
import rospy
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.srv import GetPlan
from std_srvs.srv import Empty
from Queue import PriorityQueue


def requestPath(startPose, goalPose):
	
	rospy.wait_for_service('astar_planner')
	getPlan = rospy.ServiceProxy('astar_planner', GetPlan)
	
	plan = GetPlan()
	
	try:
		plan = getPlan(startPose, goalPose,0.)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
		
	return plan.plan.poses

def clearCostMap():
	rospy.wait_for_service('/move_base/clear_costmaps')
	clear = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
	
	try:
		clear()
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

def checkStatus(rosInput, goalPose):
	goalPoint = Point2D(goalPose.pose.position.x, goalPose.pose.position.y)
	
	endTime = rospy.Time.now() + rospy.Duration(20)
	
	while rospy.Time.now() < endTime and not rospy.is_shutdown():
		robotPose = rosInput.getRobotPose()
		robotPoint = Point2D(robotPose.pose.position.x, robotPose.pose.position.y)
		if(robotPoint.distTo(goalPoint) < .3):
			return 3
		if(rosInput.getStatus() is not 1):
			return rosInput.getStatus()
		#rospy.sleep(.1)
	
	return -1

def rotate(w,time):
	global moveBasePub
	msg = Twist()
	msg.angular.z = w
	
	endTime = rospy.Time.now() + rospy.Duration(time)
	
	while rospy.Time.now() < endTime and not rospy.is_shutdown():
		moveBasePub.publish(msg)
		rospy.sleep(.08)

def filterFrontiers(frontiers,robotPoint,threshold = 7):
	pq = PriorityQueue()

	for frontier in frontiers:
		if frontier.size < threshold:
			continue
		pq.put( (robotPoint.distTo(frontier.centroid()), frontier) )		

	lst = []	
	while not pq.empty() and not rospy.is_shutdown():
		lst.append(pq.get()[1])

	return lst

#Main Function
if __name__ == '__main__':

	rospy.init_node('final_main')

	rosInput = ExplorationRosInput()
	#rosInput.setup()

	globalCentroidTopic = rospy.Publisher("/globalCentroid", GridCells, queue_size=1)
	globalFrontierTopic = rospy.Publisher("/globalFrontier", GridCells, queue_size=1)
	obstaclesTopic = rospy.Publisher("/ExplorationObstacles", GridCells, queue_size=1)
	freeTopic = rospy.Publisher("/ExplorationFree", GridCells, queue_size=1)
	unknownTopic = rospy.Publisher("/ExplorationUnknown", GridCells, queue_size=1)
	goalPub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
	
	global moveBasePub
	moveBasePub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

	rospy.sleep(1)

	#rotate(1,15) #rotate for 10 seconds
	#rotate(-1,45) #rotate for 10 seconds	

	while(not rospy.is_shutdown()):
		print "started"
		rospy.sleep(1)
		print "cleared"
		mmap = ExplorationMap(rosInput.getGlobalMap(), rosInput.getUpdateMap())    
		print "received"
		
		globalFrontierTopic.publish(mmap.getGlobalFrontier().toGridCell(mmap.resolution,mmap.origin))
		#continue
		raw_input()
		#~ frontier = mmap.getClosestFrontier(startPoint)
		#~ print "got frontier"
		#~ rospy.sleep(1)
		
		#centroid = frontier.centroid() #maybe we will not need this
		
		#goalPose = frontier.closestPointTo(startPoint).toPoseStamped(mmap.resolution, mmap.origin, 0)
		startPose = rosInput.getRobotPose()
		startPoint = Point2D().fromPoseStamped(startPose, mmap.resolution, mmap.origin)
		
		#goalPoint = mmap.findClosestUnknown(startPoint)
		#frontier = mmap.getClosestFrontier(startPoint)
		frontiers = mmap.getAllFrontiers()
		print "before"		
		for frontier in frontiers:
			globalFrontierTopic.publish(frontier.toGridCell(mmap.resolution,mmap.origin))
			raw_input()
		print "filtered"
		frontiers = filterFrontiers(frontiers, startPoint)
		for frontier in frontiers:
			globalFrontierTopic.publish(frontier.toGridCell(mmap.resolution,mmap.origin))
			raw_input()
		#continue

		frontier = Frontier()
		for f in frontiers:
			globalFrontierTopic.publish(f.toGridCell(mmap.resolution,mmap.origin))
			print "request path"
			path = requestPath(startPose, f.getMiddlePoint().toPoseStamped(mmap.resolution,mmap.origin))
			print "got path"
		
			if len(path) >= 1: 
				frontier = f
				break
		else:
			print "NO MORE FRONTIERS"
			break


		print "got the closest Frontier"
		
		goalPoint = frontier.getMiddlePoint()
		goalPose = goalPoint.toPoseStamped(mmap.resolution, mmap.origin)

		globalCentroidTopic.publish(goalPoint.toGridCell(mmap.resolution,mmap.origin))
		freeTopic.publish(mmap.getGridCell(3))
		#obstaclesTopic.publish(mmap.getGridCell(1))
		#unknownTopic.publish(mmap.getGridCell(2))
		#globalFrontierTopic.publish(mmap.getClosestFrontier(startPoint).toGridCell(mmap.resolution,mmap.origin))
		
		
		for pose in path:
			goalPub.publish(pose)
			rospy.sleep(1)
			
			status = checkStatus(rosInput, pose)
			
			if status is not 3:
				clearCostMap()
				print "recovery turn"
				rotate(1,3)
				rotate(-1,5)
				break
		
		print "broke"
			

	rospy.spin()



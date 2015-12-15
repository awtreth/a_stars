#!usr/bin/env python

#custom imports
from exploration_ros_input import *
from exploration_map import *
import point2d, exploration_map, frontier, exploration_ros_input

#import libraries
import rospy
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from std_srvs.srv import Empty


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

	rospy.sleep(1)
	
	while(not rospy.is_shutdown()):
		print "started"
		clearCostMap()
		print "cleared"
		mmap = ExplorationMap(rosInput.getGlobalMap(), rosInput.getUpdateMap())    
		print "received"
		
		#~ frontier = mmap.getClosestFrontier(startPoint)
		#~ print "got frontier"
		#~ rospy.sleep(1)
		
		#centroid = frontier.centroid() #maybe we will not need this
		
		#goalPose = frontier.closestPointTo(startPoint).toPoseStamped(mmap.resolution, mmap.origin, 0)
		startPose = rosInput.getRobotPose()
		startPoint = Point2D().fromPoseStamped(startPose, mmap.resolution, mmap.origin)
		
		#goalPoint = mmap.findClosestUnkown(startPoint)
		#goalPose = goalPoint.toPoseStamped(mmap.resolution, mmap.origin)

		#globalCentroidTopic.publish(goalPoint.toGridCell(mmap.resolution,mmap.origin))
		#freeTopic.publish(mmap.getGridCell(0))
		#obstaclesTopic.publish(mmap.getGridCell(1))
		#unknownTopic.publish(mmap.getGridCell(2))
		#globalFrontierTopic.publish(mmap.getClosestFrontier(startPoint).toGridCell(mmap.resolution,mmap.origin))
		frontiers = mmap.getAllFrontiers()
		
		maxF = frontiers[0]
		mmax = frontiers[0].size
		for frontier in frontiers:
			if frontier.size > mmax:
				maxF = frontier
				mmax = frontier.size
		
		globalFrontierTopic.publish(maxF.toGridCell(mmap.resolution,mmap.origin))
		
		rospy.sleep(1)
		
		continue
		
		
		if(goalPoint.x is 0):
			print "clear"
			clearCostMap()
		
		
		path = requestPath(startPose, goalPose)
		
		if len(path)<=1: #no path
			print "no paths"
			clearCostMap()
		
		#globalFrontierTopic.publish(frontier.toGridCell(mmap.resolution,mmap.origin))
		obstaclesTopic.publish(mmap.getGridCell(1))
		
		goalPub.publish(goalPose)
		rospy.sleep(1)
		while rosInput.getStatus() is 1 and not rospy.is_shutdown():
			print "status = 1"
			#if(rosInput.newMap): break
			rospy.sleep(1)
		else:
			print "status = " + repr(rosInput.getStatus())
		continue
		
		#rotate a little bit
		if(rosInput.status is 3):continue
		else: break
		
		for pose in path:
			goalPub.publish(pose)
			rospy.sleep(1)
			
			while rosInput.getStatus() is 1 and not rospy.is_shutdown():
				print "status = 1"
				if(rosInput.newMap): break
				rospy.sleep(1)
			else:
				print "status = " + repr(rosInput.getStatus())
				#rotate a little bit
				if(rosInput.status is 3):continue
				else: break
			break
		
		print "broke"
			

	rospy.spin()



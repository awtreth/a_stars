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
	goalPub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

	rospy.sleep(1)
	
	while(not rospy.is_shutdown()):

		mmap = ExplorationMap(rosInput.getGlobalMap(), rosInput.getUpdateMap())    
		print "received"
		frontier = mmap.getFrontier()
		centroid = frontier.centroid() #maybe we will not need this
		
		startPose = rosInput.getRobotPose()
		goalPose = centroid.toPoseStamped(mmap.resolution, mmap.origin, 0)
		
		path = requestPath(startPose, goalPose)
		
		if len(path)<=1: #no path
			clearCostMap()
		
		globalFrontierTopic.publish(frontier.toGridCell(mmap.resolution,mmap.origin))
		globalCentroidTopic.publish(frontier.centroid().toGridCell(mmap.resolution,mmap.origin))
		
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



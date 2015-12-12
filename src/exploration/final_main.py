#!usr/bin/env python

#custom imports
from exploration_ros_input import *
from exploration_map import *
import point2d, exploration_map, frontier, exploration_ros_input

#import libraries
import rospy
from nav_msgs.msg import GridCells, OccupancyGrid


#Main Function
if __name__ == '__main__':

	rospy.init_node('final_main')

	rosInput = ExplorationRosInput()
	#rosInput.setup()

	globalCentroidTopic = rospy.Publisher("/globalCentroid", GridCells, queue_size=1)
	globalFrontierTopic = rospy.Publisher("/globalFrontier", GridCells, queue_size=1)

	rospy.sleep(1)
	
	while(True and not rospy.is_shutdown()):

		if(rosInput.newMap == True):
			mmap = ExplorationMap(rosInput.getMap())    
			#print "received"

			frontier = mmap.getFrontier()

			globalFrontierTopic.publish(frontier.toGridCell(mmap.resolution,mmap.origin))
			globalCentroidTopic.publish(frontier.centroid().toGridCell(mmap.resolution,mmap.origin))
		
		rospy.sleep(2)

	rospy.spin()



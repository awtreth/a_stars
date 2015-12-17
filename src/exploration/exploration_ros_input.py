import rospy, tf
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray 
from geometry_msgs.msg import PoseStamped 
from map_msgs.msg import OccupancyGridUpdate
from std_srvs.srv import Empty

def clearCostMap():
	rospy.wait_for_service('/move_base/clear_costmaps')
	clear = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
	
	try:
		clear()
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))


class ExplorationRosInput(object):
	
	def __init__(self):
		self.setup()
		
	def setup(self):
		rospy.Subscriber('/move_base/status', GoalStatusArray, self.readStatus)
		rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.readGlobalCostMap)
		rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.readGlobalCostMapUpdates)
		rospy.Subscriber('/map', OccupancyGrid, self.readMap)
		self.lst = tf.TransformListener()
		self.status = 1
		self.newMap = False
		self.newStatus = False
		self.hasUpdate = False
	
	def readGlobalCostMap(self, rosmap):
		print "got new map"
		self.globalMap = rosmap
	
	def readGlobalCostMapUpdates(self, rosmap):
		#print "updated map"
		if rosmap.width == self.globalMap.info.width:
			print "got update"			
			self.updateMap = rosmap
			self.hasUpdate = True
	
	def getMap(self):
		self.newMap = False
		return self.map
		#return self.globalmap
	
	def getUpdateMap(self):
		self.hasUpdate = False
		while self.hasUpdate is False and not rospy.is_shutdown():
			clearCostMap()
			rospy.sleep(1)
		self.hasUpdate = False
		return self.updateMap

	def getGlobalMap(self):
		return self.globalMap
	
	def getStatus(self):
		self.newStatus = False
		return self.status
	
	def getRobotPose(self):
		self.lst.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(5.0))
		#(trans, rot) = lst.lookupTransform('map', 'base_footprint', rospy.Time(0))
		
		startPose = PoseStamped()
		startPose.header.stamp = rospy.Time()
		startPose.header.frame_id = "base_footprint"
		startPose = self.lst.transformPose("map", startPose)
		
		return startPose
	
	#Callback function that gets robot Status
	def readStatus(self,rosStatus):
		if(len(rosStatus.status_list)>0):
			self.status = rosStatus.status_list[-1].status
			self.newStatus = True
	
	#Callback function that gets the globalCostMap
	def readMap(self,rosmap):
		self.map = rosmap
		self.newMap = True
	

import rospy
from nav_msgs.msg import OccupancyGrid

class ExplorationRosInput(object):
	
	def __init__(self):
		self.setup()
		
		
	def setup(self):
		#rospy.Subscriber('/move_base/status', GoalStatusArray, readStatus)
		rospy.Subscriber('/map', OccupancyGrid, self.readGlobalMap)
		rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.readLocalMap)
		self.newLocalMap = False
		self.newGlobalMap = False
		self.newStatus = False
	
	def getLocalMap(self):
		self.newLocalMap = False
		return self.localMap
		
	def getGlobalMap(self):
		self.newGlobalMap = False
		return self.globalMap
		
	def getStatus(self):
		self.newStatus = False
		return self.status
	
	
	#Callback function that gets robot Status
	def readStatus(self,rosStatus):
		self.status = rosStatus.status_list.status
		self.newStatus = True
	
	#Callback function that gets the globalCostMap
	def readGlobalMap(self,rosmap):
		self.globalMap = rosmap
		self.newGlobalMap = True
	
	#Callback function that gets the localCostMap
	def readLocalMap(self,rosmap):
		self.localMap = rosmap
		self.newLocalMap = True
	

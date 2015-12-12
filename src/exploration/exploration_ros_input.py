import rospy
from nav_msgs.msg import OccupancyGrid

class ExplorationRosInput(object):
	
	def __init__(self):
		self.setup()
		
	def setup(self):
		#rospy.Subscriber('/move_base/status', GoalStatusArray, readStatus)
		rospy.Subscriber('/map', OccupancyGrid, self.readMap)
		self.newMap = False
		
	def getMap(self):
		self.newMap = False
		return self.map
		
	def getStatus(self):
		self.newStatus = False
		return self.status
	
	#Callback function that gets robot Status
	def readStatus(self,rosStatus):
		self.status = rosStatus.status_list.status
		self.newStatus = True
	
	#Callback function that gets the globalCostMap
	def readMap(self,rosmap):
		self.map = rosmap
		self.newMap = True
	

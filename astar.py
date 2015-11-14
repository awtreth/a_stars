#!usr/bin/env python

import rospy, math

from nav_msgs.msg import OccupancyGrid, GridCells
from Queue import PriorityQueue
from enum import Enum

class Point(object):
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def dist(self, otherPoint):
		return math.sqrt(self.x**2 + self.y**2)

	def minus(self, otherPoint):
		return Point(self.x-otherPoint.x, self.y-otherPoint.y)
		
	def plus(self, otherPoint)
		return Point(otherPoint.x+self.x, otherPoint.y+self.y)
		

class NavMap(object):
	
	class CellState (Enum):
		free = 0
		frontier = 1
		explored = 2
		blocked = 3
	
	def __init__(self, ocmapE, goalPoint): #ocmap = OccupancyGrid global_cost_map
		
		width = ocmap.info.width
		height = ocmap.info.height
		self.table = [[CellState.free]*width][height]
		
		self.goalPoint = goalPoint
		
		for(col in range(height)):
			for(row in range(width)):
				if(ocmap.data[col*width] > 90):
					self.map[row][col] = CellState.blocked
	
	def expand(parent):
		table[parent.pos.y][parent.pos.x] = CellState.explored
		
		children = []
		
		for(i in range(-1,1)):
			for(j in range(-1,1)):
				if(i==j==0 or table[i][j] != CellState.free):
					continue
				table[i][j] = CellState.frontier
				pos = parent.plus(Point(i,j)) #we can have problems with the definition of (i,j) (row,col) and (x,y) FIXME
				h_n = dist(self.goalPoint.minus(parent.pos), Point(0,0))
				children.append(Node(pos, parent, h_n))
					
		return children
	
	
class Node(object):
	
	def __init__(self, pos, parent, h_n):
		self.pos = point
		self.parent = parent
		self.h_n = h_n
		self.g_n = parent.g_n+1

	def cost():
		return self.g_n + self.h_n
		

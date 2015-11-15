#!usr/bin/env python

import rospy, math

from nav_msgs.msg import OccupancyGrid, GridCells
from Queue import PriorityQueue
from enum import Enum

#Helper class
class Point(object):
	def __init__(self, x, y):
		self.x = x
		self.y = y

	#distance
	def dist(self, otherPoint):
		return math.sqrt(self.x**2 + self.y**2)
	#this - other
	def minus(self, otherPoint):
		return Point(self.x-otherPoint.x, self.y-otherPoint.y)
	#this + other
	def plus(self, otherPoint):
		return Point(otherPoint.x+self.x, otherPoint.y+self.y)
		
#keep tracks of the state of each cell in the map
class NavMap(object):
	#possible cell states
	class CellState (Enum):
		free = 0
		frontier = 1
		explored = 2
		blocked = 3
	
	def __init__(self, ocmapE, goalPoint): #ocmap = OccupancyGrid global_cost_map
		
		width = ocmap.info.width #just to make it easier to see
		height = ocmap.info.height
		
		#where we store the state of the cells
		self.table = [[CellState.free]*width][height] #At first we fill it all with CellState.free
		
		self.goalPoint = goalPoint
		
		for(col in range(height)):
			for(row in range(width)):
				if(ocmap.data[col*width] > 90): #ocmap.data[] is a simple array
					self.map[row][col] = CellState.blocked
				# else it keeps with CellState.free
	
	#return the expanded nodes from the node parent
	def expand(parent): #parent is a node
		#TODO: we can check if parent is in frontier (we can throw an exception)
		table[parent.pos.y][parent.pos.x] = CellState.explored
		
		children = [] #return list
		for(i in range(-1,1)):
			for(j in range(-1,1)):
				#with abs(i)==abs(j) we just consider turns of 90 degreees
				#trye i == j == 0 to include diagonals
				if(abs(i)==abs(j) or table[i][j] != CellState.free):
					continue
				table[i][j] = CellState.frontier
				pos = parent.plus(Point(i,j)) #we can have problems with the definition of (i,j) (row,col) and (x,y) FIXME
				pointDiff = self.goalPoint.minus(parent.pos)
				h_n = abs(pointDiff.x) + abs(pointDiff.y) #h_n = abs(dx) + abs(dy) #for 90 degree turn
				children.append(Node(pos, parent, h_n))
					
		return children
	
	
class Node(object):
	
	
	def __init__(self, pos):
		self.pos = pos
		self.parent = 0 #check if it is right, because parent is of tyep Node
		self.h_n = 0
		self.g_n = 0
	
	def __init__(self, pos, parent, h_n):
		self.pos = pos
		self.parent = parent
		self.h_n = h_n
		self.g_n = parent.g_n+1

	def cost():
		return self.g_n + self.h_n
		

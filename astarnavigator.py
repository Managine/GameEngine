'''
 * Copyright (c) 2014, 2015 Entertainment Intelligence Lab, Georgia Institute of Technology.
 * Originally developed by Mark Riedl.
 * Last edited by Mark Riedl 05/2015
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
'''

import sys, pygame, math, numpy, random, time, copy
from pygame.locals import * 

from constants import *
from utils import *
from core import *
from mycreatepathnetwork import *
from mynavigatorhelpers import *
from nearestgatherer import *


###############################
### AStarNavigator
###
### Creates a path node network and implements the FloydWarshall all-pairs shortest-path algorithm to create a path to the given destination.
			
class AStarNavigator(NavMeshNavigator):

	def __init__(self):
		NavMeshNavigator.__init__(self)
		self.updated = False


	### Create the pathnode network and pre-compute all shortest paths along the network.
	### self: the navigator object
	### world: the world object
	def createPathNetwork(self, world):
		self.pathnodes, self.pathnetwork, self.navmesh = myCreatePathNetwork(world, self.agent)
		return None
		
	### Finds the shortest path from the source to the destination using A*.
	### self: the navigator object
	### source: the place the agent is starting from (i.e., it's current location)
	### dest: the place the agent is told to go to
	def computePath(self, source, dest):
		### Make sure the next and dist matricies exist
		if self.agent != None and self.world != None: 
			self.source = source
			self.destination = dest
			### Step 1: If the agent has a clear path from the source to dest, then go straight there.
			###   Determine if there are no obstacles between source and destination (hint: cast rays against world.getLines(), check for clearance).
			###   Tell the agent to move to dest
			### Step 2: If there is an obstacle, create the path that will move around the obstacles.
			###   Find the pathnodes closest to source and destination.
			###   Create the path by traversing the self.next matrix until the pathnode closes to the destination is reached
			###   Store the path by calling self.setPath()
			###   Tell the agent to move to the first node in the path (and pop the first node off the path)
			if clearShot(source, dest, self.world.getLines(), self.world.getPoints(), self.agent):
				self.agent.moveToTarget(dest)
			else:
				start = findClosestUnobstructed(source, self.pathnodes, self.world.getLinesWithoutBorders())
				end = findClosestUnobstructed(dest, self.pathnodes, self.world.getLinesWithoutBorders())
				if start != None and end != None:
					#print len(self.pathnetwork)
					newnetwork = unobstructedNetwork(self.pathnetwork, self.world.getGates())
					#print len(newnetwork)
					closedlist = []
					path, closedlist = astar(start, end, newnetwork)
					if path is not None and len(path) > 0:
						path = shortcutPath(source, dest, path, self.world, self.agent)
						self.setPath(path)
						if self.path is not None and len(self.path) > 0:
							first = self.path.pop(0)
							self.agent.moveToTarget(first)
		return None
		
	### Called when the agent gets to a node in the path.
	### self: the navigator object
	def checkpoint(self):
		myCheckpoint(self)
		return None

	### This function gets called by the agent to figure out if some shortcutes can be taken when traversing the path.
	### This function should update the path and return True if the path was updated.
	def smooth(self):
		return mySmooth(self)

	def update(self, delta):
		myUpdate(self, delta)


def unobstructedNetwork(network, worldLines):
	newnetwork = []
	for l in network:
		hit = rayTraceWorld(l[0], l[1], worldLines)
		if hit == None:
			newnetwork.append(l)
	return newnetwork

def astar(init, goal, network):
	path = []
	open = {}
	closed = []
	### YOUR CODE GOES BELOW HERE ###
	parent = {}
	thisDistance = {}
	adjacentList = constructAdjacentList(network)
	node = init
	open[init] = [0,0,0]
	while node != goal:
		closed.append(node)
		if not (node in adjacentList):
			return [], closed
		for nextNode in adjacentList[node]:
			if nextNode in closed:
				continue
			distanceToGoal = distance(nextNode, goal)
			distanceToNode = distance(nextNode, node)
			if not (nextNode in open) or open[nextNode][2] >  open[node][0] + open[nextNode][1] + distanceToNode:
				if not (nextNode in open):
					open[nextNode] = [0, distanceToGoal,0]
				open[nextNode][0] = open[node][0] + distanceToNode
				open[nextNode][2] = open[nextNode][0]+open[nextNode][1]
				parent[nextNode] = node
		del open[node]
		nextInfo = getMin(open)
		if nextInfo == None:
			return[], closed
		else:
			node = nextInfo
		
	while node != init:
		path.insert(0,node)
		node = parent[node]
	path.insert(0,init)
	### YOUR CODE GOES ABOVE HERE ###
	return path, closed
	
	
def myUpdate(nav, delta):
	### YOUR CODE GOES BELOW HERE ###
	if not isinstance(nav.agent, Gatherer):
		return
	nav.updated = False
	destination = nav.agent.moveTarget
	source = nav.agent.position
	if rayTraceWorld(source, destination, nav.world.getGates()) != None:
		print "replan"
		nav.agent.setTargets(nav.agent.targets)
		nav.agent.stopMoving()
	# for line in nav.world.getGates():
	# 	if minimumDistance(line, destination) < nav.agent.getMaxRadius():
	# 		print "replan"
	# 		nav.agent.setTargets(nav.agent.targets)
	# 		nav.agent.stopMoving()

	resourcePoint = {}
	for resource in nav.world.resources:
		resourcePoint[resource.position] = 0
	for index, target in enumerate(nav.agent.targets):
		if not (target in resourcePoint):
			nav.agent.targets.pop(index)
			nav.agent.stopMoving()
	### YOUR CODE GOES ABOVE HERE ###
	return None



def myCheckpoint(nav):
	### YOUR CODE GOES BELOW HERE ###
	if not isinstance(nav.agent, Gatherer):
		return
	if nav.updated:
		return;
	nav.updated = True
	tmpTargets = sortTargets(nav.agent.getLocation(), nav.agent.targets)
	if tmpTargets[0] == nav.agent.targets[0]:
		return ;
	nav.agent.setTargets(nav.agent.targets)
	if len(nav.agent.targets)>0:
		nav.computePath(nav.agent.getLocation(), nav.agent.targets[0])
	### YOUR CODE GOES ABOVE HERE ###
	return None


### Returns true if the agent can get from p1 to p2 directly without running into an obstacle.
### p1: the current location of the agent
### p2: the destination of the agent
### worldLines: all the lines in the world
### agent: the Agent object
def clearShot(p1, p2, worldLines, worldPoints, agent):
	### YOUR CODE GOES BELOW HERE ###
	if rayTraceWorld(p1, p2, worldLines) != None:
		return False
	for point in worldPoints:
		if minimumDistance((p1, p2), point) < agent.getMaxRadius():
			return False
	### YOUR CODE GOES ABOVE HERE ###
	return True

def constructAdjacentList(network):
	res = {}
	for edge in network:
		for i in range(2):
			point = edge[i]
			if point in res.keys():
				res[point].append(edge[1-i])
			else:
				res[point] = [edge[1-i]]
	return res

def getMin(open):
	minValue = 999999999.0
	node = None
	for n in open.keys():
		if open[n][2]<=minValue:
			minValue = open[n][2]
			node = n
	return node

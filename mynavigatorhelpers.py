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
from nearestgatherer import *

### This function optimizes the given path and returns a new path
### source: the current position of the agent
### dest: the desired destination of the agent
### path: the path previously computed by the Floyd-Warshall algorithm
### world: pointer to the world
def shortcutPath(source, dest, path, world, agent):
	### YOUR CODE GOES BELOW HERE ###
	for index in range(len(path)):
		if index >= len(path):
			break
		point = path[index]
		tmpPath = path[index+1:]
		for indexR in range(len(tmpPath)):
			if indexR >= len(tmpPath):
				break
			pointR = tmpPath[indexR]
			if not clearShot(point, pointR, world.getLines(), world.getPoints(), agent):
				continue
			path = path[0:index+1] + path[indexR + index + 1:]
			tmpPath = path[index+1:]
			indexR = 0
	### YOUR CODE GOES BELOW HERE ###
	return path


### This function changes the move target of the agent if there is an opportunity to walk a shorter path.
### This function should call nav.agent.moveToTarget() if an opportunity exists and may also need to modify nav.path.
### nav: the navigator object
### This function returns True if the moveTarget and/or path is modified and False otherwise
def mySmooth(nav):
	if nav.path == None:
		return False
	### YOUR CODE GOES BELOW HERE ###
	currentTarget = nav.agent.moveTarget
	if not (currentTarget in nav.path):
		return False
	index = nav.path.index(currentTarget)
	for i in range(len(nav.path)-index-1,0,-1):
		if not clearShot(nav.agent.getLocation(), nav.path[i+index], nav.world.getLines(), nav.world.getPoints, nav.agent):
			continue
		nav.path = [nav.agent.getLocation()] + nav.path[i+index:]
		nav.agent.moveToTarget(nav.path[i+index])
		print "optimized"
		return True
	return False
	### YOUR CODE GOES ABOVE HERE ###

def clearShot(p1, p2, worldLines, worldPoints, agent):
	### YOUR CODE GOES BELOW HERE ###
	if rayTraceWorld(p1, p2, worldLines) != None:
		return False
	for point in worldPoints:
		if minimumDistance((p1, p2), point) < agent.getMaxRadius():
			return False
	### YOUR CODE GOES ABOVE HERE ###
	return True

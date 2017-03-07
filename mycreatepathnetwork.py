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

import sys, pygame, math, numpy, random, time, copy, operator
from pygame.locals import *

from constants import *
from utils import *
from core import *

# Creates a pathnode network that connects the midpoints of each navmesh together
def myCreatePathNetwork(world, agent = None):
	nodes = []
	edges = []
	polys = []
	### YOUR CODE GOES BELOW HERE ###

	hullEdges = []
	obstacleEdges = world.getLinesWithoutBorders()
	totalEdges = world.getLines()
	obstaclesList = world.getObstacles()
	obstacles = []
	maxLength = math.sqrt(world.dimensions[0]**2 + world.dimensions[1]**2)/2
	for obstacle in obstaclesList:
		obstacles.append(obstacle.getPoints())
	points = world.getPoints()
	#points = points[4:]+points[0:3]
	hullEdges.append((points[0],points[1]))
	hullEdges.append((points[1],points[2]))
	hullEdges.append((points[2],points[3]))
	hullEdges.append((points[0],points[3]))

	#construct hull edges
	for index,point in enumerate(points):
		pointsPre = []
		if index!=0:
			pointsPre = points[:index]
		for point1 in points[index+1:]:
			samePolyton = checkOnSamePolyton(point,point1,obstacles)
			if not samePolyton[0]:
				if rayTraceWorldNoEndPoints(point, point1, hullEdges + obstacleEdges)!=None:
					continue
				# if distance(point,point1) > maxLength:
				# 	continue
				hullEdges.append((point,point1))
			elif not samePolyton[1]:
				midPoint = ((point[0]+point1[0])/2, (point[1]+point1[1])/2)
				if pointInsidePolygonPoints(midPoint, samePolyton[2]):
					continue
				if rayTraceWorldNoEndPoints(point, point1, hullEdges + obstacleEdges)!=None:
					continue
				# if distance(point,point1) > maxLength:
				# 	continue
				hullEdges.append((point,point1))
			for point0 in pointsPre:
				if (((point0,point1) in hullEdges) or checkOnSamePolyton(point0,point1,obstacles)[1]) and (((point0,point) in hullEdges) or checkOnSamePolyton(point0,point,obstacles)[1]):
					tmpPolygon = [point0,point,point1]
					if not hasPointInside(obstacles, tmpPolygon):
						polys.append(tmpPolygon)
	#merge polygons
	for i in range(len(polys)):
		if i>=len(polys):
			break
		polygon = polys[i]
		neighbors = getNeighbors(polygon,polys,obstacles)
		while len(neighbors)!=0:
			breakFlag = False
			for j in range(len(neighbors)):
				mergeInfo = neighbors[j]
				# if len(mergeInfo[1])==5:
				# 	print mergeInfo
				# if rayTraceWorldNoEndPoints(mergeInfo[1],mergeInfo[2],obstacleEdges)==None:
				tmpPolygon = mergeInfo[1]
				# print tmpPolygon
				if not isConvex(tmpPolygon):
					continue
				#print tmpPolygon
				#if (mergeInfo[0] in hullEdges):
				if mergeInfo[0] in hullEdges:
					hullEdges.remove(mergeInfo[0])
				elif (mergeInfo[0][1], mergeInfo[0][0]) in hullEdges:
					hullEdges.remove((mergeInfo[0][1], mergeInfo[0][0]))
				polys[i] = tmpPolygon
				polygon = tmpPolygon
				if polys.index(mergeInfo[2]) < i:
					i-=1
				polys.remove(mergeInfo[2])
				breakFlag = True
				break
			if breakFlag:
				neighbors = getNeighbors(polygon,polys,obstacles)
			else:
				break
	#put nodes
	nodesTemp = []
	for edge in hullEdges:
		point = ((edge[0][0]+edge[1][0])/2, (edge[0][1]+edge[1][1])/2)
		if agent == None and getMinDistance(point, totalEdges) < 26:
			continue;
		elif agent!=None and getMinDistance(point, totalEdges) < agent.getMaxRadius()+15:
			continue
		nodesTemp.append((point,edge))
		nodes.append(point)
	for polygon in polys:
		node = [0,0]
		for point in polygon:
			node[0] += point[0]
			node[1] += point[1]
		node = (node[0]/len(polygon),node[1]/len(polygon))
		if agent == None and getMinDistance(point, totalEdges) < 26:
			continue;
		elif agent != None and getMinDistance(node, totalEdges) < agent.getMaxRadius()+15:
			continue
		nodesTemp.append((node,None))
		nodes.append(node)

	#put edges
	for index,point in enumerate(nodes):
		nodesTmp = nodes[index+1:]
		pointR = findClosestUnobstructed(point, nodesTmp, obstacleEdges)
		while pointR != None:
			indexR = nodes.index(pointR)
			if rayTraceWorldNoEndPoints(point,pointR,edges)==None:
				hullEdgesTmp = hullEdges[:]
				if nodesTemp[index][1] in hullEdgesTmp:
					hullEdgesTmp.remove(nodesTemp[index][1])
				if nodesTemp[indexR][1] in hullEdgesTmp:
					hullEdgesTmp.remove(nodesTemp[indexR][1])
				insectPoint = rayTraceWorldNoEndPoints(point, pointR, hullEdgesTmp)
				line = (point,pointR)
				if insectPoint == None:
					if agent == None and getMinDistanceForObstacles(line, obstaclesList)>26:
						edges.append(line)
					elif agent!=None and getMinDistanceForObstacles(line, obstaclesList)>agent.getMaxRadius() + 15:# or almostEqual(insectPoint,point) or almostEqual(insectPoint,pointR):
						edges.append(line)
			nodesTmp.remove(pointR)
			pointR = findClosestUnobstructed(point, nodesTmp, obstacleEdges)

	### YOUR CODE GOES ABOVE HERE ###
	return nodes, edges, polys

def checkOnSamePolyton(point1, point2, polygons):
	for polygon in polygons:
		if point1 in polygon and point2 in polygon:
			index1 = polygon.index(point1)
			index2 = polygon.index(point2)
			if index1 - index2 == 1 or index2 - index1 == 1:
				return [True,True,polygon]
			elif abs(index1-index2) == len(polygon)-1:
				return [True,True,polygon]
			else:
				return [True,False,polygon]
		if point1 in polygon or point2 in polygon:
			return [False,False]
	return False,False
	
def getMinDistance(point, lines):
	minValue = 9999999
	for line in lines:
		minValue = min(minValue, minimumDistance(line,point))
	return minValue

def getNeighbors(polygon, polygons, obstacles):
	if len(polygon)>4:
		return []
	index = [[(0,1),(1,2),(0,2)],[(0,1),(1,0),(1,2),(2,1),(2,3),(3,2),(0,3),(3,0)]]
	res = []
	for i in range(len(index[len(polygon)-3])):
		if i % 2 != 0:
			continue
		p1 = polygon[index[len(polygon)-3][i][0]]
		p2 = polygon[index[len(polygon)-3][i][1]]
		if checkOnSamePolyton(p1,p2,obstacles)[1]:
			continue
		for poly in polygons:
			if poly == polygon:
				continue
			breakFlag = False
			if len(poly)>4:
				continue
			if not polygonsAdjacent(polygon, poly):
				continue
			for j in range(len(index[len(poly)-3])):
				if poly[index[len(poly)-3][j][0]] == p1 and poly[index[len(poly)-3][j][1]] == p2:
					indexI = index[len(polygon)-3][i]
					indexJ = index[len(poly)-3][j]
					tmpPolygon = []
					tmpPolygon.append(p1)
					if indexI[0] < indexI[1]:
						#tmpPolygon.extend(polygon[0:indexI[0]])
						if len(polygon[indexI[0]+1:indexI[1]])==0:
							if polygon[0:indexI[0]]!=None:
								tmp = polygon[0:indexI[0]]
								tmp.reverse()
								tmpPolygon.extend(tmp)
							if polygon[indexI[1]+1:]!=None:
								tmp = polygon[indexI[1]+1:]
								tmp.reverse()
								tmpPolygon.extend(tmp)
						else:
							tmpPolygon.extend(polygon[indexI[0]+1:indexI[1]])
						#tmpPolygon.extend(polygon[indexI[1]+1:])
					else:
						tmpPolygon.extend(polygon[indexI[0]+1:])
						tmpPolygon.extend(polygon[0:indexI[1]])
					tmpPolygon.append(p2)

					if indexJ[0]<indexJ[1]:
						tmp = poly[indexJ[0]+1:indexJ[1]]
						tmp.reverse()
						tmpPolygon.extend(tmp)
						tmpPolygon.extend(poly[indexJ[1]+1:])
						tmpPolygon.extend(poly[0:indexJ[0]])
					else:
						tmp = poly[0:indexJ[1]]
						tmp.reverse()
						tmpPolygon.extend(tmp)
						tmp = poly[indexJ[0]+1:]
						tmp.reverse()
						tmpPolygon.extend(tmp)
						tmpPolygon.extend(poly[indexJ[0]+1:indexJ[1]])
					res.append([(p1,p2),tmpPolygon,poly])#p3,poly[3-index[j][0]-index[j][1]],poly])
					breakFlag = True
					break
			if breakFlag:
				break
	return res

def getMinDistanceForObstacles(line, obstacles):
	minValue = 9999999
	for obstacle in obstacles:
		for point in obstacle.getPoints():
			minValue = min(minValue, minimumDistance(line, point))
	return minValue

def hasPointInside(obstacles, polygon):
	for obstacle in obstacles:
		for point in obstacle:
			if pointInsidePolygonPoints(point, polygon) and not pointOnPolygon(point, polygon):
				return True
	return False

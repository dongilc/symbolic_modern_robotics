#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#file://v-rep-maze.py
"""
Created on Mon Mar 30 18:18:18 2020

@author: MarkAPost

This program goes with the V-REP scene "v-rep-lab.ttt"
It is used to generate random line mazes from
defined maze elements "Start" "End" and "Line"
"""

import sim
import numpy as np
import random

MAXLINES = 200
MAXCOLLISION = 100
MAXSIZE = 4
Directions = {'straight', 'left', 'right'}
Offsets = {'straight':(0.5, 0.0, 0.0), 'left':(0.25, 0.25, 0.0), 'right':(0.25, -0.25, 0.0)}
Angles = {'straight':(0.0, 0.0, 0.0), 'left':(0.0, 0.0, np.pi/2.0), 'right':(0.0, 0.0, -np.pi/2.0)}

#Check if next location for line is clear by forward prediction
def checkLine(clientID, currentLine, marker, floor, offset, rotation):
	res = sim.simxSetObjectPosition(clientID, marker, currentLine, (offset[0]+1.0, offset[1], offset[2]), sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok:  print('Could not set object position')
	res = sim.simxSetObjectOrientation(clientID, marker, currentLine, rotation, sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok:  print('Could not set object orientation')
	res,currentPosition = sim.simxGetObjectPosition(clientID, marker, floor, sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok:  print('Could not get object position')
	currentPosition = (np.around(currentPosition[0], 3), np.around(currentPosition[1], 3), np.around(currentPosition[2], 3))
	gridPosition = (np.around(currentPosition[0]), np.around(currentPosition[1]), np.around(currentPosition[2]))
	res = sim.simxSetObjectPosition(clientID, marker, floor, gridPosition, sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok:  print('Could not set object position')
	return currentPosition, gridPosition

#Create a new line segment by copying the last line segment and moving/rotating it by an offset
def drawLine(clientID, lastline, maze, offset, rotation):
	res,newlines = sim.simxCopyPasteObjects(clientID, [lastline], sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok: print('Could not copypaste line'); return 0
	newline = newlines[0] #Most V-REP API Functions return/accept lists of handles
	res = sim.simxSetObjectParent(clientID, newline, maze, False, sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok:  print('Could not set object parent')
	res = sim.simxSetObjectPosition(clientID, newline, lastline, offset, sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok:  print('Could not set object position')
	res = sim.simxSetObjectOrientation(clientID, newline, lastline, rotation, sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok:  print('Could not set object orientation')
	return newline

#Connect to simulator running on localhost
#V-REP runs on port 19997, a script opens the API on port 19999
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

#Connect to the simulation
if clientID == -1:
	print('Could not connect to remote API server')
	exit()

print('Connected to remote API server')

#Get handles to maze and initial line segment that will be used
res,floor = sim.simxGetObjectHandle(clientID, 'Floor', sim.simx_opmode_oneshot_wait)
if res != sim.simx_return_ok: raise RuntimeError('Could not get handle to Floor');
res,maze = sim.simxGetObjectHandle(clientID, 'Maze', sim.simx_opmode_oneshot_wait)
if res != sim.simx_return_ok: raise RuntimeError('Could not get handle to Maze');
res,line = sim.simxGetObjectHandle(clientID, 'Line', sim.simx_opmode_oneshot_wait)
if res != sim.simx_return_ok: raise RuntimeError('Could not get handle to Line')
res,start = sim.simxGetObjectHandle(clientID, 'Start', sim.simx_opmode_oneshot_wait)
if res != sim.simx_return_ok: raise RuntimeError('Could not get handle to Start')
res,end = sim.simxGetObjectHandle(clientID, 'End', sim.simx_opmode_oneshot_wait)
if res != sim.simx_return_ok: raise RuntimeError('Could not get handle to End')

#Align start point to grid
res,currentPosition = sim.simxGetObjectPosition(clientID, start, floor, sim.simx_opmode_oneshot_wait)
if res != sim.simx_return_ok:  print('Could not get object position')
res = sim.simxSetObjectPosition(clientID, start, floor, (np.around(currentPosition[0]), np.around(currentPosition[1]), 0.002), sim.simx_opmode_oneshot_wait)
if res != sim.simx_return_ok:  print('Could not set object position')
res = sim.simxSetObjectPosition(clientID, line, start, (0.25, 0.0, -0.001), sim.simx_opmode_oneshot_wait)
if res != sim.simx_return_ok:  print('Could not set object position')

#Remove all old copied lines (these show up as names "Line<x>" with x an integer)
print('Removing old maze line segments')
for linecount in range(0, MAXLINES):
	res,oldline = sim.simxGetObjectHandle(clientID, 'Line'+str(linecount), sim.simx_opmode_oneshot_wait)
	if res == sim.simx_return_ok:
		sim.simxRemoveObject(clientID, oldline, sim.simx_opmode_oneshot_wait)
		
#Start at the start point, and build a maze algorithmically by copying the initial "Line"
lineCount = 0
collisionCount = 0
lineHistory = [tuple((line, {random.sample(Directions, 1)[0]}))]
currentLine = lineHistory[-1][0]
currentDirection = next(iter(lineHistory[-1][1]))

#Start a history of locations visited by the maze assumed to describe integer-sized squares from (x,y) to (x+1,y+1)
res,currentPosition = sim.simxGetObjectPosition(clientID, start, floor, sim.simx_opmode_oneshot_wait)
gridPosition = (np.around(currentPosition[0]), np.around(currentPosition[1]), np.around(currentPosition[2]))
positionHistory = [gridPosition]
print('Starting maze generation at location '+str(currentPosition))

#Main maze generation loop
while sim.simxGetConnectionId(clientID) != -1 and lineCount < MAXLINES and collisionCount < MAXCOLLISION:
	#Select where the next line will go
	offset = Offsets[currentDirection]
	rotation = Angles[currentDirection]

	#Check whether the maze has already visited that location by moving the goal as a marker
	currentPosition, gridPosition = checkLine(clientID, currentLine, end, floor, offset, rotation)
	print('Planning to go '+currentDirection+' and checking location '+str(gridPosition)+' from '+str(currentPosition))

	#Check for collisions or off the map and backtrace if one occurs
	if gridPosition in positionHistory or np.abs(gridPosition[0]) > MAXSIZE or np.abs(gridPosition[1]) > MAXSIZE:
		print('Collision at location '+str(gridPosition)+' aligned from '+str(currentPosition))

		#Restart from another part of the maze history, find a part where a new direction is possible and update history with it
		possibleDirections = [line for line in lineHistory if len(line[1]) < len(Directions)]
		if len(possibleDirections) == 0:
			break #we are out of options
		backtrack = random.choice(possibleDirections)
		currentLine = backtrack[0]
		currentDirection = random.sample(Directions - backtrack[1], 1)[0]
		lineHistory[lineHistory.index(backtrack)][1].add(currentDirection)
		collisionCount += 1
	else:
		print('Drawing a maze segment at location '+str(currentPosition)+' with direction '+currentDirection)

		#Draw a new maze segment
		intermediateLine = drawLine(clientID, currentLine, maze, (0.5, 0.0, 0.0), (0.0, 0.0, 0.0))
		newLine = drawLine(clientID, intermediateLine, maze, offset, rotation)
		if intermediateLine == 0 or newLine == 0:
			collisionCount += 1
			continue

		lineCount += 2
		res,currentPosition = sim.simxGetObjectPosition(clientID, newLine, floor, sim.simx_opmode_oneshot_wait)
		gridPosition = (np.around(currentPosition[0]), np.around(currentPosition[1]), np.around(currentPosition[2]))

		#Decide on a direction to take next
		currentLine = newLine
		currentDirection = random.sample(Directions, 1)[0]
		lineHistory.append(tuple((currentLine, {currentDirection})))
		positionHistory.append(tuple(gridPosition))

#Place the end point at the last added line segment in the maze, wherever it is
sim.simxSetObjectPosition(clientID, end, lineHistory[-1][0], (0.75, 0.0, 0.002), sim.simx_opmode_oneshot_wait)
res,currentPosition = sim.simxGetObjectPosition(clientID, end, floor, sim.simx_opmode_oneshot_wait)
gridPosition = (np.around(currentPosition[0], 3), np.around(currentPosition[1], 3), np.around(currentPosition[2], 3))

#Backtrack once if end point collides with something else
if gridPosition in positionHistory or np.abs(gridPosition[0]) > MAXSIZE or np.abs(gridPosition[1]) > MAXSIZE:
	print('Goal Collision at location '+str(gridPosition))
	if len(lineHistory) > 1:
		sim.simxRemoveObject(clientID, lineHistory[-1][0], sim.simx_opmode_oneshot_wait)
		sim.simxSetObjectPosition(clientID, end, lineHistory[-2][0], (0.75, 0.0, 0.002), sim.simx_opmode_oneshot_wait)

sim.simxFinish(clientID)
print('Draw ended')

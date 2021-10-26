#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#file://v-rep-lab.py
"""
Created on Mon Mar 23 18:18:18 2020

@author: MarkAPost

This program goes with the V-REP scene "v-rep-lab.ttt"
It serves as an example for controlling a simple 
differential-drive robot with a camera such as a BubbleRob
"""

import sim
import time
import cv2
import numpy as np

#Program Variables
leftMotorSpeed = 0.0
rightMotorSpeed = 0.0
leftWheelOdom = 0.0
rightWheelOdom = 0.0
lastLeftWheelPosition = 0.0
lastRightWheelPosition = 0.0

#Start Program and just in case, close all opened connections
print('Program started')
sim.simxFinish(-1)

#Connect to simulator running on localhost
#V-REP runs on port 19997, a script opens the API on port 19999
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
thistime = time.time()

#Connect to the simulation
if clientID != -1:
	print('Connected to remote API server')

	#Get handles to simulation objects
	print('Obtaining handles of simulation objects')
	#Overhead camera for navigation
	res,camera = sim.simxGetObjectHandle(clientID, 'Camera', sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok: print('Could not get handle to Camera')
	#Ultrasonic collision sensor optionally used for collision detection
	res,collision = sim.simxGetObjectHandle(clientID, 'collisionSensor', sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok: print('Could not get handle to collisionSensor')
	#Wheel drive motors
	res,leftMotor = sim.simxGetObjectHandle(clientID, 'leftMotor', sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok: print('Could not get handle to leftMotor')
	res,rightMotor = sim.simxGetObjectHandle(clientID, 'rightMotor', sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok: print('Could not get handle to rightMotor')
	#Wheels
	res,leftWheel = sim.simxGetObjectHandle(clientID, 'leftWheel', sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok: print('Could not get handle to leftWheel')
	res,rightWheel = sim.simxGetObjectHandle(clientID, 'rightWheel', sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok: print('Could not get handle to rightWheel')
	#Body
	res,body = sim.simxGetObjectHandle(clientID, 'Robot', sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok: print('Could not get handle to Robot')
	#Floor
	res,floor = sim.simxGetObjectHandle(clientID, 'Floor', sim.simx_opmode_oneshot_wait)
	if res != sim.simx_return_ok: print('Could not get handle to Floor')

	#Start main control loop
	print('Starting control loop')
	res, resolution, image = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_streaming)
	while (sim.simxGetConnectionId(clientID) != -1):
		#Get image from Camera
		lasttime = thistime
		thistime = time.time()
		res, resolution, image = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_buffer)
		if res == sim.simx_return_ok:
			#Process image 
			print("Image OK!", "{:02.1f}".format(1.0 / (thistime - lasttime)), "FPS")
			#Convert from V-REP flat RGB representation to OpenCV BGR colour planes
			#Thanks go to Konstantinos Karapas for correcting the conversions
			original = np.array(image, dtype=np.uint8)
			original.resize([resolution[0], resolution[1], 3])
			original = cv2.flip(original, 0)
			original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)
			#Filter the image into components (alternately, cv2.split includes components of greys, etc.)
			blue = cv2.inRange(original, np.array([224,0,0]), np.array([255,32,32]))
			green = cv2.inRange(original, np.array([0,224,0]), np.array([32,255,32]))
			red = cv2.inRange(original, np.array([0,0,224]), np.array([32,32,255]))
			#Apply Canny edge detection
			blueEdges = cv2.Canny(blue, 32, 64)
			greenEdges = cv2.Canny(green, 32, 64)
			redEdges = cv2.Canny(red, 32, 64)
			#Combine edges from red, green, and blue channels
			edges = cv2.merge((blueEdges, greenEdges, redEdges))
			#Images must all be the same dimensions as reported by original.shape 
			images = np.vstack((original, edges))
			#Show processed images together in OpenCV window
			cv2.imshow('Camera', images)
			components = np.vstack((blue, green, red))
			cv2.imshow('Components', components)


		elif res == sim.simx_return_novalue_flag:
			#Camera has not started or is not returning images
			print("No image yet")
			pass

		else:
			#Something else has happened
			print("Unexpected error returned", res)

		#Get wheel odometry directly from rotary joints in radians
		leftWheelDiameter = \
		sim.simxGetObjectFloatParameter(clientID, leftWheel, 18, sim.simx_opmode_oneshot)[1] \
		- sim.simxGetObjectFloatParameter(clientID, leftWheel, 15, sim.simx_opmode_oneshot)[1]
		rightWheelDiameter = \
		sim.simxGetObjectFloatParameter(clientID, rightWheel, 18, sim.simx_opmode_oneshot)[1] \
		- sim.simxGetObjectFloatParameter(clientID, rightWheel, 15, sim.simx_opmode_oneshot)[1]
		leftWheelPosition = sim.simxGetJointPosition(clientID, leftMotor, sim.simx_opmode_oneshot)[1]
		rightWheelPosition = sim.simxGetJointPosition(clientID, rightMotor, sim.simx_opmode_oneshot)[1]
		#Deal with the joint singularity
		dTheta = leftWheelPosition - lastLeftWheelPosition
		if dTheta > np.pi:
			dTheta -= 2*np.pi
		elif dTheta < -np.pi:
			dTheta += 2*np.pi
		leftWheelOdom += dTheta * leftWheelDiameter / 2
		lastLeftWheelPosition = leftWheelPosition
		dTheta = rightWheelPosition - lastRightWheelPosition
		if dTheta > np.pi:
			dTheta -= 2*np.pi
		elif dTheta < -np.pi:
			dTheta += 2*np.pi
		rightWheelOdom += dTheta * rightWheelDiameter / 2
		lastRightWheelPosition = rightWheelPosition

		#Place your Mobile Robot Control code here

		#Read keypresses for external control (note you must have the OpenCV image window selected when pressing keys!)
		keypress = cv2.waitKey(1) & 0xFF #will read a value of 255 if no key is pressed
		if keypress == ord(' '):
			leftMotorSpeed = 0.0
			rightMotorSpeed = 0.0
		elif keypress == ord('w'):
			leftMotorSpeed += 0.1
			rightMotorSpeed += 0.1
		elif keypress == ord('s'):
			leftMotorSpeed -= 0.1
			rightMotorSpeed -= 0.1
		elif keypress == ord('a'):
			leftMotorSpeed -= 0.1
			rightMotorSpeed += 0.1
		elif keypress == ord('d'):
			leftMotorSpeed += 0.1
			rightMotorSpeed -= 0.1
		elif keypress == ord('q'):
			break

		#Set actuators on mobile robot
		sim.simxSetJointTargetVelocity(clientID, leftMotor, leftMotorSpeed, sim.simx_opmode_oneshot)
		sim.simxSetJointTargetVelocity(clientID, rightMotor, rightMotorSpeed, sim.simx_opmode_oneshot)
		
		#Position tracking information
		position = sim.simxGetObjectPosition(clientID, body, floor, sim.simx_opmode_oneshot)[1]
		orientation = sim.simxGetObjectOrientation(clientID, body, floor, sim.simx_opmode_oneshot)[1]
		print("Pos:[", round(position[0], 2), round(position[1], 2), round(position[2], 2), "]", \
			"Rot:[", round(orientation[0], 2), round(orientation[1], 2), round(orientation[2], 2), "]\n", \
			"LWheelOdom:", "{:03.2f}".format(leftWheelOdom), \
			"RWheelOdom:", "{:03.2f}".format(rightWheelOdom))

	#End simulation
	sim.simxFinish(clientID)

else:
	print('Could not connect to remote API server')

#Close all simulation elements
sim.simxFinish(clientID)
cv2.destroyAllWindows()
print('Simulation ended')

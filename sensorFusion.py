'''
NOTES:
1.	The Euler Angle Calculation functions are being
	called multiple times with the same values. For
	speed, maybe pass the Kalman Filter calculated 
	values right off the bat.
'''
#Imports
import FaBo9Axis_MPU9250
import time
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import os

#Creating Object
sensor = FaBo9Axis_MPU9250.MPU9250()

#Empty list to hold full set of raw data
dataSet = []

#Empty lists to hold raw data
gyroYaw = []
gyroPitch = []
gyroRoll = []

accX = []
accY = []
accZ = []

magX = []
magY = []
magZ = []

#Empty lists to hold Calculated Values
gyroEulerAngle = np.matrix([[0],[0],[0]])
gyroEulerYaw = []
gyroEulerPitch = []
gyroEulerRoll = []

accEulerAngle = np.matrix([[0],[0],[0]])
accEulerYaw = []
accEulerPitch = []
accEulerRoll = []

magEulerAngle = np.matrix([[0],[0],[0]])
magEulerYaw = []
magEulerPitch = []
magEulerRoll = []

#Variables for Calibration Sequence
calAccEuler = np.matrix([[0],[0],[0]])
calMagEuler = np.matrix([[0],[0],[0]])
magPsiValues = []

accXCorrection = 0
accYCorrection = 0
accZCorrection = 0

#Time-Related Variables:
dt = .019 #delta T for integration
times = [] #x-axis for plotting

#Variables for Filtering
A = np.matrix([[1,0,0,-dt,0,0],[0,1,0,0,-dt,0],[0,0,1,0,0,-dt],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
B = np.matrix([[dt,0,0],[0,dt,0],[0,0,dt],[0,0,0],[0,0,0],[0,0,0]])
C = np.matrix([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0]])
P = np.identity(6)
Q = np.identity(6) * .0005
R = np.matrix([[.1,0,0],[0,.1,0],[0,0,10]])
state_estimate = np.matrix([[0],[0],[0],[0],[0],[0]])
filteredData = np.matrix([[0],[0],[0]])
filteredRoll = []
filteredPitch = []
filteredYaw = []


	
#Functions for calculating Euler Angles	
def accCalcEuler(sensorValues):
	phi = math.atan2((sensorValues[5]),(math.sqrt((sensorValues[4])**2 + (sensorValues[6])**2)))
	phi = math.degrees(phi)
	theta = math.atan2((-1*sensorValues[4]),(math.sqrt((sensorValues[5])**2 + (sensorValues[6])**2)))
	theta = math.degrees(theta)
	psi = 0
	eulerEstimate = np.matrix([[phi],[theta],[psi]]) 
	return eulerEstimate

def gyroCalcEuler(sensorValues, prevEulerAngle):
	currentAngularVelocities = np.matrix([[math.radians(sensorValues[1])],[math.radians(sensorValues[2])],[math.radians(sensorValues[3])]])
	phi = prevEulerAngle.item((0,0))
	theta = prevEulerAngle.item((1,0))
	M = np.matrix([[1, (math.tan(theta)*math.sin(phi)), (math.tan(theta)*math.cos(phi))], [0, math.cos(phi), -1*math.sin(phi)], [0, (math.sin(phi)/math.cos(theta)), (math.cos(phi)/math.cos(theta))]])
	eulerAngle = prevEulerAngle + M*currentAngularVelocities*dt
	return eulerAngle

def magCalcEuler(sensorValues, accEulerAngle, magCorrectionMean, gyroEstimate):
	phi = 0
	theta = 0
	accPhi = math.radians(accEulerAngle.item((0,0)))
	accTheta = math.radians(accEulerAngle.item((1,0)))
	psi = math.atan2((sensorValues[9]*math.sin(accPhi) - sensorValues[8]*math.cos(accPhi)),((sensorValues[7]*math.cos(accTheta))+(sensorValues[8]*math.sin(accTheta)*math.sin(accPhi))+(sensorValues[9]*math.sin(accTheta)*math.cos(accPhi))))
	if (magCorrectionMean == 0):
		psi = math.degrees(psi)
	else:
		psi = (math.degrees(psi) - magCorrectionMean)
	eulerEstimate2 = np.matrix([[phi],[theta],[psi]])
	return eulerEstimate2

#Function to collect data and store in temporary list
def getData(currentTime, gyroRollCorrection, gyroPitchCorrection, gyroYawCorrection):
	accel = sensor.readAccel()				#Reads the acceleration list? from the sensor
	gyro = sensor.readGyro()				#Reads the gyro list? from the sensor
	mag = sensor.readMagnet()				#Reads the magnetometer list? from the sensor
	times.append(currentTime)
	
	gyroXCorr = gyro['x'] - gyroRollCorrection
	gyroRoll.append(gyroXCorr)
	gyroYCorr = gyro['y'] - gyroPitchCorrection
	gyroPitch.append(gyroYCorr)
	gyroZCorr = gyro['z'] - gyroYawCorrection
	gyroYaw.append(gyroZCorr)
	accelXCorr = accel['x'] - accXCorrection
	accX.append(accelXCorr)
	accelYCorr = accel['y'] - accYCorrection
	accY.append(accelYCorr)
	accelZCorr = accel['z'] + (9.8005-9.78941028858)
	accZ.append(accelZCorr)
	magXCorr = mag['x'] 
	magX.append(magXCorr)
	magYCorr = mag['y'] 
	magY.append(magYCorr)
	magZCorr = mag['z'] 
	magZ.append(magZCorr)
	sensorValues = [currentTime, gyroXCorr, gyroYCorr, gyroZCorr, accelXCorr, accelYCorr, accelZCorr, magXCorr, magYCorr, magZCorr]
	dataSet.append(sensorValues)
	return sensorValues
	
#Function for Kalman Filtering
def kalmanFilter(sensorValues, magCorrectionMean, gyroYaw):
	#tell python to modify global forms of variables
	global A
	global B
	global C
	global P
	global Q
	global R
	global state_estimate
	
	#Collect Gyro Data
	p = math.radians(sensorValues[1])
	q = math.radians(sensorValues[2])
	r = math.radians(sensorValues[3])
	
	#calculate Angles from measurements
	phi_hat_acc = math.radians(accCalcEuler(sensorValues).item((0,0)))
	theta_hat_acc = math.radians(accCalcEuler(sensorValues).item((1,0)))
		
	phi_hat = state_estimate.item((0,0))
	theta_hat = state_estimate.item((1,0))
	psi_hat = state_estimate.item((2,0))
	
	psi_hat_mag= math.radians(magCalcEuler(sensorValues,accCalcEuler(sensorValues), magCorrectionMean, gyroYaw).item((2,0)))
	#psi_hat_mag = 2 * psi_hat_mag - math.radians(2 * magCorrectionMean)
	
	phi_dot = p+math.sin(phi_hat)*math.tan(theta_hat)*q+math.cos(phi_hat)*math.tan(theta_hat)*r
	theta_dot = math.cos(phi_hat)*q - math.sin(phi_hat)*r
	psi_dot = math.sin(phi_hat)/math.cos(theta_hat)*q + math.cos(phi_hat)/math.cos(theta_hat)*r
	delta_angle = np.matrix([[phi_dot],[theta_dot],[psi_dot]])
	
	#predict actual attitude
	state_estimate = A * state_estimate + B * delta_angle
	P = A*P*np.transpose(A) + Q
	
	#Update readings
	Z = np.matrix([[phi_hat_acc],[theta_hat_acc],[psi_hat_mag]])
	r = Z - C*state_estimate
	S = R + C*P*np.transpose(C)
	K = P*np.transpose(C)*(np.linalg.inv(S)) 
	state_estimate = state_estimate + K*r
	P = (np.identity(6) - K*C) * P;
	state_estimate_degrees = state_estimate * (180/math.pi)
	return state_estimate_degrees
	
	
	
#Calibration Sequence
while True:
	for i in range(100):
		calSensorValues = getData(0,0,0,0)
		calAccEuler = accCalcEuler(calSensorValues)
		calMagEuler = magCalcEuler(calSensorValues, calAccEuler, 0, -50)
		magPsiValues.append(calMagEuler.item((2,0)))
		time.sleep(.009)
	magCorrectionMean = np.mean(magPsiValues)
	print("magCorrectionMean : {:.3f}".format(magCorrectionMean))
	print("")
	gyroRollCorrection = np.mean(gyroRoll)
	print(" gyroRollCorrection : {:.3f}".format(gyroRollCorrection))
	gyroPitchCorrection = np.mean(gyroPitch)
	print(" gyroPitchCorrection : {:.3f}".format(gyroPitchCorrection))
	gyroYawCorrection = np.mean(gyroYaw)
	print("gyroYawCorrection : {:.3f}".format(gyroYawCorrection))
	print("")
	accXCorrection = np.mean(accX)
	print("accXCorrection : {:.3f}".format(accXCorrection))
	accYCorrection = np.mean(accY)
	print("accYCorrection : {:.3f}".format(accYCorrection))
	print("")

	time.sleep(0.5)




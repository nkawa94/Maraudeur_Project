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
def getData(currentTime,gyroRollCorrection, gyroPitchCorrection, gyroYawCorrection):
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
for i in range(100):  
	calSensorValues = getData(0,0,0,0)
	
	calAccEuler = accCalcEuler(calSensorValues)
	calMagEuler = magCalcEuler(calSensorValues, calAccEuler, 0, -20)
	magPsiValues.append(calMagEuler.item((2,0)))
	time.sleep(.009)
	
magCorrectionMean = np.mean(magPsiValues)
	
gyroRollCorrection = np.mean(gyroRoll)
       
gyroPitchCorrection = np.mean(gyroPitch)
	
gyroYawCorrection = np.mean(gyroYaw)
	
accXCorrection = np.mean(accX)
	
accYCorrection = np.mean(accY)
	
	
time.sleep(1)
	
times = []
dataSet = []
	
gyroYaw = []
gyroPitch = []
gyroRoll = []
	
accX = []
accY = []
accZ = []

magX = []
magY = []
magZ = []
	
penul_y = 0
penul_x = 0
f = 0

penul_g_P = 0
penul_g_Y = 0
penul_g_R = 0

#this function return angle and distance
def angle_distance(magX,magY,accY):
    last_y = magY[-1]
    last_x = magX[-1]
    angle = 360*math.atan2(np.mean([last_y,penul_y]),np.mean([last_x,penul_x]))/(math.pi/2)
    penul_y = last_y
    penul_x = last_x
    #parameters calculate distance
    e = accY[-1]*(0.5**2)/2    
    g = np.mean([e,f])
    g *=100 
    distance = math.sqrt((g**2))
    f = e
    return angle, distance

#Unit test 
while True:
    sensorValues = getData(1,gyroRollCorrection,gyroPitchCorrection, gyroYawCorrection)
    gyroEulerAngle = gyroCalcEuler(sensorValues, gyroEulerAngle)
    gyroEulerRoll.append(math.degrees(gyroEulerAngle.item(0,0)))
    gyroEulerPitch.append(math.degrees(gyroEulerAngle.item(1,0)))
    gyroEulerYaw.append(math.degrees(gyroEulerAngle.item(2,0)))
    
    
    gyro = np.mean(gyroEulerAngle)
    gyroRo = np.mean(gyroEulerRoll)
    gyroPi = np.mean(gyroEulerPitch)
    gyroYa = np.mean(gyroEulerYaw)
    
    last_g_P = gyroEulerPitch[-1]
    last_g_Y = gyroEulerYaw[-1]
    last_g_R = gyroRoll[-1]
    an_gy = 180*math.atan2(np.mean([last_g_P,penul_y]),np.mean([last_g_Y,penul_x]))/(math.pi)
    penul_g_P = last_g_P
    penul_g_Y = last_g_Y
    penul_g_R = last_g_R
    
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    last_y = magY[-1]
    
    last_x = magX[-1]
    
    angle = 360*math.atan2(np.mean([last_y,penul_y]),np.mean([last_x,penul_x]))/(math.pi/2)

    
    print("l'angle vaut : {:.3f}".format(angle))
    print("++++++++++++++++-----*************")
    penul_y = last_y
    penul_x = last_x
   
    print("AcceleX : {:.3f}".format(accX[-1]))
    print("AcceleY : {:.3f}".format(accY[-1]))
    print("AcceleZ : {:.3f}".format(accZ[-1]))
    print("::::::::::::::::::::::::::::::::::::::")
    accEulerAngle = accCalcEuler(sensorValues)
    accEulerRoll.append(accEulerAngle.item((0,0)))
    accEulerPitch.append(accEulerAngle.item((1,0)))
    accEulerYaw.append(accEulerAngle.item((2,0)))
    
    acc = np.mean(accEulerAngle)
    accRo = np.mean(accEulerRoll)
    accPi = np.mean(accEulerPitch)
    accYa = np.mean(accEulerYaw)
    
    e = accY[-1]*(0.5**2)/2    
    g = np.mean([e,f])
    
    g *=100 
    
    distance = math.sqrt((g**2))
    f = e
    print("Vous avez parcouru : {:.3f} cm".format(distance))
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("")
    magEulerAngle = magCalcEuler(sensorValues, accEulerAngle,magCorrectionMean, gyroEulerAngle.item((2,0)))
    magEulerRoll.append(magEulerAngle.item((0,0)))
    magEulerPitch.append(magEulerAngle.item((1,0)))
    magEulerYaw.append(magEulerAngle.item((2,0)))
    
    mag = np.mean(magEulerAngle)
    magRo = np.mean(magEulerRoll)
    magPi = np.mean(magEulerPitch)
    magYa = np.mean(magEulerYaw)

    filteredData = kalmanFilter(sensorValues, magCorrectionMean, math.degrees(gyroEulerAngle.item((2,0))))
    filteredRoll.append(filteredData.item((0,0)))
    filteredPitch.append(filteredData.item((1,0)))
    filteredYaw.append(filteredData.item((2,0)))

    filtered = np.mean(filteredData)
    filteredRo = np.mean(filteredRoll)
    filteredPi = np.mean(filteredPitch)
    filteredYa = np.mean(filteredYaw)
    
    

    print("/////////////////////////////////////////")
    print("val gyroRoll = {:.3f}".format(gyroRo))
    print("gyroPitch = {:.3f}".format(gyroPi))
    print("gyroYaw = {:.3f}".format(gyroYa))
    print("angle retourné par gyro = {:.3f}".format(an_gy))
    print("*****************************************")
    print("accRoll = {:.3f}".format(accRo))
    print("accPitch = {:.3f}".format(accPi))
    print("accYaw = {:.3f}".format(accYa))
    print("++++++++++++++++++++++++++++++++++++++++++")
    print("magRoll = {:.3f}".format(magRo))
    print("magPitch = {:.3f}".format(magPi))
    print("magYaw = {:.3f}".format(magYa))
    print("-------------------------------------------")
    print("filter = {:.3f}".format(filtered))
    print("filterRoll = {:.3f}".format(filteredRo))
    print("filterPitch = {:.3f}".format(filteredPi))
    print("filterYaw = {:.3f}".format(filteredYa))
    print("")

    time.sleep(1)


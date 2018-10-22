
import matplotlib
import matplotlib.pyplot as plt
import time
from math import sin, cos
import numpy as np

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

def objectHandle(clientID, objectName, operationMode = vrep.simx_opmode_oneshot_wait):
    error, handle = vrep.simxGetObjectHandle(clientID,objectName,operationMode)
    if error != 0:
        print('Handle {} not found'.format(objectName))
    else:
        print('Connected to {}!'.format(objectName))
    return handle

def localToGlobal(pos, angle, angle_s,  dist, radiusRobot=0.0975):
    xRef = pos[0]
    yRef = pos[1]
    thetaRef = angle[2]

    tTrans = [[1, 0, xRef],
            [0, 1, yRef],
            [0, 0, 1]]

    cos_theta = cos(thetaRef)
    sin_theta = sin(thetaRef)

    tRot = [[cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta, 0],
            [0, 0, 1]]

    xsensor = dist * cos(angle_s[1]) + radiusRobot
    ysensor = dist * sin(angle_s[1]) + radiusRobot
    coord = [xsensor, ysensor, 1]

    localToGlobalMatrix = np.dot(tTrans,tRot)
    resMatrix = np.dot(localToGlobalMatrix, coord)

    return resMatrix[0],resMatrix[1]

def globalToLocal():
    pass

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',25000,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')

    # Handle p3dx
    p3dx = objectHandle(clientID, 'Pioneer_p3dx')

    # Handle of Actuators
    leftMotorHandle = objectHandle(clientID, 'Pioneer_p3dx_leftMotor')
    rightMotorHandle = objectHandle(clientID, 'Pioneer_p3dx_rightMotor')

    # Handle of Sensors
    sonarHandle = []
    for i in range(1,17):
        sonarHandle.append(objectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor{}'.format(i)))

    groundTruthX = []
    groundTruthY = []

    cloudPointX = []
    cloudPointY = []

    j = 0

    try:
        while True:
            print(j)
            j += 1
            if i%100 == 0:
                print("The program ran {} iteractions".format(j))
            vLeft = 1
            vRight = 1
            time.sleep(0.1)
            #vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, vrep.simx_opmode_streaming)
            #vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, vrep.simx_opmode_streaming)


            for i, sH in enumerate(sonarHandle):
                error_pos, pos = vrep.simxGetObjectPosition(clientID, p3dx, -1, vrep.simx_opmode_blocking)
                error_angle, angle = vrep.simxGetObjectOrientation(clientID, p3dx, -1, vrep.simx_opmode_blocking)
                error_angle_s, angle_s = vrep.simxGetObjectOrientation(clientID, sH, p3dx, vrep.simx_opmode_blocking)

                readSonar = vrep.simxReadProximitySensor(clientID, sH, vrep.simx_opmode_blocking)

                groundTruthX.append(pos[0])
                groundTruthY.append(pos[1])
                print(pos[0], pos[1])

                if error_pos != 0 and error_angle != 0:
                    print('Error with sonar {}'.format(i+1))
                else:
                    dist = readSonar[2][2]

                    if readSonar[1] == True:
                        cPX, cPY = localToGlobal(pos, angle, angle_s, dist)
                        cloudPointX.append(cPX)
                        cloudPointY.append(cPY)

    except KeyboardInterrupt:
        # stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)

        plt.plot(groundTruthY, groundTruthX, 'b')
        plt.plot(cloudPointY, cloudPointX, 'ro')
        plt.show()

else:
    print ('Failed connecting to remote API server')
print ('Program ended')

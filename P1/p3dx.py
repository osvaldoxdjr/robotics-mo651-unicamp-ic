# Importing general modules
import matplotlib
import matplotlib.pyplot as plt
import time
from math import sin, cos, pi
import numpy as np

# Importing V-REP Module
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

def objectHandle(clientID, objectName, operationMode=vrep.simx_opmode_oneshot_wait):
    # Get Handle
    error, handle = vrep.simxGetObjectHandle(clientID,objectName,operationMode)
    if error != 0:
        print('Handle {} not found'.format(objectName))
    else:
        print('Connected to {}!'.format(objectName))
    return handle

def localToGlobal(pos, angle, angle_s,  dist, R):
    # Get position in global reference
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

    xsensor = dist * cos(angle_s[1]) + R
    ysensor = dist * sin(angle_s[1]) + R
    coord = [xsensor, ysensor, 1]

    localToGlobalMatrix = np.dot(tTrans,tRot)
    resMatrix = np.dot(localToGlobalMatrix, coord)

    return resMatrix[0],resMatrix[1]

def globalToLocal():
    # Get position in local reference
    pass

def correctEncAngle(angleEnc):
    # Correct angle position of encoders
    if angleEnc >=0:
        return angleEnc
    else:
        return 2*pi+angleEnc

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

    laser2D = objectHandle(clientID, 'LaserScanner_2D')


    # Initializing
    groundTruthX = []
    groundTruthY = []
    odometryX = []
    odometryY = []
    cloudPointX = []
    cloudPointY = []
    R = 0.0975
    L = 0.36205
    j = 0

    errorEncL, iniLeftAngle = vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_streaming)
    errorEncR, iniRightAngle = vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_streaming)
    error_pos, pos = vrep.simxGetObjectPosition(clientID, p3dx, -1, vrep.simx_opmode_streaming)
    error_angle, angle = vrep.simxGetObjectOrientation(clientID, p3dx, -1, vrep.simx_opmode_streaming)
    time.sleep(0.5)

    try:
        while True:
            time.sleep(0.3)
            if j == 0:
                error_pos, pos = vrep.simxGetObjectPosition(clientID, p3dx, -1, vrep.simx_opmode_streaming)
                error_angle, angle = vrep.simxGetObjectOrientation(clientID, p3dx, -1, vrep.simx_opmode_streaming)
                E = [pos[0],pos[1],angle[2]]
                errorEncL, iniLeftAngle = vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_streaming)
                errorEncR, iniRightAngle = vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_streaming)
                start_time = time.time()
                print(error_pos, error_angle, errorEncL, errorEncR)
            else:
                error_pos, pos = vrep.simxGetObjectPosition(clientID, p3dx, -1, vrep.simx_opmode_streaming)
                error_angle, angle = vrep.simxGetObjectOrientation(clientID, p3dx, -1, vrep.simx_opmode_streaming)

                angleEncLeft = vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_streaming)[1]
                angleEncRight = vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_streaming)[1]
                t = time.time() - start_time
                #print(t)
                start_time = time.time()
                angleEncLeft = correctEncAngle(angleEncLeft)
                angleEncRight = correctEncAngle(angleEncRight)

                if 0 <= angleEncLeft <= pi and pi <= iniLeftAngle <= 2*pi or angleEncLeft > iniLeftAngle:
                    leftDiff = pi - abs(abs(angleEncLeft - iniLeftAngle) - pi)
                else:
                    leftDiff = -(pi - abs(abs(angleEncLeft - iniLeftAngle) - pi))

                if 0 <= angleEncRight <= pi and pi <= iniRightAngle <= 2*pi or angleEncRight > iniRightAngle:
                    rightDiff = pi - abs(abs(angleEncRight - iniRightAngle) - pi)
                else:
                    rightDiff = -(pi - abs(abs(angleEncRight - iniRightAngle) - pi))

                print(leftDiff, rightDiff)

                #print('Ground = ',angle[2])
                #print(rightDiff)
                #print(leftDiff)
                #print(angleEncRight, iniRightAngle, angleEncLeft, iniLeftAngle)

                Vl = R*leftDiff/t
                Vr = R*rightDiff/t

                S = ((Vr+Vl)/2)*t
                angT = ((Vr-Vl)/(L))*t
                #print('Delta =',  angT)

                iniLeftAngle = angleEncLeft
                iniRightAngle = angleEncRight

                #gyro = vrep.simxGetFloatSignal(clientID, 'gyroZ', vrep.simx_opmode_blocking)[1]

                E = np.add(E, [S*cos(E[2]+angT/2),S*sin(E[2]+angT/2),angT])
                #print('Odometry = ', E[2])
                #print()

                #print('E',E)
                #print('pos', pos)

            groundTruthX.append(pos[0])
            groundTruthY.append(pos[1])
            odometryX.append(E[0])
            odometryY.append(E[1])

            #print(j)
            j += 1

            vLeft = 1
            vRight = -1

            #vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, vrep.simx_opmode_streaming)
            #vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, vrep.simx_opmode_streaming)

            for i, sH in enumerate(sonarHandle):
                error_pos, pos = vrep.simxGetObjectPosition(clientID, p3dx, -1, vrep.simx_opmode_streaming)
                error_angle, angle = vrep.simxGetObjectOrientation(clientID, p3dx, -1, vrep.simx_opmode_streaming)
                error_angle_s, angle_s = vrep.simxGetObjectOrientation(clientID, sH, p3dx, vrep.simx_opmode_streaming)

                readSonar = vrep.simxReadProximitySensor(clientID, sH, vrep.simx_opmode_streaming)

                if error_pos != 0 and error_angle != 0:
                    print('Error with sonar {}'.format(i+1))
                else:
                    dist = readSonar[2][2]

                    if readSonar[1] == True:
                        cPX, cPY = localToGlobal(pos, angle, angle_s, dist, R)
                        cloudPointX.append(cPX)
                        cloudPointY.append(cPY)

    except KeyboardInterrupt:
        # stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)

        plt.plot(groundTruthY, groundTruthX, 'b')
        plt.plot(odometryY,odometryX,'r-')
        #plt.plot(cloudPointY, cloudPointX, 'go')
        plt.show()

else:
    print ('Failed connecting to remote API server')
print ('Program ended')
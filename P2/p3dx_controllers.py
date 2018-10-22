## Importing general modules
import matplotlib.pyplot as plt
import time
from math import sin, cos, pi
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

#import cv2

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

def charExtract():
    pass
    '''
    # valor mínimo da nuvem de pontos do laser2D
    minimumX = abs(min(laserX))
    minimumY = abs(min(laserY))

    # deslocando os números negativos
    arX = np.array(laserX) + minimumX
    arY = np.array(laserY) + minimumY

    # convertendo os números para inteiros
    def roof(x):
        return int(1000 * x)

    arXmap = list(map(roof, arX))
    arYmap = list(map(roof, arY))

    # representando o mapa em forma de matriz binária
    _2dMap = np.zeros([max(arXmap) + 1, max(arYmap) + 1], dtype=int)

    for i in arXmap:
        for j in arYmap:
            _2dMap[i][j] = 255

    # extraíndo bordas
    edges = cv2.Canny(_2dMap, 50, 150, apertureSize=3)

    # hough
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
    for rho, theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))

        cv2.line(_2dMap, (x1, y1), (x2, y2), (0, 0, 255), 2)'''

def objectHandle(clientID, objectName, operationMode=vrep.simx_opmode_oneshot_wait):
    # Get Handle
    error, handle = vrep.simxGetObjectHandle(clientID,objectName,operationMode)
    if error != 0:
        print('Handle {} not found'.format(objectName))
    else:
        print('Connected to {}!'.format(objectName))
    return handle

def localToGlobal(xObject, yObject, theta, x, y):
    # Get position in global reference
    xRef = xObject
    yRef = yObject
    thetaRef = theta

    tTrans = [[1, 0, xRef],
              [0, 1, yRef],
              [0, 0, 1]]

    cos_theta = cos(thetaRef)
    sin_theta = sin(thetaRef)

    tRot = [[cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta, 0],
            [0, 0, 1]]

    coord = [x, y, 1]

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

    # Create fuzzy control
    #f_s2 = ctrl.Antecedent(np.arange(0, 501, 1), 'f_s2')
    f_s4 = ctrl.Antecedent(np.arange(0, 501, 1), 'f_s4')
    #f_s5 = ctrl.Antecedent(np.arange(0, 501, 1), 'f_s5')
    #f_s7 = ctrl.Antecedent(np.arange(0, 501, 1), 'f_s7')
    #f_s10 = ctrl.Antecedent(np.arange(0, 501, 1), 'f_s10')
    #f_s12 = ctrl.Antecedent(np.arange(0, 501, 1), 'f_s12')
    #f_s13 = ctrl.Antecedent(np.arange(0, 501, 1), 'f_s13')
    #f_s15 = ctrl.Antecedent(np.arange(0, 501, 1), 'f_s15')
    f_vRight = ctrl.Consequent(np.arange(0, 201, 1), 'f_vRight')
    f_vLeft = ctrl.Consequent(np.arange(0, 201, 1), 'f_vLeft')

    # Sensor trampf points
    sensor_mp = [0, 0, 0, 20]
    sensor_p = [0, 20, 90, 100]
    sensor_l = [100, 150, 200, 300]
    sensor_ml = [200, 300, 500, 500]

    # Actuators trampf points
    actuator_l = [0, 0, 0, 50]
    actuator_m = [50, 70, 110, 130]
    actuator_r = [100, 150, 200, 200]
    '''
    f_s2['mp'] = fuzz.trapmf(f_s2.universe, sensor_mp)
    f_s2['p'] = fuzz.trapmf(f_s2.universe, sensor_p)
    f_s2['l'] = fuzz.trapmf(f_s2.universe, sensor_l)
    f_s2['ml'] = fuzz.trapmf(f_s2.universe, sensor_ml)
    '''
    f_s4['mp'] = fuzz.trapmf(f_s4.universe, sensor_mp)
    f_s4['p'] = fuzz.trapmf(f_s4.universe, sensor_p)
    f_s4['l'] = fuzz.trapmf(f_s4.universe, sensor_l)
    f_s4['ml'] = fuzz.trapmf(f_s4.universe, sensor_ml)
    '''
    f_s5['mp'] = fuzz.trapmf(f_s5.universe, sensor_mp)
    f_s5['p'] = fuzz.trapmf(f_s5.universe, sensor_p)
    f_s5['l'] = fuzz.trapmf(f_s5.universe, sensor_l)
    f_s5['ml'] = fuzz.trapmf(f_s5.universe, sensor_ml)

    f_s7['mp'] = fuzz.trapmf(f_s7.universe, sensor_mp)
    f_s7['p'] = fuzz.trapmf(f_s7.universe, sensor_p)
    f_s7['l'] = fuzz.trapmf(f_s7.universe, sensor_l)
    f_s7['ml'] = fuzz.trapmf(f_s7.universe, sensor_ml)

    f_s10['mp'] = fuzz.trapmf(f_s10.universe, sensor_mp)
    f_s10['p'] = fuzz.trapmf(f_s10.universe, sensor_p)
    f_s10['l'] = fuzz.trapmf(f_s10.universe, sensor_l)
    f_s10['ml'] = fuzz.trapmf(f_s10.universe, sensor_ml)

    f_s12['mp'] = fuzz.trapmf(f_s12.universe, sensor_mp)
    f_s12['p'] = fuzz.trapmf(f_s12.universe, sensor_p)
    f_s12['l'] = fuzz.trapmf(f_s12.universe, sensor_l)
    f_s12['ml'] = fuzz.trapmf(f_s12.universe, sensor_ml)

    f_s13['mp'] = fuzz.trapmf(f_s13.universe, sensor_mp)
    f_s13['p'] = fuzz.trapmf(f_s13.universe, sensor_p)
    f_s13['l'] = fuzz.trapmf(f_s13.universe, sensor_l)
    f_s13['ml'] = fuzz.trapmf(f_s13.universe, sensor_ml)

    f_s15['mp'] = fuzz.trapmf(f_s15.universe, sensor_mp)
    f_s15['p'] = fuzz.trapmf(f_s15.universe, sensor_p)
    f_s15['l'] = fuzz.trapmf(f_s15.universe, sensor_l)
    f_s15['ml'] = fuzz.trapmf(f_s15.universe, sensor_ml)
    '''
    f_vRight['l'] = fuzz.trapmf(f_vRight.universe, actuator_l)
    f_vRight['m'] = fuzz.trapmf(f_vRight.universe, actuator_m)
    f_vRight['r']= fuzz.trapmf(f_vRight.universe, actuator_r)

    f_vLeft['l'] = fuzz.trapmf(f_vLeft.universe, actuator_l)
    f_vLeft['m'] = fuzz.trapmf(f_vLeft.universe, actuator_m)
    f_vLeft['r'] = fuzz.trapmf(f_vLeft.universe, actuator_r)


    rule1 = ctrl.Rule(f_s4['l'] | f_s4['ml'], f_vRight['r'])
    rule2 = ctrl.Rule(f_s4['l'] | f_s4['ml'], f_vLeft['r'])
    rule3 = ctrl.Rule(f_s4['p'] | f_s4['mp'], f_vRight['r'])
    rule4 = ctrl.Rule(f_s4['p'] | f_s4['mp'], f_vLeft['l'])

    Rules = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
    vRight_out = ctrl.ControlSystemSimulation(Rules)
    vLeft_out = ctrl.ControlSystemSimulation(Rules)


    vRight_out.input['f_s4'] = 125
    vLeft_out.input['f_s4'] = 125
    vRight_out.compute()
    vLeft_out.compute()
    print('\n\n\n')

    print(vRight_out.output['f_vRight'])
    vRight_out.output['f_vRight']


    f_s4.view()
    f_vRight.view()
    f_vRight.view(sim=vRight_out)
    f_vLeft.view(sim=vLeft_out)


    plt.show()
    plt.pause(0.0001)


    plt.figure()

    #laser2D = objectHandle(clientID, 'LaserScanner_2D')


    # Initializing
    groundTruthX = []
    groundTruthY = []
    odometryX = []
    odometryY = []
    '''
    cloudPointXGT = []
    cloudPointYGT = []
    cloudPointXodo= []
    cloudPointYodo = []
    laserXGT = []
    laserYGT = []
    laserXodo = []
    laserYodo = []
    '''
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

                Vl = R*leftDiff/t
                Vr = R*rightDiff/t

                S = ((Vr+Vl)/2)*t
                angT = ((Vr-Vl)/(L))*t

                iniLeftAngle = angleEncLeft
                iniRightAngle = angleEncRight

                #gyro = vrep.simxGetFloatSignal(clientID, 'gyroZ', vrep.simx_opmode_blocking)[1]

                E = np.add(E, [S*cos(E[2]+angT/2),S*sin(E[2]+angT/2),angT])

            groundTruthX.append(pos[0])
            groundTruthY.append(pos[1])
            odometryX.append(E[0])
            odometryY.append(E[1])

            j += 1



            sonarValues = []
            print('saa\n\n\n')
            for i, sH in enumerate(sonarHandle):
                error_pos, pos = vrep.simxGetObjectPosition(clientID, p3dx, -1, vrep.simx_opmode_streaming)
                error_angle, angle = vrep.simxGetObjectOrientation(clientID, p3dx, -1, vrep.simx_opmode_streaming)
                error_angle_s, angle_s = vrep.simxGetObjectOrientation(clientID, sH, p3dx, vrep.simx_opmode_streaming)
                readSonar = vrep.simxReadProximitySensor(clientID, sH, vrep.simx_opmode_streaming)

                if error_pos != 0 and error_angle != 0:
                    print('Error with sonar {}'.format(i+1))
                    sonarValues.append(5)

                else:
                    if readSonar[1]:
                        dist = readSonar[2][2]
                        sonarValues.append(dist)
                    else:
                        sonarValues.append(5)

            print(sonarValues[3])
            #h = input()

            vRight_out.input['f_s4'] = sonarValues[3]*100
            vLeft_out.input['f_s4'] = sonarValues[3]*100
            vRight_out.compute()
            vLeft_out.compute()

            vRight = vRight_out.output['f_vRight']/100
            vLeft =  vLeft_out.output['f_vLeft']/100

            vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, vrep.simx_opmode_streaming)

            '''
                if readSonar[1] == True:
                    cPX, cPY = localToGlobal(pos[0], pos[1], angle[2],
                                             dist * cos(angle_s[1]) + R, dist * sin(angle_s[1]) + R)
                    cloudPointXGT.append(cPX)
                    cloudPointYGT.append(cPY)
                    cPX, cPY = localToGlobal(E[0], E[1], E[2],
                                             dist * cos(angle_s[1]) + R, dist * sin(angle_s[1]) + R)
                    cloudPointXodo.append(cPX)
                    cloudPointYodo.append(cPY)



            laser = vrep.simxUnpackFloats(vrep.simxGetStringSignal(clientID, 'Laser2D', vrep.simx_opmode_streaming)[1])
            error_pos, pos = vrep.simxGetObjectPosition(clientID, p3dx, -1, vrep.simx_opmode_streaming)
            error_angle, angle = vrep.simxGetObjectOrientation(clientID, p3dx, -1, vrep.simx_opmode_streaming)

            i = 0
            while (i < int(np.shape(laser)[0])//3):
                cPX, cPY = localToGlobal(pos[0], pos[1], angle[2], laser[(i*3)+1], laser[(i*3)+2])
                laserXGT.append(cPX)
                laserYGT.append(cPY)
                cPX, cPY = localToGlobal(E[0], E[1], E[2], laser[(i*3)+1], laser[(i*3)+2])
                laserXodo.append(cPX)
                laserYodo.append(cPY)
                i += 1
            '''

    except KeyboardInterrupt:
        # stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)

        plt.figure('Laser 2D - Ground Truth')
        plt.plot(groundTruthX, groundTruthY,'b', label = 'Ground Truth')
        plt.plot(odometryX,odometryY,'r-', label = 'Odometria')
        #plt.plot(laserXGT,laserYGT,  'go', label = 'Mapa')
        plt.legend()
        plt.show()

        plt.figure('Sonar - Ground Truth')
        plt.plot(groundTruthX, groundTruthY,'b', label = 'Ground Truth')
        plt.plot(odometryX,odometryY,'r-', label = 'Odometria')
        #plt.plot(cloudPointXGT, cloudPointYGT,  'go', label = 'Mapa')
        plt.legend()
        plt.show()

        plt.figure('Laser 2D - Odometria')
        plt.plot(groundTruthX, groundTruthY,'b', label = 'Ground Truth')
        plt.plot(odometryX,odometryY,'r-', label = 'Odometria')
        #plt.plot(laserXodo,laserYodo,  'go', label = 'Mapa')
        plt.legend()
        plt.show()

        plt.figure('Sonar - Odometria')
        plt.plot(groundTruthX, groundTruthY,'b', label = 'Ground Truth')
        plt.plot(odometryX,odometryY,'r-', label = 'Odometria')
        #plt.plot(cloudPointXodo, cloudPointYodo,  'go', label = 'Mapa')
        plt.legend()
        plt.show()

        fig = plt.figure()
        plt.axis('off')
        #plt.scatter(laserXodo, laserYodo)

        # plt.show()

        '''    
        fig.savefig('imgOdo.png')

        img = cv2.imread('imgOdo.png')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # #extraindo bordas
        edges = cv2.Canny(gray, 200, 200)
        cv2.imwrite('edgesOdo.png', edges)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # hough
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 5, 1, 1)

        for line in lines:
            coords = line[0]
            x1 = coords[0]
            y1 = coords[1]
            x2 = coords[2]
            y2 = coords[3]

            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        cv2.imwrite('houghlinesOdo.png', img)
        '''

else:
    print ('Failed connecting to remote API server')
print ('Program ended')

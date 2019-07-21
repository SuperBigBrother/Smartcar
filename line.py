# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.

import vrep
import sys
import numpy as np
import time
import math

print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP

if clientID != -1:
    print('Connected to remote API server')

else:
    sys.exit('Failed connecting to remote API server')

# handles
# car 0
errorCode, car_0 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx#0',
                                                          vrep.simx_opmode_blocking)
print("Car 0", car_0, errorCode)
errorCode, left_motor_handle_0 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#0',
                                                          vrep.simx_opmode_blocking)
print("Car 0 leftJoint", left_motor_handle_0, errorCode)
errorCode, right_motor_handle_0 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#0',
                                                           vrep.simx_opmode_blocking)
print("Car 0 rightJoint", right_motor_handle_0, errorCode)

# car 1
errorCode, car_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx#1',
                                                          vrep.simx_opmode_blocking)
print("Car 1", car_1, errorCode)
errorCode, left_motor_handle_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#1',
                                                          vrep.simx_opmode_blocking)
print("Car 1 leftJoint", left_motor_handle_1, errorCode)
errorCode, right_motor_handle_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#1',
                                                           vrep.simx_opmode_blocking)
print("Car 1 rightJoint", right_motor_handle_1, errorCode)

# car 2
errorCode, car_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx#2',
                                                          vrep.simx_opmode_blocking)
print("Car 2", car_2, errorCode)
errorCode, left_motor_handle_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#2',
                                                          vrep.simx_opmode_blocking)
print("Car 1 leftJoint", left_motor_handle_2, errorCode)
errorCode, right_motor_handle_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#2',
                                                           vrep.simx_opmode_blocking)
print("Car 1 rightJoint", right_motor_handle_2, errorCode)

# init sensors
sensors_0_dist = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
sensors_1_dist = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
sensors_2_dist = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
for i in range(16):
    errorCode, sensors_0_dist[i] = vrep.simxGetFloatSignal(clientID, 'Sensor_0_{}'.format(i), vrep.simx_opmode_streaming)

#init gps
errorCode, car0_pos = vrep.simxGetObjectPosition(clientID, car_0, -1, vrep.simx_opmode_streaming)
errorCode, car0_dir = vrep.simxGetObjectOrientation(clientID, car_0, -1, vrep.simx_opmode_streaming)
errorCode, car1_pos = vrep.simxGetObjectPosition(clientID, car_1, -1, vrep.simx_opmode_streaming)
errorCode, car1_dir = vrep.simxGetObjectOrientation(clientID, car_1, -1, vrep.simx_opmode_streaming)
errorCode, car2_pos = vrep.simxGetObjectPosition(clientID, car_2, -1, vrep.simx_opmode_streaming)
errorCode, car2_dir = vrep.simxGetObjectOrientation(clientID, car_2, -1, vrep.simx_opmode_streaming)



# sample codes
v0 = 2
vLeft_0 = v0
vRight_0 = v0
vLeft_1 = v0
vRight_1 = v0
vLeft_2 = v0
vRight_2 = v0
braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



def gotopos():
    vLeft_0 = 1;
    vright_0 = 1;
    if((abs(car0_dir[2] - car1_dir[2]) > 0.001) or (abs(car0_pos[0] - car1_pos[0]) > 0.0001) or (abs(car0_pos[1] - car2_pos[1]) > 0.0001) or (abs(car0_dir[2] - car2_dir[2]) > 0.001)):
        if((abs(car0_dir[2] - car1_dir[2]) > 0.001) or (abs(car0_pos[0] - car1_pos[0]) > 0.0001)):
            vLeft_1 = -0.1*(car0_dir[2] - car1_dir[2]) + 1*(car0_pos[0] - car1_pos[0]+1)
            vright_1 = 0.1*(car0_dir[2] - car1_dir[2]) + 1*(car0_pos[0] - car1_pos[0]+1)
        else:
            vLeft_1 = 0;
            vright_1 = 0;
        if((abs(car0_dir[2] - car2_dir[2]) > 0.001) or (abs(car0_pos[0] - car2_pos[0]) > 0.0001)):
            vLeft_2 = -0.1*(car0_dir[2] - car2_dir[2]) + 1.1*(car0_pos[0] - car2_pos[0]+1)
            vright_2 = 0.1*(car0_dir[2] - car2_dir[2]) + 1.1*(car0_pos[0] - car2_pos[0]+1)
        else:
            vLeft_2 = 0;
            vright_2 = 0;
        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle_0, vLeft_0, vrep.simx_opmode_oneshot) #电机速度
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle_0, vright_0, vrep.simx_opmode_oneshot) #电机速度
        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle_1, vLeft_1, vrep.simx_opmode_oneshot) #电机速度
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle_1, vright_1, vrep.simx_opmode_oneshot) #电机速度
        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle_2, vLeft_2, vrep.simx_opmode_oneshot) #电机速度
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle_2, vright_2, vrep.simx_opmode_oneshot) #电机速度
        return 1
    else:
        return 1

while 1:
    for i in range(16):
        errorCode, sensors_0_dist[i] = vrep.simxGetFloatSignal(clientID, 'Sensor_0_{}'.format(i), vrep.simx_opmode_buffer) #读传感器信息
    # print(sensors_0_dist)
    errorCode, car0_pos = vrep.simxGetObjectPosition(clientID, car_0, -1, vrep.simx_opmode_streaming)
    errorCode, car0_dir = vrep.simxGetObjectOrientation(clientID, car_0, -1, vrep.simx_opmode_streaming)
    errorCode, car1_pos = vrep.simxGetObjectPosition(clientID, car_1, -1, vrep.simx_opmode_streaming)
    errorCode, car1_dir = vrep.simxGetObjectOrientation(clientID, car_1, -1, vrep.simx_opmode_streaming)
    errorCode, car2_pos = vrep.simxGetObjectPosition(clientID, car_2, -1, vrep.simx_opmode_streaming)
    errorCode, car2_dir = vrep.simxGetObjectOrientation(clientID, car_2, -1, vrep.simx_opmode_streaming)

    stage_1 = 1
    while(stage_1):
        errorCode, car0_pos = vrep.simxGetObjectPosition(clientID, car_0, -1, vrep.simx_opmode_streaming)
        errorCode, car0_dir = vrep.simxGetObjectOrientation(clientID, car_0, -1, vrep.simx_opmode_streaming)
        errorCode, car1_pos = vrep.simxGetObjectPosition(clientID, car_1, -1, vrep.simx_opmode_streaming)
        errorCode, car1_dir = vrep.simxGetObjectOrientation(clientID, car_1, -1, vrep.simx_opmode_streaming)
        errorCode, car2_pos = vrep.simxGetObjectPosition(clientID, car_2, -1, vrep.simx_opmode_streaming)
        errorCode, car2_dir = vrep.simxGetObjectOrientation(clientID, car_2, -1, vrep.simx_opmode_streaming)
        stage_1 = gotopos()


        
    # print("Car 0 position:", errorCode, car0_pos)
    print("Car 0 orientation:", errorCode, car0_dir)
    # print("Car 1 position:", errorCode, car1_pos)
    print("Car 1 orientation:", errorCode, car1_dir)
    # print("Car 2 position:", errorCode, car2_pos)
    print("Car 2 orientation:", errorCode, car2_dir)
    

    for i in range(16):
        vLeft_0 = vLeft_0 + braitenbergL[i] * sensors_0_dist[i]
        vRight_0 = vRight_0 + braitenbergR[i] * sensors_0_dist[i]

    #errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle_0, vLeft_0, vrep.simx_opmode_oneshot) #电机速度
    #errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle_0, vRight_0, vrep.simx_opmode_oneshot)
    #time.sleep(0.5)

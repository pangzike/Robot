from controller import Robot
from controller import Motor
from controller import Lidar
from controller import LidarPoint
from controller import GPS
from controller import Keyboard
import ctypes
import math as mt
import numpy as np, argparse
from new_map import map
if __name__ == '__main__':

    # Press the green button in the gutter to run the script.
    timestep = int(1)
    velocity = 10
    speed_forward = [velocity ,velocity ,velocity ,velocity]
    speed_leftCircle = [ velocity, -  velocity, -  velocity ,  velocity]
    speed_rightCircle = [- velocity,   velocity , velocity , - velocity]
    speed_back = [-velocity ,-velocity,-velocity,-velocity]

    speed_backward = [-velocity, -velocity, -velocity, -velocity]
    speed_leftward = [velocity, -velocity, velocity, -velocity]
    speed_rightward = [-velocity, velocity, -velocity, velocity]
    robot = Robot()
    motor = []
    for i in range(4):
        cur_motor:Motor
        cur_motor = robot.getMotor("motor" + str(i + 1))
        cur_motor.setPosition(float('inf'))
        cur_motor.setVelocity(0.0)
        motor.append(cur_motor)


    keyboard = Keyboard()
    keyboard.enable(1)

    lidar:Lidar
    lidar = robot.getLidar('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()
    G_mid:GPS
    G_mid = robot.getGPS('gps_mid')
    G_right:GPS
    G_right = robot.getGPS('gps_right')
    G_left:GPS
    G_left = robot.getGPS('gps_left')
    G_front:GPS
    G_front = robot.getGPS('gps_front')
    G_back:GPS
    G_back = robot.getGPS('gps_back')
    G_back.enable(timestep)
    G_front.enable(timestep)
    G_left.enable(timestep)
    G_right.enable(timestep)
    G_mid.enable(timestep)
    cur_state = {}

    last_position = None

    def get_location():
        cur_state['mid'] = G_mid.getValues()
        cur_state['front'] = G_front.getValues()
        cur_state['back'] = G_back.getValues()
        cur_state['left'] = G_left.getValues()
        cur_state['right'] = G_right.getValues()

    path = []
    target_pointer = 0
    speed = [0, 0, 0, 0]
    speed2 = [0, 0, 0, 0]
    count = 0
    Map = map(5, 5, [0,0])
    get_location()
    while robot.step(timestep) != -1:
        count = count + 1
        last_position = cur_state['mid']
        get_location()
        lidar_information = lidar.getRangeImage()
        print(lidar_information)
        distance_list = [ i - 0.15 if i - 0.15 > 0 else 0 for i in lidar_information]
        front_uv = [cur_state['mid'][0] - cur_state['front'][0], cur_state['mid'][1] - cur_state['front'][1]]
        front_uv = front_uv / np.linalg.norm(front_uv)

        print(last_position, cur_state['mid'])

        if count % 3 == 0:
            Map.explore(lidar_information, cur_state['mid'], front_uv)

        if count % 200 == 0:
            Map.plot()

        keyValue1 = keyboard.getKey()
        keyValue2 = keyboard.getKey()

        if keyValue1 == 87:
            speed[:] = speed_forward[:]

        elif keyValue1 == 83:
            speed[:] = speed_backward[:]

        elif keyValue1 == 65:
            speed[:] = speed_leftward[:]

        elif keyValue1 == 68:
            speed[:] = speed_rightward[:]

        elif keyValue1 == 81:
            speed[:] = speed_leftCircle[:]

        elif keyValue1 == 69:
            speed[:] = speed_rightCircle[:]

        else:
            speed[:] = [0, 0, 0, 0]


        if keyValue2 == 87:
            speed2[:] = speed_forward[:]

        elif keyValue2 == 83:
            speed2[:] = speed_backward[:]

        elif keyValue2 == 65:
            speed2[:] = speed_leftward[:]

        elif keyValue2 == 68:
            speed2[:] = speed_rightward[:]

        else:
            speed2[:] = [0, 0, 0, 0]

        for i in range(4):
            motor[i].setVelocity(speed[i] + speed2[i])

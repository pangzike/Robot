from controller import Robot
from controller import Motor
from controller import Lidar
from controller import LidarPoint
from controller import GPS
import math as mt
from new_map import map
from bi_rrt import RRT
import numpy as np
# Press the green button in the gutter to run the script.
timestep = int(20)
velocity = 20
width = 5
length = 5

slow_speed = -velocity/5
speed_forward = [velocity, velocity, velocity, velocity]
speed_leftCircle = [ velocity, slow_speed, slow_speed,  velocity]
speed_rightCircle = [slow_speed, velocity, velocity, slow_speed]
speed_back = [-velocity, -velocity, -velocity, -velocity]
speed = [0, 0, 0, 0]

robot = Robot()
motor = []
for i in range(4):
    cur_motor:Motor
    cur_motor = robot.getMotor("motor" + str(i + 1))
    cur_motor.setPosition(float('inf'))
    cur_motor.setVelocity(0.0)
    motor.append(cur_motor)

lidar:Lidar
lidar = robot.getLidar('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()
G_mid: GPS
G_mid = robot.getGPS('gps_mid')
G_right: GPS
G_right = robot.getGPS('gps_right')
G_left: GPS
G_left = robot.getGPS('gps_left')
G_front: GPS
G_front = robot.getGPS('gps_front')
G_back: GPS
G_back = robot.getGPS('gps_back')
G_back.enable(timestep)
G_front.enable(timestep)
G_left.enable(timestep)
G_right.enable(timestep)
G_mid.enable(timestep)
cur_state = {}

def get_location():
    cur_state['mid'] = G_mid.getValues()
    cur_state['front'] = G_front.getValues()
    cur_state['back'] = G_back.getValues()
    cur_state['left'] = G_left.getValues()
    cur_state['right'] = G_right.getValues()

def get_distance(x1,x2):
    return mt.sqrt((x1[0]-x2[0])**2 + (x1[1]-x2[1])**2)

count = 0
path = []
target_pointer = 0
Map = None
end = None
while robot.step(timestep) != -1:

    if count == 0 :
        get_location()
        end = [-cur_state['mid'][0], -cur_state['mid'][1]]
        # Map = map(5, 5, end)  # world1
        # Map = map(4.5, 6, end) # world2
        Map = map(10, 10, end)# world3
        rrt = RRT(Map, Map.cord2pixel(cur_state['mid']), Map.cord2pixel(end))
        count = count + 1
        continue

    get_location()
    if get_distance(cur_state['mid'], end) < 1e-1:
        for i in range(4):
            motor[i].setVelocity(0)
        continue

    count = count + 1
    lidar_information = lidar.getRangeImage()

    # print(lidar_information)
    # print(path)
    # distance_list = [i if i > 0 else 0 for i in lidar_information]
    distance_list = [i - 0.3 if i - 0.3 > 0 else 0 for i in lidar_information]
    front_uv = [cur_state['mid'][0] - cur_state['front'][0], cur_state['mid'][1] - cur_state['front'][1]]
    front_uv = front_uv / np.linalg.norm(front_uv)

    if count % 3 == 0 and count > 5:
        rrt.map.explore(lidar_information, cur_state['mid'], front_uv)
        # flag , new_path = rrt.check_car_pos (cur_state['mid'] )
        # if flag:
        #     speed[:] = speed_back[:]
        #     path = new_path
        #     target_pointer = 0

    if count == 50:
        rrt.find_path(10000)
        path = rrt.rel_path()
        target_pointer = 0

    if count % 100 == 0:
        flag , new_path = rrt.adjust_path(cur_state['mid'])
        if flag:
            speed[:] = speed_back[:]
            path = new_path
            target_pointer = 0
            
    

    if count % 10000 == 0:
        rrt.map.plot()

    # if count == 1 or count % 100 == 0:

    # if target_pointer >= len(path) and (count % 100 == 0 or target_pointer >= len(path) and len(path)!= 0) :
    #     cur_t = Map.choose_target(cur_state['mid'])
    #     print(Map.cord2pixel(cur_state['mid']))
    #     print(cur_t)
    #     rrt = RRT(Map, Map.cord2pixel(cur_state['mid']),cur_t)
    #     path = rrt.find_path(50000)
    #     if len(path) == 0:
    #         continue
    #     path = rrt.rel_path(path)
    #     target_pointer = 0

    if len(path) == 0:
        for i in range(4):
            motor[i].setVelocity(0)
        continue

    target = Map.pixel2cord(path[target_pointer])

    if get_distance(cur_state['mid'], target) < 1e-1:
        target = Map.pixel2cord(path[target_pointer])
        target_pointer = target_pointer + 1

    k_cur_d = 1
    k_tar_d = 1
    k_cur = mt.acos((cur_state['front'][0]-cur_state['back'][0])/get_distance(cur_state['front'],cur_state['back']))
    if cur_state['front'][1] - cur_state['back'][1] < 0:
        k_cur_d = -1
    k_tar = mt.acos((target[0] - cur_state['mid'][0])/get_distance(cur_state['mid'],target))
    if target[1] - cur_state['mid'][1] < 0:
        k_tar_d = -1

    if k_cur_d < 0 and k_tar_d < 0:
        k_cur = -k_cur
        k_tar = -k_tar
    elif k_cur_d > 0 and k_tar_d < 0:
        if k_cur + k_tar < mt.pi:
            k_tar = -k_tar
        else:
            k_tar = -k_tar + 2 * mt.pi
    elif k_cur_d < 0 and k_tar_d > 0:
        if k_cur + k_tar < mt.pi:
            k_cur = -k_cur
        else:
            k_cur = -k_cur + 2 * mt.pi
# 0.1
    if abs(k_cur - k_tar) < 0.1:
        speed[:] = speed_forward[:]
# 0.3
    elif abs(k_cur - k_tar) > 0.3:
        if k_cur > k_tar:
            speed =[ i for i in speed_rightCircle]
        else:
            speed =[ i for i in speed_leftCircle]
    else:
        if k_cur > k_tar:
            speed = [ speed_forward[i] * (k_cur - k_tar) / k_cur + speed_rightCircle[i] * k_tar/ k_cur  for i in range(4)]
        else:
            speed = [ speed_forward[i] * (k_cur - k_tar) / k_cur + speed_leftCircle[i] * k_tar/ k_cur for i in range(4)]

    for i in range(4):
        motor[i].setVelocity(speed[i])

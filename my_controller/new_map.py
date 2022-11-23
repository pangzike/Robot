import numpy as np
from math import *
from PIL import Image
class map():
    def __init__(self, col, row, target):
        self.col = int(col * 60)
        self.row = int(row * 60)
        self.map = np.zeros((self.row, self.col), dtype=int)
        self.target = self.cord2pixel(target)

    def cord2pixel(self, cord):
        r = int(self.row / 2 - cord[0] * 60)
        c = int(self.col / 2 - cord[1] * 60)
        return [r, c]

    def pixel2cord(self, ij): # ij 为 2元组
        x = (self.row / 2 - ij[0]) / 60
        y = (self.col / 2 - ij[1]) / 60
        return [x, y]



    def explore(self, distancelist, car_position, front_uv): # front_uv 小车正对方向的单位向量, 要是 mid and front / distance 处理膨胀
        landmark = self.cord2pixel(car_position)  # 把车的位置作为标志位

        for i in range(512):
            angle = i * (pi * 2) / 512
            rotate_matrix = np.asarray([[cos(angle), sin(angle)], [-sin(angle), cos(angle)]])
            cur_uv = np.dot(front_uv, rotate_matrix)
            obstacle_point = [landmark[0] + cur_uv[0] * 60 * distancelist[i], landmark[1] + cur_uv[1] * 60 * distancelist[i]]
            if distancelist[i] == 0:
                continue
            if obstacle_point[0] < 0 :
                obstacle_point[0] = 0
            if obstacle_point[1] < 0:
                obstacle_point[1] = 0
            if obstacle_point[0] > self.row - 1:
                obstacle_point[0] = self.row - 1
            if obstacle_point[1] > self.col - 1:
                obstacle_point[1] = self.col - 1
            obstacle_point = [int(obstacle_point[0]), int(obstacle_point[1])]
            steps = max(abs(obstacle_point[0] - landmark[0]), abs(obstacle_point[1] - landmark[1]))

            xs = np.linspace(landmark[0], obstacle_point[0], (steps + 1), dtype=int)
            ys = np.linspace(landmark[1], obstacle_point[1], (steps + 1), dtype=int)
            # self.map[obstacle_point[0]][obstacle_point[1]] = -1
            # for i in range(len(xs) - 5):
            #     if self.map[xs[i]][ys[i]] != -1:
            #         self.map[xs[i]][ys[i]] = 1
            # for i in range(len(xs) - 10):
            #     self.map[xs[i]-1:xs[i] + 1][ys[i]-1:ys[i] + 1] = self.map[xs[i]-1:xs[i] + 1][ys[i]-1:ys[i] + 1] + 1
            #         # self.map[xs[len(xs) - 1 - i]][len(ys) - 1 - i] = -1
            for i in range(5):
                if len(xs) - 1 - i > 5:
                    self.map[xs[-i]][ys[-i]] = self.map[xs[-i]][ys[-i]] + 1
                    if ys[-i] + 1 < self.col -1:
                        self.map[xs[-i]][ys[-i] + 1] = self.map[xs[-i]][ys[-i] + 1] + 1
                    if ys[-i] - 1 >= 0:
                        self.map[xs[-i]][ys[-i] - 1] = self.map[xs[-i]][ys[-i] - 1] + 1
    def is_empty(self, i, j):
        if self.map[i][j] < 11:
            return True
        else:
            return False

    def is_obstacle(self, i , j):
        if self.map[i ][j ] > 11:# 可调
            return True
        else:
            return False

    def plot(self):
        out = []
        for x in range(self.row):
            temp = []
            for y in range(self.col):
                if self.map[x][y] > 11:
                    temp.append(0)
                else:
                    temp.append(255)
                # else:
                #     temp.append(255)
                # temp.append(255 - self.map[x][y])
            out.append(temp)
        out = np.uint8(out)
        img = Image.fromarray(out)
        img.show()

    # def choose_target(self, car_position):
    #     landmark = self.cord2pixel(car_position)
    #     out = [0, 0]
    #     if self.map[self.target[0]][self.target[1]] > 40:
    #         return self.target
    #     else:
    #         for i in range(10, self.row, 1):
    #             for j in range(10, self.col, 1):
    #                 if self.map[i][j] > 40 and self.map[i][j] < 60 :
    #                     if out[0] + out[1] < i + j and landmark[0] < 1.3 * i and landmark[1] < 1.3 * j:
    #                         out[0] = i
    #                         out[1] = j
    #
    #     return out




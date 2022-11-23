import numpy as np
from math import *
# 一个 map 500*500
class pixel():
    def __init__(self):
        self.value = 0

    def have_explored(self):
        if self.value != 0:
            return True
        else:
            return False

    def is_obstacle(self):
        if self.value < -5:
            return True
        else:
            return False

    def is_empty(self):
        if self.value > 5:
            return True
        else:
            return False

    def explore_obstacle(self):
        self.value = self.value - 1

    def explore_empty(self):
        self.value = self.value + 2

class block():
    def __init__(self, row = 10, col = 10): # 需要输入几行几列
        self.row = row
        self.col = col
        self.block = []
        for i in range(col):
            self.block.append([pixel() for j in range(row)])

    def sum_unexplore(self): # block中没有被探索过的点数
        sum = 0
        for i in range(self.col):
            for j in range(self.row):
                if self.block[i][j].have_explored() is False:
                    sum = sum + 1
        return sum

    def have_explore(self): # 这个block有没有被探索到过
        if self.sum_unexplore() != self.row * self.col:
            return True
        else:
            return False

class map():
    def __init__(self, row = 10, col = 10):
        self.row = row
        self.col = col
        self.map = []
        self.view = np.zeros((row*90, col*90), dtype=int)
        for i in range(3):
            self.map.append([block(row*30, col*30) for j in range(3)])

    def sys_view(self):
        for i in range(self.row * 90):
            for j in range(self.col * 90):
                blocki, blockj, pixeli, pixelj = self.tdttd(i, j)
                self.view[i][j] = self.map[blocki][blockj].block[pixeli][pixelj].value

    def cord2pixel(self, cord):
        r = int(self.row * 45 - cord[0] * 90)
        c = int(self.col * 45 - cord[1] * 90)
        return [r, c]

    def tdttd(self , r, c):
        blocki = r / (self.row * 30)
        blockj = c / (self.col * 30)
        pixeli = r % (self.row * 30)
        pixelj = c % (self.col * 30)
        return int(blocki), int(blockj), int(pixeli), int(pixelj)

    def explore(self, distancelist, car_position, front_uv): # front_uv 小车正对方向的单位向量, 要是 mid and front / distance 处理膨胀
        landmark = self.cord2pixel(car_position)  # 把车的位置作为标志位
        for i in range(256):
            angle = i * (pi * 2) / 256
            rotate_matrix = np.asarray([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])
            cur_uv = np.dot(front_uv, rotate_matrix)
            obstacle_point = [landmark[0] + int(cur_uv[0] * 90 * distancelist[i]), int(landmark[1] + cur_uv[1] * 90 * distancelist[i])]
            steps = max(abs(obstacle_point[0] - landmark[0]), abs(obstacle_point[1] - landmark[1]))
            xs = np.linspace(landmark[0], obstacle_point[0], (steps + 1), dtype=int)
            ys = np.linspace(landmark[1], obstacle_point[1], (steps + 1), dtype=int)
            for j in range(len(xs) - 2):
                blocki, blockj, pixeli, pixelj = self.tdttd(xs[j], ys[j])
                self.map[blocki][blockj].block[pixeli][pixelj].value = self.map[blocki][blockj].block[pixeli][pixelj].value + 2
            for j in range(2):
                blocki, blockj, pixeli, pixelj = self.tdttd(xs[len(xs) - 1 - j], ys[len(ys) - 1 - j])
                self.map[blocki][blockj].block[pixeli][pixelj].value = self.map[blocki][blockj].block[pixeli][pixelj].value - 1

        self.sys_view()


if __name__ == '__main__':
    b = block(5, 5)
    print('hh')


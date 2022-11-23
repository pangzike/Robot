
import numpy as np
import math
class Node():
    def __init__(self, cord, parent):
        self.cord = cord
        self.parent = parent
    def __eq__(self, other):
        return self.cord == other.cord
STAT_OBSTACLE = '#'
STAT_NORMAL = '.'

class RRT():
    def __init__(self, map, start, end):
        self.map = map
        self.deltaq = 150
        self.start_tree = []
        self.end_tree = []
        self.start = Node(start,-1)
        self.end = Node(end,-1)
        self.start_tree.append(self.start)
        self.end_tree.append(self.end)
        self.near = self.deltaq + 10
        self.route = None

    def check_path(self, xy1, xy2):
        steps = max(abs(xy1[0] - xy2[0]), abs(xy1[1] - xy2[1]))
        xs = np.linspace(xy1[0], xy2[0], steps + 1)
        ys = np.linspace(xy1[1], xy2[1], steps + 1)
        for i in range(1, steps):
            if self.map.map[math.ceil(xs[i])][math.ceil(ys[i])] > 11:
                return False
        return True

    def distance(self, xy1, xy2):
        return math.sqrt((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2)

    def nearst_point(self, V, target):
        out = None
        min_distance = 1000
        for i,point in enumerate(V):
            tmp = self.distance(point.cord,target)
            if  tmp < min_distance:
                out = i
                min_distance = tmp
        return out

    def steer(self, q, q_new):
        distance = self.distance(q,q_new)
        if distance > self.deltaq:
            new_cord = (q[0] + int((self.deltaq / distance) * (q_new[0] - q[0])),
                        q[1] + int((self.deltaq / distance) * (q_new[1] - q[1])))
        else:
            new_cord = q_new

        return new_cord

    def near_node(self, V, point, distance):
        out = []
        for i,p in enumerate(V):
            if self.distance(p.cord, point) < distance and self.check_path(p.cord, point):
                out.append(i)
        return out

    def distance_to_start(self, V, index):
        out = 0
        cur_index = index
        while V[cur_index].parent != -1:
            out = out + self.distance(V[cur_index].cord,V[V[cur_index].parent].cord)
            cur_index = V[cur_index].parent
        return out

    def find_parent(self, V, near_point_indexs, point):
        min = 1000
        min_index = 0
        for c in near_point_indexs:
            dis = self.distance(V[c].cord, point) + self.distance_to_start(V, c)
            if dis < min:
                min = dis
                min_index = c
        return min_index

    def rewiring(self, V, near_point_indexs, point_index):
        dis = self.distance_to_start(V, point_index)
        for c in near_point_indexs:
            if dis + self.distance(V[point_index].cord, V[c].cord) < self.distance_to_start(V, c):
                V[c].parent = point_index


    def find_path(self, K):
        V1 = self.start_tree
        V2 = self.end_tree
        for i in range(K):
            q_rand = (np.random.randint(0+10,self.map.row-10),np.random.randint(0+10,self.map.col-10))
            q_nearst_index = self.nearst_point(V1,q_rand)
            new_point_cord = self.steer(V1[q_nearst_index].cord, q_rand)

            # RRT *
            new_point_list = self.near_node(V1, new_point_cord, self.near)
            new_point_parent = self.find_parent(V1, new_point_list, new_point_cord)
            new_point = Node(new_point_cord, new_point_parent)

            # new_point = Node(new_point_cord, q_nearst_index)

            if self.check_path(new_point.cord, V1[new_point.parent].cord):
                V1.append(new_point)
                # RRT *
                self.rewiring(V1, new_point_list, len(V1) - 1)

                q_nearst_index_1= self.nearst_point(V2, new_point.cord)
                new_point_1_cord = self.steer(V2[q_nearst_index_1].cord, new_point.cord)
                # RRT *
                new_point_1_list = self.near_node(V2, new_point_1_cord, self.near)
                new_point_1_parent = self.find_parent(V2, new_point_1_list, new_point_1_cord)
                new_point_1 = Node(new_point_cord, new_point_1_parent)


                # new_point_1 = Node(new_point_1_cord, q_nearst_index_1)
                if self.check_path(new_point_1.cord, V2[new_point_1.parent].cord):
                    V2.append(new_point_1)
                    cur_parent = len(V2) - 1
                    # RRT *
                    self.rewiring(V2, new_point_1_list, cur_parent)

                    while True:
                        new_point_2_cord = self.steer(new_point_1.cord,new_point.cord)
                        new_point_2 = Node(new_point_2_cord, cur_parent)
                        if self.check_path(new_point_2.cord,new_point_1.cord):
                            V2.append(new_point_2)
                            cur_parent = len(V2) - 1
                            new_point_1 = new_point_2
                        else:
                            new_point_2.cord = (-1,-1)
                            break

                        if new_point_1 == new_point:
                            break
                else:
                    new_point_1.cord = (-1,-1)
                if new_point_1 == new_point:
                    out_1 = [new_point.cord]
                    out_2 = []
                    cursor = new_point
                    while True:
                        cursor = V1[cursor.parent]
                        out_1.append(cursor.cord)
                        if cursor.parent == -1:
                            break
                    cursor = new_point_1
                    while True:
                        cursor = V2[cursor.parent]
                        out_2.append(cursor.cord)
                        if cursor.parent == -1:
                            break
                    out_1 = out_1[::-1]
                    out = out_1 + out_2
                    self.route = out
                    return
            if len(V1) > len(V2):
                tmp = V1
                V1 = V2
                V2 = tmp
        return
    def rel_path(self):
        out = []
        for i in range(len(self.route) - 1):
            xy1, xy2 = self.route[i], self.route[i + 1]
            steps = max(abs(xy1[0] - xy2[0]), abs(xy1[1] - xy2[1]))
            xs = np.linspace(xy1[0], xy2[0], steps + 1)
            ys = np.linspace(xy1[1], xy2[1], steps + 1)
            for j in range(0, steps + 1):
                out.append((math.ceil(xs[j]), math.ceil(ys[j])))

        rel_out = [out[i] for i in range(0, len(out), 30)]
        # rel_out = [out[i] for i in range(0, len(out))]

        rel_out.append(out[len(out) - 1])
        rel_out.append(
            (2 * out[len(out) - 1][0] - out[len(out) - 5][0], 2 * out[len(out) - 1][1] - out[len(out) - 5][1]))
        return rel_out

    def adjust_path(self, cur_position):
        landmark = self.map.cord2pixel(cur_position)
        flag = False
        for i in range(len(self.route) - 1):
            xy1, xy2 = self.route[i], self.route[i + 1]
            if self.check_path(xy1, xy2):
                flag = True
                break
        if flag:
            self.start_tree = []
            self.end_tree = []
            self.start = Node(landmark, -1)
            self.start_tree.append(self.start)
            self.end_tree.append(self.end)
            self.find_path(100000)

        return flag, self.rel_path()

    def check_car_pos(self, cur_position):
        landmark = self.map.cord2pixel(cur_position)
        flag = False
        #判断小车是否离障碍过近
        size =8
        for i in range (-size,size):
            for j in range (-size,size):
                if self.map.is_obstacle(landmark[0] + i, landmark[1] +j ):
                    flag = True
                    break
        if flag:
            self.start_tree = []
            self.end_tree = []
            self.start = Node(landmark, -1)
            self.start_tree.append(self.start)
            self.end_tree.append(self.end)
            self.find_path(100000)

        return flag, self.rel_path()

    # def plot_tree(self):


if __name__ == '__main__':
    rrt = RRT('../../maze.png',end=(753, 28), start=(45, 545))
    path = rrt.find_path(100000)
    rel_path = rrt.rel_path(path)
    rrt.plot(rel_path)
    print('hh')
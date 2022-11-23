import numpy as np
from  math import *
from new_map import map

class RRT( ):
    def __init__(self , map , start, end):
        self.map = map
        # Todo
        self.start = start
        self.end = end
        self.height = map.row
        self.width = map.col
        self.distance = 50
        self.sample_number = 1e5
        #初始树 只有起点
        self.node_list = []
        self.node_list.append(self.start)
        #记录父节点
        self.father_node = {}
        self.father_node[ self.start] = self.start 
        
    def calculate_distance(self, node1, node2):
        dis =  ( (node1[0]-node2[0])**2 +  (node1[1]-node2[1])**2  )**0.5
        return dis 
    
    def check_valid(self, node):
        node = self.int_node(node)
        (x,y) = node
        if  x >= self.height: #考虑小车体积的边界
            return False
        if  y >= self.width:
            return False
        
        if self.map.is_obstacle(x ,y):
            return False
        
        return True

    def check_path(self, node1, node2):
        """ 连线是否经过障碍物"""
        x1,y1 = node1
        x2,y2 = node2
        num= max(abs(x1-x2), abs(y1-y2))  
        x = np.linspace(x1,x2,num)
        y = np.linspace(y1,y2, num)
        for xi,yi in zip(x,y):
            if self.map.is_obstacle (   ceil( xi )  ,  ceil(yi) )   :
                return False
        return True
    
    def check_ready(self,node):
        #Node是否在终点的步长限制范围内 并对其二者做碰撞检测
        dis = self.calculate_distance(node, self.end)
        if dis < self.distance and self.check_path (node, self.end  ):
            return True
        else:
            return False
    
    def sample_node(self, p  ):
        if np.random.uniform(0, 1) < p:
            #考虑小车体积
            return (np.random.randint(0, self.height),  np.random.randint(0, self.width))
        else:
            return self.end
        
    #返回在node_list中最近的点
    def find_node_near ( self, node) -> int:
        #计算离树中所有点的距离
        dis_list = [  self.calculate_distance(node,i) for i in self.node_list]
        index  = dis_list.index(min(dis_list))   
        #返回距离最近的结点
        return     self.node_list[index]
    
    def int_node(self,node):
        return ( ceil(node[0]), ceil(node[1]) )
    
    def find_node_new(self, tree_node ,node):
        dx =  node[0] - tree_node[0]
        dy =  node[1] - tree_node[1]
        if dx == 0:
            if dy >= 0:
                node_new =  (tree_node[0], tree_node[1] + self.distance)
            else:
                node_new =  (tree_node[0], tree_node[1] - self.distance)
        else:
            cos_theta = dx/(dx**2+dy**2)**0.5
            sin_theta = dy/(dx**2+dy**2)**0.5
            node_new =   (tree_node[0] + self.distance*cos_theta , tree_node[1] + self.distance * sin_theta)
        return self.int_node(node_new )

    def smooth_path(self, path, dis):
        # 从起点q_init开始，依次寻找能够 无碰撞连接终点q_goal的顶点
        s_path = []
        node_next = path[1]
        s_path.append(path[0])
        preindex = 0
        curindex = 0

        for i in range(1, len(path)):
            node = path[i]
            # 碰撞检测
            if self.check_path(s_path[-1], node) and i - curindex <= dis:
                preindex = i
                node_next = node
            else:
                curindex = preindex
                s_path.append(node_next)
            if s_path[-1] == path[-1]:
                break

        return s_path

    def learn(self ):  #size 考虑车的体积和墙壁
        # 随机采样  sample_number 个
        finish = False
        while  len (self.node_list) < self.sample_number:
            #随机采样
            node_rand =  self.sample_node(0.8)
            node_near = self.find_node_near(node_rand)
            #找到在邻域范围内的点
            node_new = self.find_node_new(node_near,node_rand)
            #碰撞检测 和是否在区域内
            if node_new in self.node_list: #避免环
                continue
            if not  self.check_valid (node_new):
                continue
            if not self.check_path(node_near,node_new): #连线经过障碍
                continue
            #加入随机树
            self.node_list.append(node_new)
            #记录父节点
            self.father_node[node_new] = node_near
            #检测是否到达终点附近
            if self.check_ready(node_new):
                self.node_list.append(self.end)
                self.father_node[self.end] = node_new
                finish = True
                break
        if finish:
            print("find path")
            path =  self.make_path()
            #smooth
            smooth_path = self. smooth_path ( self. inter_smooth ( self. inter_smooth ( path  )  )  ,   3)
            # change 
            path = [  self.map.pixel2cord(i)    for i in  smooth_path    ]
            return  path
        else:
            print ("没有找到")
            
    def make_path(self):
        #从终点往前找父节点
        path = []
        node = self.end
        while  self.father_node[node] != self.start:
            path.append(node)
            # print(node)
            #父节点
            node = self.father_node[node]
        path.append(self.start)
        return path
    
    def connnect_path(self, path):
        res = []
        for i in range(len(path)-1):
            node1,node2=path[i],path[i+1]
            num = max(abs(node1[0]-node2[0]), abs(node1[1]-node2[1]))  
            x = np.linspace(node1[0],node2[0],num)
            y = np.linspace(node1[1],node2[1],num)
            for xi,yi in zip( x,y): 
                res.append(( ceil(xi),  ceil(yi)))
        return res
    
    
    def inter_smooth(self, path):
        def inter(x):
            res = []
            for i in range (len(x)):
                if i % 2 == 1:
                    if i + 1 < len(x):
                        res.append ((x[i-1]+x[i+1])//2 )
                    else:
                        res.append(x[i])
                else :
                    res.append(x[i])
            return res
        
        x = []
        y = []
        for i in path:
            x.append(i[0])
            y.append(i[1])
        x = inter(x)
        y = inter(y)
        return   [i for i in zip(x,y)]
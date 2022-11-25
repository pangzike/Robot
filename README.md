# Robot

## Task

The perception, planning and control algorithms need to be completed to control the car from the starting point to the destination in an unknown environment.

The map is as below:

<img src="/Users/yangke/Documents/GitHub/Robot/image/map.png" alt="map" style="zoom:50%;" />

> Dynamic scene with two openable doors

The starting point is the upper right corner and the ending point is the lower left corner.

## Method

### Car

Modeling a Mecanum wheel trolley which can move the whole car forward, horizontally, diagonally, rotationally and in combination. Because this is a simulation environment, so this can be easily realized by modifying the friction direction.

<img src="/Users/yangke/Documents/GitHub/Robot/image/car.png" alt="car" style="zoom:70%;" />

The car has a lidar to dynamically build maps and 5 gps to get the trolley's position.

In order to complete this task, there are three main parts: **Perception, Planning, Control**

### Perception

I use the lidar to dynamically build maps.

> lidar is a range sensor. The laser probe emits laser to the target and receives the reflected laser. The distance is calculated by time difference.
>

#### Lidar settings:

```bash
--fieldofview 3.14 --near 0.025 --minRange 0.05 --maxRange 5 --type rotating --defaultFrequency 50 --horizontalResolution 512
```

#### Call radar:

```python
Map = map(5, 5) # Set the size of the graph
lidar_information = lidar.getRangeImage()
distance_list = [ i - 0.15 if i - 0.15 > 0 else 0 for i in lidar_information]
# The diagonal length of the trolley is reduced for all points (approximate)
```

#### Build map:

Processing radar information

Input:

- Distancelist is radar information (1-d list 512)
- car_ position
- gps_ mid (2 elements, which are coordinate values)

- front_uv (The angle of the central axis, which should be a vector (mid to front))

<img src="/Users/yangke/Documents/GitHub/Robot/image/gps.png" alt="gps" style="zoom:33%;" />

The radar is 512 lines, the test chart is $10 * 10$, and the ratio of the figure and the accuracy value is 60 times, which means that the half sector of $600 * 600$, 512 lines is about 256, which is equivalent to the accuracy of the point close to half (it can also be changed to 1024 if the calculation ability allows).

For each angle after rotation (the unit vector is still the unit vector after rotation) multiplied by the radar detection distance , that is, the distance between the trolley and the obstacle * scale (60), the corresponding rows and columns of the obstacle on the map can be calculated.

Traversal angle, using the rotation of unit vector matrix $front\_uv$ rotates 512 times counterclockwise to calculate the angles of 512 radar beams.

Using the difference function, the map value of the five pixels closest to the $end+1$, and the points adjacent to the corresponding column $ value+1$, because it is found in subsequent experiments that this effect will be bette.

```python
for i in range(5):
    if len(xs) - 1 - i > 5:
        self.map[xs[-i]][ys[-i]] = self.map[xs[-i]][ys[-i]] + 1
        if ys[-i] + 1 < self.col -1:
            self.map[xs[-i]][ys[-i] + 1] = self.map[xs[-i]][ys[-i] + 1] + 1
        if ys[-i] - 1 >= 0:
            self.map[xs[-i]][ys[-i] - 1] = self.map[xs[-i]][ys[-i] - 1] + 1
```

#### Outlier processing

If the map value is greater than 11, it will be regarded as an obstacle, mainly because there will be errors in radar information. However, the probability of wall reflection must be greater than that of outliers or error points. Therefore, the more times it is detected as an obstacle, the greater the probability it will be regarded as an obstacle.

#### Result

<img src="/Users/yangke/Documents/GitHub/Robot/image/result.png" alt="result" style="zoom:80%;" />

On the left, there is a good example. However, there is also a bad case, which is on the right. This is the map obtained by the trolley rotating at one angle (rotation is easy to cause errors). Therefore, the threshold value of obstacles should not be too large, and the cost of identifying obstacles is too high. Also, it can't be too small. It will regard some mistakes as obstacles

### Planning

The starting point and ending point are known. At first, the map is initialized to be empty or only the local map around the car. Other unknown parts are presumed to be free. When new obstacles are found later, the path will be modified.

#### Bi-rrt

Compared with the original RRT, this algorithm builds a second tree in the target point area for expansion. In each iteration, the starting step is the same as the original RRT algorithm, sampling random points and then expanding. After expanding the new node of the first tree, take this new target point as the expansion direction of the second tree. At the same time, the second tree will expand the first step to get $q_{new}$. If there is no collision, continue to expand the second step in the same direction until the expansion fails or $ q'_{new} = q_{new}$ indicates that it is connected to the first tree, that is, the whole algorithm is over. Of course, the balance of the two trees must be considered in each iteration, that is, the number of nodes of the two trees. The exchange order is to select the "small" tree for expansion.

Pseudocode:

<img src="/Users/yangke/Documents/GitHub/Robot/image/rrt.png" alt="rrt" style="zoom:50%;" />

#### Improve by using RRT*

The main feature of RRT* algorithm is that it can quickly find the initial path, and then with the increase of sampling points, it will continue to optimize until the target point is found or the set maximum number of cycles is reached. RRT* algorithm is progressive optimization, that is, with the increase of iteration times, the path obtained is more and more optimized, and it is never possible to obtain the optimal path in limited time. In other words, it takes a certain amount of operation time to obtain a relatively satisfactory optimization path. Therefore, the convergence time of RRT algorithm is a prominent research problem. However, it is undeniable that the cost of the path calculated by RRT* algorithm is much lower than that of RRT. The difference between the RRT* algorithm and the RRT algorithm mainly lies in two aspects: 1. The recalculation process of $x_{new}$ is as follows: 

1. The process of selecting a parent node for $x_{new}$

   At the newly generated node $x_{new}$, looking for "nearest neighbor" in the defined radius range near new as a replacement for the new parent node $x_{new}$, and then calculate the shortest distance from the root node to $x_{new}$, the penultimate point of the path as $x_{new}$'s parent.

   ```python
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
   ```

2. The Process of Rerouting Random Trees

   After reselecting the parent node for $x_{new}$, in order to further minimize the cost of connecting nodes in the random tree, reroute the random tree.

   ```python
   def rewiring(self, V, near_point_indexs, point_index):
       dis = self.distance_to_start(V, point_index)
       for c in near_point_indexs:
           if dis + self.distance(V[point_index].cord, V[c].cord) < 	self.distance_to_start(V, c):
               V[c].parent = point_index
   ```

   The points calculated with $find\_path$ are stored in $self.route$, but its granularity is very large, which is not suitable for the control of cars, so interpolation is used to reduce the route granularity

   ```python
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
   
       rel_out.append(out[len(out) - 1])
       rel_out.append(
           (2 * out[len(out) - 1][0] - out[len(out) - 5][0], 2 * out[len(out) - 1][1] - out[len(out) - 5][1]))
       return rel_out
   ```

#### Modify path

Judge whether the path is stuck due to newly discovered obstacles/being too close to obstacles.

- Detect whether there are obstacles on the route, because when the route is generated, it must be barrier free, and if there are obstacles, it must be newly explored

- Check whether there are obstacles around the trolley

- If adjustment is needed, recalculation is still used (because if the end tree is retained, the structure of the whole tree will change if some edges become illegal, so recalculation is directly performed)

### Control

Set gps at the red points of the car's front (gps_front), middle (gps_mid), rear (gps_left), left (gps_left), and right (gps_right) to get the car's position.

The pixel coordinates are used in RRT planning, while the coordinates obtained by gps are in the simulation world, so it is necessary to establish a mapping between them, which is defined in the map class.

```python
def cord2pixel(self, cord):
    r = int(self.row / 2 - cord[0] * 60)
    c = int(self.col / 2 - cord[1] * 60)
    return [r, c]
```

Set the trolley wheels to define the speed of each wheel in different running directions.

```python
velocity = 15
speed_forward = [velocity ,velocity ,velocity ,velocity]
speed_leftCircle = [ velocity, -  velocity, -  velocity ,  velocity]
speed_rightCircle = [- velocity,   velocity , velocity , - velocity]
speed_back = [-velocity ,-velocity,-velocity,-velocity]
speed = [0, 0, 0, 0]

motor = []
for i in range(4):
    cur_motor:Motor
    cur_motor = robot.getMotor("motor" + str(i + 1))
    cur_motor.setPosition(float('inf'))
    cur_motor.setVelocity(0.0)
    motor.append(cur_motor)
```

#### Main idea of control

**Path segmentation**: after the car reaches the end target, set the target as the next node in the path (stop at the end point)

After the path is segmented, **we only need to discuss how each path car should reach the destination from the starting point**

Each small section of path is an obstacle free straight line, so the trolley only needs to keep the line between the front and rear of the car (green line) and the line between the middle of the car and the target point (red line) as close as possible (by moving forward and turning left and right at the same time). At this moment, the turning direction is shown by the black line.

<img src="/Users/yangke/Documents/GitHub/Robot/image/rotate_car.png" alt="rotate_car" style="zoom:30%;" />

Now we have solved the problem of how to turn, how to express the inclination of the two straight lines and how much to turn. The simplest and easiest way to think of is to use two-point slopes. However, it should be noted that the slopes are not evenly distributed on the arc, so I came up with the idea of using Arccos function, the function image is as follows:

<img src="/Users/yangke/Documents/GitHub/Robot/image/arccos.png" alt="arccos" style="zoom:60%;" />

However, its value range is only 0 ～ Pi, while a circle has 0 ～ 2Pi, so it is also necessary to mark whether it is the lower half circle or the upper half circle.

```python
k_cur_d = 1
k_tar_d = 1
k_cur = mt.acos((cur_state['front'][0]-cur_state['back'][0])/get_distance(cur_state['front'],cur_state['back']))
if cur_state['front'][1] - cur_state['back'][1] < 0:
  k_cur_d = -1
  k_tar = mt.acos((target[0] - cur_state['mid'][0])/get_distance(cur_state['mid'],target))
  if target[1] - cur_state['mid'][1] < 0:
    k_tar_d = -1
```

Now the whole circle is represented by two variables, but how to map on a circle can make a simple comparison between $k_{cur}$ and $k_{tar}$ when judging left and right turns, which is also more complex. Here, after my calculation (I don't know how to organize it in language), the code is as follows:

```python
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
```

After such mapping, when judging the steering, you can pass the K_ cur - k_ The value of tar determines the steering direction and angle

Specifically, it is divided into three intervals (the interval dividing point is a super parameter and can be adjusted):

In order to enhance the driving smoothness, if the error radian is less than 0.1 (0.03Pi), let it move forward at full speed.

```python
    if abs(k_cur - k_tar) < 0.1:
        speed[:] = speed_forward[:]
```

If the error is too large and the radian is greater than 0.3 (0.1Pi), stop and adjust the direction first.

```python
    elif abs(k_cur - k_tar) > 0.3:
        if k_cur > k_tar:
            speed =[ i for i in speed_rightCircle]
        else:
            speed =[ i for i in speed_leftCircle]
```

If the error is between the two, adjust while driving.
$$
V = \frac{k_{cur} - k_{tar}}{k_{cur}}V_f+\frac{k_{tar}}{k_{cur}}V_{r/l}
$$

```python
   else:
          if k_cur > k_tar:
              speed = [ speed_forward[i] * (k_cur - k_tar) / k_cur + speed_rightCircle[i] * k_tar/ k_cur  for i in range(4)]
          else:
              speed = [ speed_forward[i] * (k_cur - k_tar) / k_cur + speed_leftCircle[i] * k_tar/ k_cur for i in range(4)]
```

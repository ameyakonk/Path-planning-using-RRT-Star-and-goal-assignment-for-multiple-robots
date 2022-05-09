import sys
from xml.dom.pulldom import parseString
import rospy
import numpy as np
import random
import cv2
import math
from scipy.optimize import linear_sum_assignment

from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Node:
    def __init__(self, val_, cost_, parent_):
        self.val = val_
        self.cost = cost_
        self.parent = parent_

class RRT_Star:
    def __init__(self, src, goal, length, clearance, radius, maxIter):
        self.src = src
        self.goal = goal
        self.length = length
        self.clearance = clearance
        self.radius = radius
        self.openList = []
        self.maxIter = maxIter
        self.goalSampleRate = 10
        self.minrand = 0
        self.maxrand = 1000
        self.expandDis = 20

    # def free_space(self, c, r):
    #     space = np.ndarray((1000,1000),dtype=Node)
    
    #     for x in range(1000):
    #         for y in range(1000):
    #             parent = np.array([-1, -1])
    #             space[x, y] = Node(0, 0, parent)
             
    #             if ((x - 200)**2 + (y - 200)**2 < (100+c+r)**2) or ((x - 200)**2 + (y - 800)**2 < (100+c+r)**2) or (((425-c) < y < (575+c)) and ((25-c-r) < x < (175+c+r))) or (((425-c-r) < y < (575+c+r)) and ((375-c-r) < x < (625+c+r))) or (((200-c-r) < y < (400+c+r)) and ((725-c-r) < x < (875+c+r))):

    #                 space[x, y] = Node(-1, 0, parent)
                    
    #     return space

    # def visualize(self, c):

    #     vis = np.zeros((1000, 1000, 3), np.uint8)

    #     for x in range(1000):
    #         for y in range(1000):

    #             if ((x - 200)**2 + (y - 200)**2 < (100+c)**2) or ((x - 200)**2 + (y - 800)**2 < (100+c)**2) or (((425-c) < y < (575+c)) and ((25-c) < x < (175+c))) or (((425-c) < y < (575+c)) and ((375-c) < x < (625+c))) or (((200-c) < y < (400+c)) and ((725-c) < x < (875+c))):
                    
    #                 vis[x][y] = (0, 255, 255)

    #             if ((x - 200)**2 + (y - 200)**2 < 100**2) or ((x - 200)**2 + (y - 800)**2 < 100**2) or ((425 < y < 575) and (25 < x < 175)) or ((425 < y < 575) and (375 < x < 625)) or ((200 < y < 400) and (725 < x < 875)):

    #                 vis[x][y] = (0, 0, 255)

    #     return vis

    def free_space(self, c, r):
        space = np.ndarray((1000,1000),dtype=Node)
    
        for x in range(1000):
            for y in range(1000):
                parent = np.array([-1, -1])
                space[x, y] = Node(0, 0, parent)
             
                if ((x - 200)**2 + (y - 200)**2 < (100+c+r)**2) or ((x - 200)**2 + (y - 800)**2 < (100+c+r)**2) or (((425-c) < y < (575+c)) and ((25-c-r) < x < (175+c+r))) or (((425-c-r) < y < (575+c+r)) and ((375-c-r) < x < (625+c+r))) or (((200-c-r) < y < (400+c+r)) and ((725-c-r) < x < (875+c+r))) or (((725-c-r) < y < (875+c+r)) and ((575-c-r) < x < (825+c+r))):

                    space[x, y] = Node(-1, 0, parent)
                    
        return space

    def visualize(self, c):

        vis = np.zeros((1000, 1000, 3), np.uint8)

        for x in range(1000):
            for y in range(1000):

                if ((x - 200)**2 + (y - 200)**2 < (100+c)**2) or ((x - 200)**2 + (y - 800)**2 < (100+c)**2) or (((425-c) < y < (575+c)) and ((25-c) < x < (175+c))) or (((425-c) < y < (575+c)) and ((375-c) < x < (625+c))) or (((200-c) < y < (400+c)) and ((725-c) < x < (875+c))) or (((725-c) < y < (875+c)) and ((575-c) < x < (825+c))):
                    
                    vis[x][y] = (0, 255, 255)

                if ((x - 200)**2 + (y - 200)**2 < 100**2) or ((x - 200)**2 + (y - 800)**2 < 100**2) or ((425 < y < 575) and (25 < x < 175)) or ((425 < y < 575) and (375 < x < 625)) or ((200 < y < 400) and (725 < x < 875)) or ((725 < y < 875) and (575 < x < 825)):

                    vis[x][y] = (0, 0, 255)

        return vis 

    def findRandomPoint(self):
        if random.randint(0, 1000) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand)]
        else:
            rnd = [self.goal[0], self.goal[1]]

        return rnd

    def getNearestPoint(self, rndpt):
        minpt = []
        mindis = np.inf
        for point in self.openList:
            dis = math.sqrt((point[0] - rndpt[0])**2 + (point[1] - rndpt[1])**2)
            if dis < mindis:
                minpt.clear()
                minpt.append(point)
                mindis = dis
        return minpt[0]

    def getNewPoint(self, nearestpt, rndpt):
        theta = math.atan2(rndpt[1] - nearestpt[1], rndpt[0] - nearestpt[0])
        currentDistance = math.sqrt(
            (rndpt[1] - nearestpt[1]) ** 2 + (rndpt[0] - nearestpt[0]) ** 2)
        # Find a point within expandDis of nind, and closest to rnd

        x = -1
        y = -1
        if currentDistance <= self.expandDis:
            pass
        else:
            x = nearestpt[0] + round(self.expandDis * math.cos(theta))
            y = nearestpt[1] + round(self.expandDis * math.sin(theta))
        
        return [x, y]
    
    def checkCollision(self, pt1, pt2, space):
        
        x_list = np.arange(pt1[0], pt2[0], 1)
        y_list = ((pt2[1] - pt1[1])/(pt2[0] - pt1[0] + 1e-07))*(x_list - pt1[0]) + pt1[1]

        for i in range(len(x_list)):
            if(space[round(x_list[i]), round(y_list[i])].val == -1):
                return True
        return False

    def findNeighbors(self, newNode):
        no_of_nodes = len(self.openList)
        #r = 50*math.sqrt((math.log(no_of_nodes)/no_of_nodes))
        r = 50
        neighbor_list = []
        for data in self.openList:
            dist = (newNode[0] - data[0])**2 + (newNode[1] - data[1])**2
            if dist <= r**2:
                neighbor_list.append(data)
        
        return neighbor_list

    def choose_parent(self, newNode, nearinds, space):
        if not nearinds:
            return newNode

        dlist = []
        for data in nearinds:
            dx = newNode[0] - data[0]
            dy = newNode[1] - data[1]
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if not self.checkCollision(data, newNode, space):
                dlist.append(space[data[0], data[1]].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        space[newNode[0], newNode[1]].cost = mincost
        space[newNode[0], newNode[1]].parent = minind
        return newNode

    def rewire(self, newNode, nearinds, space):
        for data in nearinds:
          
            dx = newNode[0] - data[0]
            dy = newNode[1] - data[1]
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = space[newNode[0], newNode[1]].cost + d

            if space[data[0], data[1]].cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(data, theta, d, space):
                    space[data[0], data[1]].parent = newNode
                    space[data[0], data[1]].cost = scost

    def check_collision_extend(self, nearNode, theta, d, space):

        tmpNode = nearNode.copy()
        for i in range(int(d / self.expandDis)):
            tmpNode[0] += self.expandDis * math.cos(theta)
            tmpNode[1] += self.expandDis * math.sin(theta)
            if not self.checkCollision(tmpNode, nearNode, space):
                return False
        return True

    def findNearestNodetoGoal(self):
        
        minDist = np.inf
        lastNode = []
        allow = 0
        if(len(self.openList) == 0):
            return None

        for data in self.openList:
            dis = math.sqrt((data[1] - self.goal[1]) ** 2 + (data[0] - self.goal[0]) ** 2)
            if(dis < self.expandDis):
                if(dis < minDist):
                    allow = 1
                    lastNode.clear()
                    lastNode.append(data)
                    minDist = dis
        if(allow == 0):
            return None
        return lastNode[0]
    
    def reset(self):
        return RRT_Star(self.src, self.goal, self.length, self.clearance, self.radius, self.maxIter).main()


    def findPath(self, space, lastNode, vis):

        traverseList = []
        traverseList.append((self.goal[0], self.goal[1]))
        node = space[lastNode[0], lastNode[1]].parent
        cost = space[node[0], node[1]].cost
        while((lastNode[0] != self.src[0]) and (lastNode[1] != self.src[1])):
            p_node = space[lastNode[0], lastNode[1]].parent
            if(math.sqrt((lastNode[0] - p_node[0])**2 + (lastNode[1] - p_node[1])**2) > 400):
                print("Reset initiated")
                return self.reset()
          
            traverseList.append(lastNode)
            lastNode = space[lastNode[0], lastNode[1]].parent
            cv2.line(vis, (lastNode[1], lastNode[0]) , (space[lastNode[0], lastNode[1]].parent[1], space[lastNode[0], lastNode[1]].parent[0]), (255, 255, 255), 1)
        
        traverseList.append((self.src[0], self.src[1]))
        traverseList.reverse()
        return traverseList, cost


    def rrt_Star(self):

        vis = self.visualize(self.clearance)
        
        self.openList.append([self.src[0], self.src[1]])
        
        space = self.free_space(self.clearance, self.radius)

        space[self.src[0], self.src[1]].cost = 0
        space[self.src[0], self.src[1]].parent = self.src

        col1 = 199
        col2 = 20
        for i in range(10000):
            rndpt = self.findRandomPoint()
            nearestpt = self.getNearestPoint(rndpt)
            newpt = self.getNewPoint(nearestpt, rndpt)
            if(self.isValid(newpt, space)):
                
                if not self.checkCollision(nearestpt, newpt, space):
                    neighbors = self.findNeighbors(newpt)
                    self.choose_parent(newpt, neighbors, space)
                    cv2.circle(vis, (newpt[1], newpt[0]), 3, (col1, col2, 255), -1)
                    self.openList.append([newpt[0], newpt[1]])
                    self.rewire(newpt, neighbors, space)

                vis = cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow('Informed RRT*', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                        return 0
                vis = cv2.rotate(vis, cv2.ROTATE_90_CLOCKWISE)


        n_node = self.findNearestNodetoGoal()
        if(n_node == None):
            print("No Path Found")
            return
        traverseList, cost = self.findPath(space, n_node, vis)
        vis = cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE)

        cv2.imshow('Informed RRT*', vis)
        if cv2.waitKey(100) & 0xFF == ord('q'):
                return 0
        return traverseList, cost

    def isValid(self, point, space):
        if (not (0 < point[0] < 1000)) or (not (0 < point[1] < 1000)) or (space[point[0], point[1]].val == -1):
            return False
        return True
    
    def main(self):
        return self.rrt_Star()

def nav_main():

    print("############################################# Processing #############################################")
    srcs = [[50, 50], [400, 100], [900, 100]]
    goal = [[900, 900], [500, 950], [900, 650]]
    clearance = 20
    radius = 20
    maxIter = 1000
    length = 10

    allTraverseList = []
    allTraverseCost = []

    print("START 1, GOAL 1: ", srcs[0]," ", goal[0])
    print("START 2, GOAL 2: ", srcs[1]," ", goal[1])
    print("START 3, GOAL 3: ", srcs[2]," ", goal[2])
    
    for i in range(3):
        for j in range(3):
            list_, cost  = RRT_Star(srcs[i], goal[j], length, clearance, radius, maxIter).main()
            allTraverseList.append(list_)
            allTraverseCost.append(cost)

    print("TraverseList: ")
    print(allTraverseList[0])

    return allTraverseList, allTraverseCost


def laser_callback1(msg):
    pass
    # print("scan1", msg.ranges[90])

def laser_callback2(msg):
    pass
    #print("scan2", msg.ranges[90])

def laser_callback3(msg):
    pass
    #print("scan3", msg.ranges[90])

r1_pose = []
r2_pose = []
r3_pose = []

def callback1(msg):
    global r1_pose
    x = float(msg.pose.pose.position.x)
    y = float(msg.pose.pose.position.y)
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    r1_pose = [x, y, yaw]

def callback2(msg):
    global r2_pose
    x = float(msg.pose.pose.position.x)
    y = float(msg.pose.pose.position.y)
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    r2_pose = [x, y, yaw]

def callback3(msg):
    global r3_pose
    x = float(msg.pose.pose.position.x)
    y = float(msg.pose.pose.position.y)
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    r3_pose = [x, y, yaw]

def findDist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) 
    
less_priority = [0, 0, 0]   

def checkCollision():
    global less_priority
    threshold = 0.4
    if findDist(r1_pose, r2_pose) < threshold:
        less_priority[1] = 1
    else : less_priority[1] = 0
    
    if findDist(r2_pose, r3_pose) < threshold:
        less_priority[2] = 1
    else : less_priority[2] = 0

    if findDist(r1_pose, r3_pose) < threshold:        
        less_priority[2] = 1
    else : less_priority[2] = 0

def isReached(point, cur_pose):
    radius = 0.8
    if(point[0]-cur_pose[0])**2 + (point[1]-cur_pose[1])**2 <= radius:
        return True
    return False

i_ = [0, 0, 0]
reached_count = 0
reached = [0, 0, 0]

def Turtlebot_traverse(velocity_publisher, traverse_list, pose, identifier):
    global i_ 
    global reached_count
    kp_ang = 1

    x_pose = pose[0]
    y_pose = pose[1]
    yaw = pose[2]
    
    i = i_[identifier]
    y_diff = (traverse_list[i][1] - y_pose)
    x_diff = (traverse_list[i][0] - x_pose)
    desired_ang = math.atan(y_diff/(x_diff + 1e-07))

    if(y_diff > 0 and x_diff > 0) or (y_diff < 0 and x_diff > 0):
        ang_difference = desired_ang - (yaw) 
    elif(y_diff < 0 and x_diff < 0) or (y_diff > 0 and x_diff < 0):
        ang_difference = desired_ang - (yaw) + 3.14

    ang_correction = ang_difference*kp_ang

    vel_msg = Twist()
    if(less_priority[identifier] == 0):
        vel_msg.linear.x = 0.2
    else:
        vel_msg.linear.x = 0.0

    vel_msg.angular.z = ang_correction
    velocity_publisher.publish(vel_msg)

    if isReached(traverse_list[i], (x_pose, y_pose)):
        if i == len(traverse_list) - 1:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            velocity_publisher.publish(vel_msg)
            reached_count += 1
            reached[identifier] = 1
            if(reached_count == 3):
                exit()
        i_[identifier] += 1

def main(args):

  rospy.init_node('navigator', anonymous=True)
  velocity_publisher1 = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=10)
  velocity_publisher2 = rospy.Publisher('robot2/cmd_vel', Twist, queue_size=10)
  velocity_publisher3 = rospy.Publisher('robot3/cmd_vel', Twist, queue_size=10)
  odom_sub1 = rospy.Subscriber('robot1/odom', Odometry, callback1)
  odom_sub2 = rospy.Subscriber('robot2/odom', Odometry, callback2)
  odom_sub3 = rospy.Subscriber('robot3/odom', Odometry, callback3)
#   sub1 = rospy.Subscriber('/robot1/scan', LaserScan, laser_callback1)
#   sub2 = rospy.Subscriber('/robot2/scan', LaserScan, laser_callback2)
#   sub3 = rospy.Subscriber('/robot3/scan', LaserScan, laser_callback3)

  traverse_list_, costList = nav_main()

  traverse_list = []
  for i in range(9):
    traverse_list.append(np.asarray(traverse_list_[i])/100 - 5)

  costList_np = np.asarray(costList)
  costList_np = np.reshape(costList_np, (3, 3)) 
  
  print("Traverse List")
  print(traverse_list[0])  
  print()
  print("Cost Matrix: ")
  print(costList_np)

  row_ind, col_ind = linear_sum_assignment(costList_np)
  print("Goal index for robot 1: ", col_ind[0])
  print("Goal index for robot 2: ", col_ind[1])
  print("Goal index for robot 3: ", col_ind[2])

  while (not rospy.is_shutdown()):

    try:
        pass
        #checkCollision()
        if reached[0] == 0:
            Turtlebot_traverse(velocity_publisher1, traverse_list[col_ind[0]], r1_pose, 0)
        if reached[1] == 0:
            Turtlebot_traverse(velocity_publisher2, traverse_list[3 + col_ind[1]], r2_pose, 1)
        if reached[2] == 0:
            Turtlebot_traverse(velocity_publisher3, traverse_list[6 + col_ind[2]], r3_pose, 2)

    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)


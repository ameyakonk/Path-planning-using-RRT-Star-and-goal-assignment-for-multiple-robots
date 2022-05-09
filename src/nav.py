import numpy as np
import math
import cv2
import random
import copy

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

    def free_space(self, c, r):
        space = np.ndarray((1000,1000),dtype=Node)
    
        for x in range(1000):
            for y in range(1000):
                parent = np.array([-1, -1])
                space[x, y] = Node(0, 0, parent)
             
                if ((x - 200)**2 + (y - 200)**2 < (100+c+r)**2) or ((x - 200)**2 + (y - 800)**2 < (100+c+r)**2) or (((425-c) < y < (575+c)) and ((25-c-r) < x < (175+c+r))) or (((425-c-r) < y < (575+c+r)) and ((375-c-r) < x < (625+c+r))) or (((200-c-r) < y < (400+c+r)) and ((725-c-r) < x < (875+c+r))):

                    space[x, y] = Node(-1, 0, parent)
                    
        return space

    def visualize(self, c):

        vis = np.zeros((1000, 1000, 3), np.uint8)

        for x in range(1000):
            for y in range(1000):

                if ((x - 200)**2 + (y - 200)**2 < (100+c)**2) or ((x - 200)**2 + (y - 800)**2 < (100+c)**2) or (((425-c) < y < (575+c)) and ((25-c) < x < (175+c))) or (((425-c) < y < (575+c)) and ((375-c) < x < (625+c))) or (((200-c) < y < (400+c)) and ((725-c) < x < (875+c))):
                    
                    vis[x][y] = (0, 255, 255)

                if ((x - 200)**2 + (y - 200)**2 < 100**2) or ((x - 200)**2 + (y - 800)**2 < 100**2) or ((425 < y < 575) and (25 < x < 175)) or ((425 < y < 575) and (375 < x < 625)) or ((200 < y < 400) and (725 < x < 875)):

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
        r = 70
        neighbor_list = []
        for data in self.openList:
            dist = (newNode[0] - data[0])**2 + (newNode[1] - data[1])**2
            if dist <= r**2:
                neighbor_list.append(data)
        
        print("Neighbor List", neighbor_list)
        return neighbor_list

    def choose_parent(self, newNode, nearinds, space):
        if not nearinds:
            print("child", newNode, " parent: ", None)
            return newNode

        dlist = []
        for data in nearinds:
            dx = newNode[0] - data[0]
            dy = newNode[1] - data[1]
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            # if self.check_collision_extend(data, theta, d, space):
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
        print("child", newNode, " parent: ", minind)
        return newNode

    def rewire(self, newNode, nearinds, space):
        for data in nearinds:
          
            dx = newNode[0] - data[0]
            dy = newNode[1] - data[1]
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = space[newNode[0], newNode[1]].cost + d

            if space[newNode[0], newNode[1]].cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(data, theta, d, space):
                    space[newNode[0], newNode[1]].parent = newNode
                    space[newNode[0], newNode[1]].cost = scost

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
        print(lastNode)
        if(allow == 0):
            return None
        return lastNode[0]
    
    def findPath(self, space, lastNode, vis):

        traverseList = []
        traverseList.append((self.goal[0], self.goal[1]))
        
        while((lastNode[0] != self.src[0]) and (lastNode[1] != self.src[1])):
            traverseList.append(lastNode)
            lastNode = space[lastNode[0], lastNode[1]].parent
            print("LastNode: ", lastNode)
            print("LastNode Parent: ", space[lastNode[0], lastNode[1]].parent)
            cv2.line(vis, (lastNode[1], lastNode[0]) , (space[lastNode[0], lastNode[1]].parent[1], space[lastNode[0], lastNode[1]].parent[0]), (255, 255, 255), 1)
        
        traverseList.append((self.src[0], self.src[1]))
        traverseList.reverse()
        return traverseList


    def rrt_Star(self):

        vis = self.visualize(self.clearance)
        
        self.openList.append([self.src[0], self.src[1]])
        
        space = self.free_space(self.clearance, self.radius)

        space[self.src[0], self.src[1]].cost = 0
        space[self.src[0], self.src[1]].parent = self.src

        col1 = 203
        col2 = 192

        for i in range(4000):
            rndpt = self.findRandomPoint()
           # print(rndpt)
            nearestpt = self.getNearestPoint(rndpt)
            # print(nearestpt)
            newpt = self.getNewPoint(nearestpt, rndpt)
            print(newpt)
            if(self.isValid(newpt, space)):
                
                if not self.checkCollision(nearestpt, newpt, space):
                    neighbors = self.findNeighbors(newpt)
                    newNode = self.choose_parent(newpt, neighbors, space)
                    self.openList.append([newpt[0], newpt[1]])
                    self.rewire(newNode, neighbors, space)

               # print("OpenList", self.openList)
                print("In Process")
                vis = cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow('Informed RRT*', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                        return 0
                vis = cv2.rotate(vis, cv2.ROTATE_90_CLOCKWISE)
            print()

        n_node = self.findNearestNodetoGoal()
        if(n_node == None):
            print("No Path Found")
            return
        traverseList = self.findPath(space, n_node, vis)
        vis = cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE)

        cv2.imshow('Informed RRT*', vis)
        if cv2.waitKey(100) & 0xFF == ord('q'):
                return 0
        return traverseList

    def isValid(self, point, space):
        if (not (0 < point[0] < 1000)) or (not (0 < point[1] < 1000)) or (space[point[0], point[1]].val == -1):
            return False
        return True
    
    def main(self):
        self.rrt_Star()


srcs = [[10, 10], [400, 10], [900, 10]]
goal = [[900, 900], [700, 900], [800, 800]]
clearance = 20
radius = 10
maxIter = 1000
length = 10

#RRT_Star(srcs[2], goal[2], length, clearance, radius, maxIter).main()
for i in range(3):
    RRT_Star(srcs[i], goal[i], length, clearance, radius, maxIter).main()
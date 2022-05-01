'''
MIT License
Copyright (c) 2019 Fanjin Zeng
This work is licensed under the terms of the MIT license, see <https://opensource.org/licenses/MIT>.  
'''

import numpy as np
from random import random
#import matplotlib.pyplot as plt
#from matplotlib import collections  as mc
from collections import deque
import time
import rospy
import rospy
#import pygeodesy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from tf2_msgs.msg import TFMessage
import threading
#import pyproj
#from gps_common.msg import GPSFix
#from gps_common.msg import GPSStatus
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import json 
import ros_np_multiarray as rnm
from rospy.numpy_msg import numpy_msg




class Line():
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn


def Intersection(line, center, radius):
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a)
    t2 = (-b - np.sqrt(discriminant)) / (2 * a)

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True



def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))


def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False


def isThruObstacle(line, obstacles, radius):
    for obs in obstacles:
        if Intersection(line, obs, radius):
            return True
    return False


def nearest(G, vex, obstacles, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles, radius):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx


def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
    return newvex


def window(startpos, endpos):
    width = endpos[0] - startpos[0]
    height = endpos[1] - startpos[1]
    winx = startpos[0] - (width / 2.)
    winy = startpos[1] - (height / 2.)
    return winx, winy, width, height


def isInWindow(pos, winx, winy, width, height):
    if winx < pos[0] < winx+width and \
        winy < pos[1] < winy+height:
        return True
    else:
        return False


class Graph:
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))


    def randomPosition(self):
        rx = random()
        ry = random()

        posx = self.startpos[0] - (self.sx / 2.) + rx * self.sx * 2
        posy = self.startpos[1] - (self.sy / 2.) + ry * self.sy * 2
        return posx, posy


def RRT(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.success = True
            #print('success')
            # break
    return G


def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        # update nearby vertices distance (if shorter)
        for vex in G.vertices:
            if vex == newvex:
                continue

            dist = distance(vex, newvex)
            if dist > radius:
                continue

            line = Line(vex, newvex)
            if isThruObstacle(line, obstacles, radius):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+dist)
            except:
                G.distances[endidx] = G.distances[newidx]+dist

            G.success = True
            #print('success')
            # break
    return G



def dijkstra(G):
    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return list(path)



def plot(G, obstacles, radius, path=None):
    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]
    fig, ax = plt.subplots()

    for obs in obstacles:
        circle = plt.Circle(obs, radius, color='red')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.startpos[0], G.startpos[1], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], c='black')

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)
    plt.show()


def pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)
        # plot(G, obstacles, radius, path)
        return path



class RRT_robot():
    def __init__(self):
        rospy.init_node("rrt_node", anonymous=True)

        self.current_local_x = []
        self.current_local_y = []

        self.target_local_x = []
        self.target_local_y = []

        self.rrt_starter_str = []
        self.rrt_starter_flag = True

        self.obstacle_coordinates = []
        self.new_len_of_obstacles = 0
        self.old_len_of_obstacles = 0

        self.waypoints = []

        self.radius = []
        
        self.threshold_targets = []
        self.count_for_threshold = 0
        self.threshold_flag = True
        self.old_threshold_targets = []

        ##############
        ##PUBLISHERS##
        ##############

        self.waypoints_publisher = rospy.Publisher('/waypoints', numpy_msg(Float32MultiArray), queue_size = 10)
        #self.waypoints_publisher = rospy.Publisher('/waypoints', String, queue_size = 10)
        ###############
        ##SUBSCRIBERS##
        ###############

        self.target_xyz_sub = rospy.Subscriber("/target_gps_xyz", String, self.target_xyz_callBack) 
        self.current_xyz_sub = rospy.Subscriber("/current_gps_xyz", String, self.current_xyz_callBack)
        self.obstacles = rospy.Subscriber('/obstacles', numpy_msg(Float32MultiArray), self.obstacle_callBack)
        self.rrt_starter = rospy.Subscriber('/rrt_starter', String, self.rrt_starter_callBack)
        self.radius_of_circles_sub = rospy.Subscriber('/radius_of_circles', String, self.radius_of_circles_callBack)
        self.threshold_targets_sub = rospy.Subscriber('/threshold_targets', numpy_msg(Float32MultiArray), self.threshold_targets_callBack)
        self.nav_status_feedback_sub = rospy.Subscriber('/nav_status/feedback', String, self.nav_status_feedback_callBack)


    def nav_status_feedback_callBack(self, msg):
        data = msg.data

        if data == "True":
            self.threshold_flag = True


    def threshold_targets_callBack(self, msg):
        self.threshold_targets = rnm.to_numpy_f32(msg)
        self.threshold_targets = self.threshold_targets.tolist()
        #rospy.loginfo(self.threshold_targets)
        


    def radius_of_circles_callBack(self, msg):
        self.radius = float(msg.data)

    
    def obstacle_callBack(self, data):
        self.obstacle_coordinates = rnm.to_numpy_f32(data) 
        self.obstacle_coordinates = self.obstacle_coordinates.tolist()
        
    
    def current_xyz_callBack(self, data):
        msg = data.data
        msg = msg.split(",")
        self.current_local_x = float(msg[0])
        self.current_local_y = float(msg[1])

    
    
    def target_xyz_callBack(self, data):
        msg = data.data
        msg = msg.split(",")
        self.target_local_x = float(msg[0])
        self.target_local_y = float(msg[1])

    def rrt_starter_callBack(self, data):
        self.rrt_starter_str = data.data
        


    



rrt_robot = RRT_robot()

if __name__ == '__main__':
    while True:
        if(rrt_robot.current_local_x != [] and rrt_robot.current_local_y != [] and rrt_robot.target_local_x != [] and rrt_robot.target_local_y != []):
            
            obstacles_coordinates = []
            if(rrt_robot.obstacle_coordinates != []):
                for i in range(0, len(rrt_robot.obstacle_coordinates)-1, 2):
                    #print(rrt_robot.obstacle_coordinates[0][0], rrt_robot.obstacle_coordinates[0][1])
                    obstacles_coordinates.append((rrt_robot.obstacle_coordinates[i][0], rrt_robot.obstacle_coordinates[i+1][1]))
                    #print(obstacles_coordinates)
            #rospy.loginfo("OBSTACLE COORDINATES:" + str(obstacles_coordinates))
            rrt_robot.new_len_of_obstacles = len(obstacles_coordinates)/10 #Her bir obstacle ortalama 10 uzunluguna sahip veri gondermektedir.
            startpos = (rrt_robot.current_local_x, rrt_robot.current_local_y)
            if rrt_robot.threshold_targets != []:
                endpos = (rrt_robot.threshold_targets[0], rrt_robot.threshold_targets[1])
                rospy.loginfo("Goal is scaled from threshold!")


                if rrt_robot.old_threshold_targets != []:
                    if rrt_robot.threshold_targets[0] != rrt_robot.old_threshold_targets[0]:
                        rrt_robot.threshold_flag = True
                        rospy.loginfo("Threshold targets changed!")
                    rospy.loginfo("New Threshold:" + str(rrt_robot.threshold_targets) + "\nOld Threshold:" + str(rrt_robot.old_threshold_targets))

                    rospy.loginfo("RRT Threshold Flag:" + str(rrt_robot.threshold_flag))
                    rospy.loginfo("RRT Starter Flag:" + str(rrt_robot.rrt_starter_flag))
                
                rrt_robot.old_threshold_targets = rrt_robot.threshold_targets
                
            else:
                endpos = (rrt_robot.target_local_x, rrt_robot.target_local_y)
            #obstacles = [(2., 3.), (4., 5.), (10, 10)]
            n_iter = 100
            radius = rrt_robot.radius
            stepSize = 0.7
            rospy.loginfo("Endpos:" + str(endpos))
            #rospy.loginfo("New Length of Obstacles: " + str(rrt_robot.new_len_of_obstacles) + " Old Length of Obstacles: " + str(rrt_robot.old_len_of_obstacles))

            if (rrt_robot.new_len_of_obstacles > rrt_robot.old_len_of_obstacles and rrt_robot.rrt_starter_str == "Start") or (rrt_robot.rrt_starter_str == "Start" and rrt_robot.rrt_starter_flag == True) or (rrt_robot.rrt_starter_str == "Start" and rrt_robot.threshold_flag == True):
                rospy.loginfo("If icersinde!")
                while True:
                    seconds = time.time()
                    G = RRT_star(startpos, endpos, obstacles_coordinates, n_iter, radius, stepSize)
                    seconds_2 = time.time()    
                    rospy.loginfo("Spend Time:" + str(seconds_2-seconds))
                    # G = RRT(startpos, endpos, obstacles, n_iter, radius, stepSize)
                    if G.success:
                        path = dijkstra(G)
                        
                        rrt_robot.waypoints_publisher.publish(rnm.to_multiarray_f32(np.array(path, dtype=np.float32)))
                        #plot(G, obstacles_coordinates, radius, path)
                        rospy.loginfo("New obstacle detected!" + " New Length of Obstacles: " + str(rrt_robot.new_len_of_obstacles) + " Old Length of Obstacles: " + str(rrt_robot.old_len_of_obstacles))
                        """
                        count = 0
                        for i in path:
                            if count < len(path)-3:
                                dist_1 = math.sqrt((path[count][0] - path[count+1][0])**2 + (path[count][1] - path[count+1][1])**2) + math.sqrt((path[count+1][0] - path[count+2][0])**2 + (path[count+1][1] - path[count+2][1])**2)
                                dist_2 = math.sqrt((path[count][0] - path[count+2][0])**2 + (path[count][1] - path[count+2][1])**2)
                                if dist_2 < dist_1 and (isInObstacle(((path[count][0] + path[count+2][0])/2 , (path[count][1] + path[count+2][1])/2), obstacles_coordinates, radius))== False:
                                    path.pop(count+1)
                            count = count + 1 
                        plot(G, obstacles_coordinates, radius, path)
                        rospy.sleep(5)
                        """
                        rrt_robot.rrt_starter_flag = False
                        rrt_robot.threshold_flag = False
                        break
                    else:
                        rospy.loginfo("Can not find the rrt path!")
                        path = []
                
                rospy.sleep(1)

            else:
                try:
                    #rrt_robot.waypoints_publisher.publish(rnm.to_multiarray_f32(np.array(path, dtype=np.float32)))
                    rospy.loginfo("Same obstacles! " + " New Length of Obstacles: " + str(rrt_robot.new_len_of_obstacles) + " Old Length of Obstacles: " + str(rrt_robot.old_len_of_obstacles))
                    rospy.sleep(1)
                except:
                    rospy.sleep(1)

            
            rrt_robot.old_len_of_obstacles = rrt_robot.new_len_of_obstacles


"""
            if G.success:
                path = dijkstra(G)
                if rrt_robot.new_len_of_obstacles != rrt_robot.old_len_of_obstacles:
                    rrt_robot.waypoints_publisher.publish(rnm.to_multiarray_f32(np.array(path, dtype=np.float32)))
                    plot(G, obstacles_coordinates, radius, path)
                    rospy.loginfo("New obstacle detected!" + " New Length of Obstacles: " + str(rrt_robot.new_len_of_obstacles) + " Old Length of Obstacles: " + str(rrt_robot.old_len_of_obstacles))
                    rospy.sleep(5)
                
                else:
                    rrt_robot.waypoints_publisher.publish(rnm.to_multiarray_f32(np.array(path, dtype=np.float32)))
                    rospy.loginfo("Same obstacles! " + " New Length of Obstacles: " + str(rrt_robot.new_len_of_obstacles) + " Old Length of Obstacles: " + str(rrt_robot.old_len_of_obstacles))
                    rospy.sleep(1)

            else:
                #plot(G, obstacles_coordinates, radius)
                path = []
                rospy.sleep(1)"""

            
        

    

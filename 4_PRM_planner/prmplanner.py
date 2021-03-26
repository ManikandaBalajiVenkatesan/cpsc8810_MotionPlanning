# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
import numpy as np
from scene import *
import math
from random import randint
import random
import matplotlib.pyplot as plt

disk_robot = True  # (change this to False for the advanced extension)
obstacles = None  # the obstacles
robot_radius = None  # the radius of the robot
robot_width = None  # the width of the OBB robot (advanced extension)
robot_height = None  # the height of the OBB robot (advanced extension)
edge_path = []


# ----------------------------------------
# # modify the code below
# # ----------------------------------------
#
# # Construction phase: Build the oadmap
# # You should incrementally sample configurations according to a strategy and add them to the roadmap,
# # select the neighbors of each sample according to a distance function and strategy, and
# # attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# # See graph.py to get familiar with the RoadMap class

def build_roadmap(q_range, robot_dim, scene_obstacles) :
    global obstacles, robot_width, robot_height, robot_radius, edge_path
    obstacles = scene_obstacles  # setting the global obstacle variable
    x_limit = q_range[0]  # the range of x-positions for the robot
    y_limit = q_range[1]  # the range of y-positions for the robot
    theta_limit = q_range[2]  # the range of orientations for the robot (advanced extension)
    robot_width, robot_height = robot_dim[0], robot_dim[1]  # the dimensions of the robot, represented as an oriented bounding box
    robot_radius = max(robot_width, robot_height) / 2.
    graph = Roadmap()  # the road map
    xx = np.arange(x_limit[0], x_limit[1], 1.5)
    random.shuffle(xx)
    yy = np.arange(y_limit[0], y_limit[1], 1.5)
    random.shuffle(yy)
    x = []
    y = []
    for i in range(len(xx)) :
        for j in range(len(yy)) :
            x.append(xx[i] + randint(0, 1))
            y.append(yy[j] + randint(0, 1))

    print(len(x))

    for i in range(len(x)) :  # looping through each vertex
        x_rand = x[i]
        y_rand = y[i]
        vertex_in_obstacle = False
        for j in obstacles :
            if (j.x_min - robot_radius < x_rand < j.x_max + robot_radius) and (
                    j.y_min - robot_radius < y_rand < j.y_max + robot_radius) :
                vertex_in_obstacle = True
                break
        if vertex_in_obstacle == False :
            graph.addVertex((x_rand, y_rand))
    vertices = graph.getVertices()
    print('Vertices legnth is',len(vertices))

    edge_path.append(vertices[0])  # Appending the start to edge path

    ep = 0
    while ep < len(edge_path):
        parent_vert = edge_path[ep]
        parent_vert.connetedComponentNr = 1
        k_nearest_agents = k_nearest_neighbors(graph, parent_vert, 5)
        collision(graph, obstacles, k_nearest_agents, parent_vert,0)
        ep += 1

    graph.saveRoadmap("prm_roadmap.txt")
    return graph


# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None

def find_path(q_start, q_goal, graph) :
    closed_set = OrderedSet()
    open_set = PriorityQueue(order=min, f=lambda v : v.f)
    vertices = graph.getVertices()
    for i in vertices:
        i.connetedComponentNr = 0
    graph.addVertex((q_start[0], q_start[1]))
    start_vertice = vertices[-1]
    start_vertice.connetedComponentNr = 1
    k_nearest_agents = k_nearest_neighbors(graph, start_vertice, 5)
    for k in k_nearest_agents:
        vert_k = vertices[k[1]]
        graph.addEdge(start_vertice, vert_k, k[0])
    start_edges = start_vertice.getEdges()
    graph.addVertex((q_goal[0], q_goal[1]))
    end_vertice = vertices[-1]
    k_nearest_agents = k_nearest_neighbors(graph, end_vertice, 5)
    for k in k_nearest_agents:
        vert_k = vertices[k[1]]
        graph.addEdge(end_vertice, vert_k, k[0])

    qstart = start_vertice
    qgoal = end_vertice
    parent = ([[' ']] * len(vertices))
    h = heuristic_cost(qstart, qgoal)
    g = 0
    f = g + h
    open_set.put(qstart, Value(f=f, g=g))
    node_cost = ([100000] * len(vertices))
    while open_set :
        current_node, current_value = open_set.pop()
        if current_node == qgoal :
            closed_set.add(current_node)
            path = []
            while current_node != qstart :
                end_node = current_node
                parent_node = parent[end_node.id]
                current_node = parent_node
                path.append([parent_node.q[0], parent_node.q[1]])
            path.reverse()
            return path
        closed_set.add(current_node)
        neighbours = current_node.getEdges()
        for neighbour in neighbours :
            child_id = neighbour.dest_id
            child_vertex = vertices[child_id]
            if child_vertex not in closed_set :
                g = path_cost(child_vertex, current_node) + current_value.g
                h = heuristic_cost(child_vertex, qgoal)
                f = g + h
                if child_vertex not in open_set or closed_set :
                    open_set.put(child_vertex, Value(f=f, g=g))
                elif child_vertex in open_set :
                    f_prev = open_set.get(child_vertex)
                    if f < f_prev.f :
                        open_set.put(child_vertex, Value(f=f, g=g))
                if node_cost[child_vertex.id] <= f:
                    pass
                else :
                    parent[child_vertex.id] = current_node
                    node_cost[child_vertex.id] = f


# ----------------------------------------
# below are some functions that you may want to populate/modify and use above
# # ----------------------------------------
# def dfs():


def heuristic_cost(current_node, goal_node) :
    x1 = current_node.q[0];
    y1 = current_node.q[1]
    x2 = goal_node.q[0];
    y2 = goal_node.q[1]
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist


def path_cost(current_node, parent_node) :
    x1 = current_node.q[0];
    y1 = current_node.q[1]
    x2 = parent_node.q[0];
    y2 = parent_node.q[1]
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist


def nearest_neighbors(graph, q, max_dist=10.0) :
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances
    """

    return None, None


def k_nearest_neighbors(graph, q, K) :
    """
        Returns the K-nearest roadmap vertices for a given configuration q.
        You may also want to return the corresponding distances
    """
    vertices = graph.getVertices()
    k_nearest = []
    for i in vertices :
        if i.connetedComponentNr == 0:
            # gap = math.sqrt((i.q[0] - q.q[0]) ** 2 + (i.q[1] - q.q[1]) ** 2)
            gap = distance(i.q,q.q)
            if gap < 4:
                k_nearest.append([gap, i.id])
    k = sorted(k_nearest)[0:K]
    return k


def distance(q1, q2) :
    """
        Returns the distance between two configurations.
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration
    """
    dist = math.sqrt((q1[0] - q2[0]) ** 2 + (q1[1] - q2[1]) ** 2)
    return dist


def collision(graph, obstacles, k_nearest_points, i,check):
    global edge_path

    for k in k_nearest_points :
        vertices = graph.getVertices()
        if vertices[k[1]].connetedComponentNr == 0 or check == 1:
            x1 = i.q[0];
            y1 = i.q[1]
            vert = [l for l in vertices if l.id == k[1]]
            x2 = vert[0].q[0];
            y2 = vert[0].q[1]
            obstacle_clash = 0
            for j in obstacles :
                x_max = j.x_max
                y_max = j.y_max
                x_min = j.x_min
                y_min = j.y_min
                # check one edge
                tx1 = ((y_max - y_min) * (x1 - x_max)) / (- (x1 - x2) * (y_min - y_max))
                ty1 = ((y1 - y2) * (x1 - x_max) + (x2 - x1) * (y1 - y_max)) / (- (x1 - x2) * (y_min - y_max))
                # check second edge
                tx2 = ((y_min - y_max) * (x1 - x_min)) / (- (x1 - x2) * (y_max - y_min))
                ty2 = ((y1 - y2) * (x1 - x_min) + (x2 - x1) * (y1 - y_min)) / (- (x1 - x2) * (y_max - y_min))
                # check third edge
                tx3 = ((x_min - x_max) * (y1 - y_min)) / ((x_min - x_max) * (y1 - y2))
                ty3 = ((y1 - y2) * (x1 - x_max) + (x2 - x1) * (y1 - y_min)) / ((x_min - x_max) * (y1 - y2))
                # check fourth edge
                tx4 = ((x_min - x_max) * (y1 - y_max)) / ((x_min - x_max) * (y1 - y2))
                ty4 = ((y1 - y2) * (x1 - x_max) + (x2 - x1) * (y1 - y_max)) / ((x_min - x_max) * (y1 - y2))
                if (0 <= tx1 <= 1 and 0 <= ty1 <= 1) or (0 <= tx2 <= 1 and 0 <= ty2 <= 1) or (
                        0 <= tx3 <= 1 and 0 <= ty3 <= 1) or (0 <= tx4 <= 1 and 0 <= ty4 <= 1) :
                    obstacle_clash += 1
                else :
                    pass
            if obstacle_clash == 0 :
                vertices[k[1]].connetedComponentNr = 1
                graph.addEdge(i, vertices[k[1]], k[0])
                edge_path.append(vertices[k[1]])
        else :
            pass


def interpolate(q1, q2, stepsize) :
    """
        Returns an interpolated local path between two given configurations.
        It can be used to determine whether an edge between vertices is collision-free.
    """

    return None


if __name__ == "__main__" :
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
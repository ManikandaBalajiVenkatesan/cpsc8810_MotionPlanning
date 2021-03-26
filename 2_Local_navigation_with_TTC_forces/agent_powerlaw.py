# simulator.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
from math import sqrt, exp
import random

ksi = 0.5
tou_h = 4
epsilon = 0.2
k = 1
m = 1
tou_0 = 3


class Agent(object):

    def __init__(self, csvParameters, ksi=0.5, dhor=10, timehor=5, goalRadiusSq=1, maxF=10):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0])  # the id of the agent
        self.gid = int(csvParameters[1])  # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])])  # the position of the agent
        self.vel = np.zeros(2)  # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])])  # the goal of the agent
        self.prefspeed = float(csvParameters[6])  # the preferred speed of the agent
        self.gvel = self.goal - self.pos  # the goal velocity of the agent
        self.gvel = self.gvel / (sqrt(self.gvel.dot(self.gvel))) * self.prefspeed
        self.maxspeed = float(csvParameters[7])  # the maximum sped of the agent
        self.radius = float(csvParameters[8])  # the radius of the agent
        self.goalRadiusSq = goalRadiusSq  # parameter to determine if agent is close to the goal
        self.atGoal = False  # has the agent reached its goal?
        self.ksi = ksi  # the relaxation time used to compute the goal force
        self.dhor = 10  # the sensing radius
        self.timehor = timehor  # the time horizon for computing avoidance forces
        self.F = np.zeros(2)  # the total force acting on the agent
        self.maxF = maxF  # the maximum force that can be applied to the agent


    def computeForces(self, neighbors=[]):
        """ 
            Your code to compute the forces acting on the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """
        nearest_neighbours = []
        tous = []
        d = []
        f_avoid = np.zeros(2)
        g_f = (self.gvel - self.vel) / ksi
        goal_force = (self.gvel - self.vel) / ksi

        for n in neighbors:
            if n.id == self.id:
                pass
            else:
                distance = self.dist_check(n.pos[0], n.pos[1])  # Distance check for nearest neighbours
                if distance < self.dhor:
                    nearest_neighbours.append(n)

        if not nearest_neighbours:
            self.F = goal_force

        else:
            for i in nearest_neighbours:
                tou, discr = self.ttc(i)
                tous.append(tou)
                d.append(discr)
            for j in tous:
                if j < self.timehor:
                    rel_vel = self.relative_velocity(nearest_neighbours[tous.index(j)])
                    x = self.pos - nearest_neighbours[tous.index(j)].pos
                    f_avoid = (k * exp(-j/tou_0) * (m*tou_0 + j) * (x+rel_vel*j)) / (tou_0 * sqrt(d[tous.index(j)]) * j**(m+1)) #Extra credit question 1 modification
                    goal_force += f_avoid

        if not self.atGoal:
            self.F = goal_force
            if self.F[0] > 10:
                self.F[0] = 10
            if self.F[0] < -10:
                self.F[0] = -10
            if self.F[1] > 10:
                self.F[1] = 10
            if self.F[1] < -10:
                self.F[1] = -10

    def dist_check(self, x_coordinate, y_coordinate):  # Function/method to calculate the distance between 2 agents
        distance = sqrt((x_coordinate - self.pos[0]) ** 2 + (y_coordinate - self.pos[1]) ** 2)
        return distance

    def ttc(self, j):  # Function/method to find time to collision used in cost function
        # Position of nearest agent
        x = np.zeros(2);x[0] = j.pos[0];x[1] = j.pos[1]
        # Radius of nearest agent
        r = j.radius
        # Velocity of nearest agent
        vj = np.zeros(2);vj[0] = j.vel[0];vj[1] = j.vel[1]
        vg = np.zeros(2);vg[0] = j.gvel[0];vg[1] = j.gvel[1]

        rad = self.radius + r
        w = self.pos - x
        c = w.dot(w) - rad * rad
        if c < 0:
            return 0 , 0
        v = self.vel - vj - (epsilon*(w/sqrt(w.dot(w))))  # Extra credits question 3 modification
        a = v.dot(v) - epsilon**2
        b = w.dot(v) - epsilon*rad
        if b > 0:
            return float('inf'),0
        discr = b * b - a * c
        if discr <= 0:
            return float('inf'),0
        tau = c / (-b + sqrt(discr))
        if tau < 0:
            return float('inf')
        return tau,discr


    def relative_velocity(self, neighbour):
        rel_vel = (self.vel - neighbour.vel) + [random.randint(-10, 10) / 100,random.randint(-10, 10) /100] # Extra credits question 2 modification
        return rel_vel

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel += self.F * dt  # update the velocity
            self.pos += self.vel * dt  # update the position

            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # goal has been reached
            else:
                self.gvel = self.gvel / sqrt(distGoalSq) * self.prefspeed

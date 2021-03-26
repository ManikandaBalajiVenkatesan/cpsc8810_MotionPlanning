# agent.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#


import numpy as np
from math import sqrt

class Agent(object):
    instances = []
    def __init__(self, csvParameters, dhor = 10, goalRadiusSq=1):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent

    def computeNewVelocity(self, neighbors=[]):
        """ 
            Your code to compute the new velocity of the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.   
        """

        least_cost = []
        alpha = 9
        beta = 12
        gamma = 15
        nearest_agents = []
        distances = []
        nearest_agent = []
        sqrt_sample_size = 10

        # Data Sampling
        nx, ny = (sqrt_sample_size, sqrt_sample_size)
        x = np.linspace(-self.maxspeed, self.maxspeed, nx)
        y = np.linspace(-self.maxspeed, self.maxspeed, ny)
        xv, yv = np.meshgrid(x, y)
        xv = xv.flatten()
        yv = yv.flatten()
        high_v_index = []
        for i in range(nx * nx) :
            resultant_v = sqrt((xv[i]) ** 2 + (yv[i]) ** 2)
            if resultant_v > self.maxspeed:    # Filtering sampling velocities to lie within max speed limit
                high_v_index.append(i)
        xv = np.delete(xv, high_v_index)  #Array of permissible samples within the max speed limit in x direction
        yv = np.delete(yv, high_v_index)  #Array of permissible samples within the max speed limit in y direction

        for n in neighbors:
            if n.id == self.id:
                pass
            else:
                distance = self.dist_check(n.pos[0], n.pos[1])   #Distance check for nearest neighbours
                if distance < self.dhor:
                    distances.append(distance)
                    nearest_agents.append(n)
        if not nearest_agents:
            pass
        else:
            nearest_agent = nearest_agents[distances.index(min(distances))]  #Filtering the nearest neighbour



        if self.dist_check(self.goal[0], self.goal[1]) > 1:
            if not nearest_agent:
                self.vnew[:] = self.gvel[:]
            else:
                for i in range(xv.shape[0]):
                    xvi = xv[i]
                    yvi = yv[i]
                    v_cand = np.zeros(2);v_cand[0] = xvi;v_cand[1] = yvi
                    v1 = v_cand - self.gvel
                    v1 = v1.dot(v1)
                    v2 = v_cand - self.vel
                    v2 = v2.dot(v2)
                    tc = self.ttc(nearest_agent, v_cand[0], v_cand[1])   # tc calculation with help of self object, nearest neighbour and candidate velocities
                    cost = (v1 * alpha) + (v2 * beta) + (gamma / (tc + 10e-200))
                    least_cost.append(cost)
                index = least_cost.index(min(least_cost)) # Choosing the velocities corresponding to lowest value from the cost function

                # if dist_check(nearest_agent.pos[0],nearest_agent.pos[1]) < 2.5:
                #     self.vnew[0] = 0.8 *self.xv[index];self.vnew[1] = 0.8 *self.yv[index]
                # else:
                self.vnew[0] = xv[index];self.vnew[1] = yv[index]
        else:
            self.vnew[:] = np.zeros(2)

    def dist_check(self,x_coordinate,y_coordinate): #Function/method to calculate the distance between 2 agents
        distance = sqrt((x_coordinate - self.pos[0])**2 + (y_coordinate - self.pos[1])**2)
        return distance


    def ttc(self,j,vx,vy): #Function/method to find time to collision used in cost function
        # Position of nearest agent
        x = np.zeros(2);x[0] = j.pos[0]; x[1] = j.pos[1]
        # Radius of nearest agent
        r = j.radius
        #Vel of agent (actual vel and candidate when using cost function)
        vi = np.zeros(2);vi[0] = vx;vi[1] = vy
        # Velocity of nearest agent
        vj = np.zeros(2);vj[0] = j.vel[0];vj[1] = j.vel[1]
        vg = np.zeros(2);vg[0] = j.gvel[0];vg[1] = j.gvel[1]

        rad = self.radius + r
        w = self.pos - x
        c = w.dot(w) - rad*rad
        if c < 0:
            return 0
        v = vi - vj
        a = v.dot(v)
        b = w.dot(v)
        if b > 0:
            return float('inf')
        discr = b*b - a*c
        if discr <= 0:
            return float('inf')
        tau = c/(-b + sqrt(discr))
        if tau < 0:
            return float('inf')
        return tau


    def update(self, dt):
        """ 
            Code to update the velocity and position of the agent  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  


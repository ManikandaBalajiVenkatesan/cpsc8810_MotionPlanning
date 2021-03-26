cpsc8810_assignment1

The following functions and variables were implemented and used for simulating 3_agent and 8 agent scenarios.

Functions:

1. computeForces
	To determine the distance between an agent and all the other agents to determine the nearest neighbours. Once nearest neighbours are identified their time to collision(tou) is calculated by calling ttc function which will return tou as well discriminant(D) in ttc calculation. The goal force of the agent is computed using goal velocity and current velocity. Avoidance force due to nearest neighbors are calculated and added to the goal force. After all the avoidance force are added , the resultant goal force components are cheked whether they exceed maximum force capable of that agent and if so it is limited to that.

2. dist_check
	To calculate shortest distance between any two agents. Takes self, x and y coordinate(x_corodinate, y_coordinate) of the other agent as input. Returns the distance between two agents.

3. ttc
	To determing time to collision between two agents considering their relative position and velocities. Self, other agent(j) and self's x and y velocities(vx & vy) are given as input. Returns 'inf' if there they would collide or 0 if they have collided or time to collision if they are about to collide.
	Reference source: https://fb7024eb-a-1e6e9713-s-sites.googlegroups.com/a/g.clemson.edu/cpsc-motion/schedule/lec02.pdf?attachauth=ANoY7cqNf3JNqsNz_9DmnxtKNqGgW34dF5TWTmzEysiRpLfZII3dUhEuCpXmADz0g4pN_Kk6dGnFNChjX1klSNnWck-Gli7A8dcGFNFJiHm8oukUaBwp8lWGVA8PPrR0XsV-BHo72bLhumUlY3JQ9Icvno2LMmlwgTlyAxrzj52kmMxo6KO1BgMPs3KBKJBUkPG-ygZn1-vLVEiLlONTdLVjuAl-1xbzwA%3D%3D&attredirects=0	

4. relative_velocity
	To determine the relative velocity between an agent and one of its nearest neighbour agent.

Parameters:

1. nearest_agents - list to store agents that are within sensing radius of an agent
2. tous - list to store all time to collision of nearest neighbour
3. f_avoid - list for avoidance force vector due to nearest neighbour 
4. distances - distance between self and agents in nearest agents list
5. nearest_neighbours - to store agent that is nearest of all the agents within sensing radius
6. rel_vel - relative velocity between agent and nearest neighbour
7. x - difference between position vectors of agent & nearest neighbour
8. n_dir - unit vector of avoidance force direction.
9. discr - discriminant of ttc calculation , used for powerlaw based collision avoidance.
10. neighbors - list of all agents in the scenario
11. x - position vector of nearby agent
12. r - radius of nearby agent
13. vi - velocity vector of self agent
14. vj - velocity vector of nearby agent
15. vg - goal velocity vector of nearby agent
16. rad - sum of self agent and nearby agent
17. ksi - characteristic time set to 0.5 seconds
18. tou_h - time horizon set to 4 seconds
19. epsilon - uncertainity for relative velocity set to 0.2
20. eta - random perturbation for relative velocity



For extra credit question agent_powerlaw.py & simulator_powerlaw.py are created.
Parameters exclusively used for powerlaw model:
k - scaling factor was set to 1
m - exponent of the power law which was set to 1

For extra credit question 1 we implemented Powerlaw model from slide 40 of Lecture 06 file and checked for epsilon 0 and 0.2

For extra credit question 2 we implemented random perturbation of eta
For extra credit question 3 we implemented adversarial uncertainity model along with powerlaw
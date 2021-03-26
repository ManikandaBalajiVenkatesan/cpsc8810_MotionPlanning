# astar.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#
import pprint


# Compute the optimal path from start to goal.
# The car is moving on a 2D grid and
# its orientation can be chosen from four different directions:
forward = [[-1,  0], # 0: go north
           [ 0, -1], # 1: go west
           [ 1,  0], # 2: go south
           [ 0,  1]] # 3: go east

# The car can perform 3 actions: -1: right turn and then move forward, 0: move forward, 1: left turn and then move forward
action = [-1, 0, 1]
action_name = ['R', 'F', 'L']
cost = [1, 1, 10] # corresponding cost values

# action =forward
# action_name = ['U', 'L', 'R', 'D']
# cost = [1, 1, 1, 1] # corresponding cost values

# GRID:
#     0 = navigable space
#     1 = unnavigable space
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = (4, 3, 0) # (grid row, grid col, orientation)

goal = (2, 0, 1) # (grid row, grid col, orientation)



heuristic = [[2, 3, 4, 5, 6, 7], # Manhattan distance
        [1, 2, 3, 4, 5, 6],
        [0, 1, 2, 3, 4, 5],
        [1, 2, 3, 4, 5, 6],
        [2, 3, 4, 5, 6, 7]]

from utils import (Value, OrderedSet, PriorityQueue)

"""
Two data structures are provided for your open and closed lists: 

 1. OrderedSet is an ordered collection of unique elements.
 2. PriorityQueue is a key-value container whose `pop()` method always pops out
    the element whose value has the highest priority.

 Common operations of OrderedSet, and PriorityQueue
   len(s): number of elements in the container s
   x in s: test x for membership in s
   x not in s: text x for non-membership in s
   s.clear(): clear s
   s.remove(x): remove the element x from the set s;
                nothing will be done if x is not in s

 Unique operations of OrderedSet:
   s.add(x): add the element x into the set s
   s.pop(): return and remove the LAST added element in s;

 Example:
   s = Set()
   s.add((0,1,2))    # add a triplet into the set
   s.remove((0,1,2)) # remove the element (0,1,2) from the set
   x = s.pop()

 Unique operations of PriorityQueue:
   PriorityQueue(order="min", f=lambda v: v): build up a priority queue
       using the function f to compute the priority based on the value
       of an element
   s.put(x, v): add the element x with value v into the queue
                update the value of x if x is already in the queue
   s.get(x): get the value of the element x
            raise KeyError if x is not in s
   s.pop(): return and remove the element with highest priority in s;
            raise IndexError if s is empty
            if order is "min", the element with minimum f(v) will be popped;
            if order is "max", the element with maximum f(v) will be popped.
 Example:
   s = PriorityQueue(order="min", f=lambda v: v.f)
   s.put((1,1,1), Value(f=2,g=1))
   s.put((2,2,2), Value(f=5,g=2))
   x, v = s.pop()  # the element with minimum value of v.f will be popped
"""

# ----------------------------------------
# modify the code below
# ----------------------------------------
def neighbour_orientation(parent_node,node):        #to determine the orientation between parent & child node
    pose = [node[0] - parent_node[0],node[1] - parent_node[1]]
    for i in range(len(forward)):
        if pose == forward[i]:
            return i
        else:
            pass

def find_neighbour(grid,parent_node):            #to identify the neighbouring nodes around parent node
    neighbours = [(parent_node[0],parent_node[1]-1,0), (parent_node[0]-1, parent_node[1],0), (parent_node[0], parent_node[1] + 1,0),(parent_node[0]+ 1, parent_node[1],0)]      #list of all possible nodes
    feasible_neighbours = []
    for node in neighbours:                          #to filter out improbable neighbours
        x_coordinate=int(node[0])
        y_coordinate=int(node[1])
        if (0 <= x_coordinate < 5 and 0 <= y_coordinate < 6):   #to check whether neighbour is beyond the grid limit
            if (grid[x_coordinate][y_coordinate] == 0):         #to check whether neighbour is not a wall
                grid_orientation = int(neighbour_orientation(parent_node, node))     #finding orientation between parent & neighbour
                node = [x_coordinate,y_coordinate,grid_orientation]
                if move_action(parent_node,node) != 2 and move_action(parent_node,node) != -2: #removing neighbours that are behind parent
                    feasible_neighbours.append((x_coordinate,y_coordinate,grid_orientation))          #appending feasible childeren after filtering
    return feasible_neighbours

def find_value(action,current_value,location):    #calculation of cost of particular action from parent to neighbour
    h = heuristic[location[0]][location[1]]       #heuristic cost for that neighbouring node
    g = current_value.g + 1                       #path cost for child is path cost for parent + 1
    f = h + g + cost[action+1]                    #total cost for that neighbouring node
    return f, g

def move_action(current_node,node):        #to determine the action that needs to be taken to move from parent to neighbour
    action = node[2] - current_node[2]    #difference between orientation of parent and neighbour
    if action == 3:                             #change in orientation from north to east is 3 which is left turn
        return -1
    elif action == -3:                          #change in orientation from east to north is -3 which is right turn
        return 1
    else:
        return action

def compute_path(grid,start,goal,cost,heuristic):

    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    # Use thePriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)

    # Keep track of the parent of each node. Since the car can take 4 distinct orientations,
    # for each orientation we can store a 2D array indicating the grid cells.
    # E.g. parent[0][2][3] will denote the parent when the car is at (2,3) facing up

    parent = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]

    # creating a node cost list similar in structure to parent list with inital value of 100
    # used to determine whether the parent list needs to be updated for a particular child
    # if the new cost to reach a child is less than the previous calculated cost
    # then parent list and node_cost list will be updated with new found parent node and cost
    node_cost = [[[ 100 for row in range(len(grid[0]))] for col in range(len(grid))],
             [[ 100 for row in range(len(grid[0]))] for col in range(len(grid))],
             [[ 100 for row in range(len(grid[0]))] for col in range(len(grid))],
             [[ 100 for row in range(len(grid[0]))] for col in range(len(grid))]]

    # The path of the car
    path =[['-' for row in range(len(grid[0]))] for col in range(len(grid))]

    x = start[0]
    y = start[1]
    theta = start[2]
    h = heuristic[x][y]
    g = 0
    f = g+h
    open_set.put(start, Value(f=f,g=g))

    # your code: implement A*
    while open_set: #code runs in loop as long as there is a node in open_set
        current_node,current_value = open_set.pop()     #current node is minimum from open_set

        if current_node == goal:                        #to stop the code once goal is reached
            closed_set.add(current_node)                #adding goal to closed_set
            while current_node != start:                #to back track, from end to start, loop runs till we backtrack to start node
                parent_node = parent[current_node[2]][current_node[0]][current_node[1]]     #accessing parent list to find the parent node of current node
                action = move_action(parent_node,current_node)       #to determine what action was required to move from parent node to current node
                path[parent_node[0]][parent_node[1]] = action_name[action+1]    #updating path list with action required to move from parent to child node
                current_node = parent_node              #now parent node becomes current node before path identification code runs again
            pprint.pprint(path)
            return path,closed_set

        closed_set.add(current_node)        #adding current node to closed_set

        neighbours = find_neighbour(grid, current_node)    #to determine the list of feasible children for current node
        for neighbour in neighbours:                          #running through all feasible childs
            actions = move_action(current_node,neighbour)    #determining action required to move from current node to child node
            total_cost_neighbour, step_cost_neighbour = find_value(actions,current_value,neighbour)  #calculating total & path cost for moving to child node
            if neighbour not in open_set or closed_set:     # if child is not present in open or closed set then add to open_set
                open_set.put(neighbour,Value(f=total_cost_neighbour,g=step_cost_neighbour))
            # incase child is present in open set then new & old value are compared and if new value is low then it is updated
            elif neighbour in open_set:
                f_prev = open_set.get(neighbour)    #accessing old value of child from open_set
                f_prev_f = f_prev.f
                if total_cost_neighbour < f_prev_f:          #comparing old and new values
                    open_set.put(neighbour,Value(f=total_cost_neighbour, g=step_cost_neighbour)) #updating open set if new value is less than old value

            #if cost to travel to child is found to be lesser than previously calculated cost for the child then its parent and cost is updated
            if node_cost[neighbour[2]][neighbour[0]][neighbour[1]] > total_cost_neighbour:
                parent[neighbour[2]][neighbour[0]][neighbour[1]] = current_node[0],current_node[1],current_node[2]      #updating parent node
                node_cost[neighbour[2]][neighbour[0]][neighbour[1]] = total_cost_neighbour                                           #updating cost

    # Initially you may want to ignore theta, that is, plan in 2D.
    # To do so, set actions=forward, cost = [1, 1, 1, 1], and action_name = ['U', 'L', 'R', 'D']
    # Similarly, set parent=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    return path, closed_set

if __name__ == "__main__":
    path,closed=compute_path(grid, init, goal, cost, heuristic)
    """
    for i in range(len(path)):
        print(path[i])
    """
    print("\nExpanded Nodes")    
    for node in closed:
        print(node)

"""
To test the correctness of your A* implementation, when using cost = [1, 1, 10] your code should return 

['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-'] 

In this case, the elements in your closed set (i.e. the expanded nodes) are: 
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)      
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

"""
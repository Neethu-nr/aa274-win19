import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy.matlib as mp

check = 0.015
# Represents a motion planning problem to be solved using A*
class AStar(object):

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, plan_resolution=1):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., (-5, -5))
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., (5, 5))
        self.occupancy = occupancy                 # occupancy grid
        self.plan_resolution = plan_resolution               # resolution of the discretization of state space (cell/m)

        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state


        # print "initial",self.x_goal
        # print self.is_free_end(self.x_goal)
        # if not self.is_free_end(self.x_goal):
        #     self.closest_obstacle=[]
        #     #find closest obstacle
        #     self.find_closest_obstacle(x)
        #     self.move_to_closest_free_grid(mp.repmat(self.x_goal,8,1))
        #     print "out", self.out
        #     self.x_goal=tuple(self.out[np.argmin(self.distance(self.x_goal,self.out))]) 
            
        # print "new", self.x_goal


        self.closed_set = []    # the set containing the states that have been visited
        self.open_set = []      # the set containing the states that are condidate for future expension

        self.f_score = {}       # dictionary of the f score (estimated cost from start to goal passing through state)
        self.g_score = {}       # dictionary of the g score (cost-to-go from start to state)
        self.came_from = {}     # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.append(x_init)
        self.g_score[x_init] = 0
        self.f_score[x_init] = self.distance(x_init,x_goal)

        self.path = None        # the final path as a list of states

    def move_to_closest_free_grid(self,x) :
        print self.plan_resolution
        res = self.plan_resolution
        diag = res/np.sqrt(2)
        delta_x = np.array([[res,0],[-res,0],[0,res],[0,-res],[diag,diag],[diag,-diag],[-diag,diag],[-diag,-diag]])
        x =(x+delta_x)
        neighbors = np.zeros((8,2))
        for i in range(8) :
            neighbors[i,:] = self.snap_to_grid(x[i,:])
        for neigh in neighbors :
            if self.is_free_end(neigh) :
                self.out.append(neigh)
                return 
        for neigh in neighbors:
            self.move_to_closest_free_grid(neigh)
        return 

    def find_closest_obstacle(self,x) :
        print self.plan_resolution
        res = self.plan_resolution
        diag = res/np.sqrt(2)
        delta_x = np.array([[res,0],[-res,0],[0,res],[0,-res],[diag,diag],[diag,-diag],[-diag,diag],[-diag,-diag]])
        x =(x+delta_x)
        neighbors = np.zeros((8,2))
        for i in range(8) :
            neighbors[i,:] = self.snap_to_grid(x[i,:])
        for neigh in neighbors :
            if (~self.is_free(neigh)) :
                self.closest_obstacle.append(neigh)
                return 
        for neigh in neighbors:
            self.find_closest_obstacle(neigh)
        return 

    def is_free_end(self, x):
        for dim in range(len(x)):
            if x[dim] < self.statespace_lo[dim]:
                return False
            if x[dim] >= self.statespace_hi[dim]:
                return False
        if not self.occupancy.is_free(x):
                return False
        return True

    def is_free(self, x):
        if  x==self.x_init or x==self.x_goal:
            return True
        for dim in range(len(x)):
            if x[dim] < self.statespace_lo[dim]:
                return False
            if x[dim] >= self.statespace_hi[dim]:
                return False
        diag = check/np.sqrt(2)
        delta_x = [(0,0),(check,0),(-check,0),(0,check),(0,-check),(diag,diag),(diag,-diag),(-diag,diag),(-diag,-diag)]
        for i in range(9):
            new = self.snap_to_grid((x[0]+delta_x[i][0],x[1]+delta_x[i][1]))
            if not self.occupancy.is_free(new):
                return False
        return True

    # computes the euclidean distance between two states
    # INPUT: (x1, x2)
    #          x1 - first state tuple
    #          x2 - second state tuple
    # OUTPUT: Float euclidean distance
    def distance(self, x1, x2):
        return np.linalg.norm(np.array(x1)-np.array(x2))

    # returns the closest point on a discrete state grid
    # INPUT: (x)
    #          x - tuple state
    # OUTPUT: A tuple that represents the closest point to x on the discrete state grid
    def snap_to_grid(self, x):
        return (self.plan_resolution*round(x[0]/self.plan_resolution), self.plan_resolution*round(x[1]/self.plan_resolution))

    def get_neighbors(self, x):
        # TODO: fill me in!
        res = self.plan_resolution
        diag = res/np.sqrt(2)
        delta_x = [(res,0),(-res,0),(0,res),(0,-res),(diag,diag),(diag,-diag),(-diag,diag),(-diag,-diag)]
        neighbors = []
        for i in range(8):
            new = self.snap_to_grid((x[0]+delta_x[i][0],x[1]+delta_x[i][1]))
            if self.is_free((new)):
                neighbors.append(new)
        return neighbors

    # Gets the state in open_set that has the lowest f_score
    # INPUT: None
    # OUTPUT: A tuple, the state found in open_set that has the lowest f_score
    def find_best_f_score(self):
        return min(self.open_set, key=lambda x: self.f_score[x])

    # Use the came_from map to reconstruct a path from the initial location
    # to the goal location
    # INPUT: None
    # OUTPUT: A list of tuples, which is a list of the states that go from start to goal
    def reconstruct_path(self):
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    # Plots the path found in self.path and the obstacles
    # INPUT: None
    # OUTPUT: None
    def plot_path(self):
        if not self.path:
            return

        fig = plt.figure()

        self.occupancy.plot(fig.number)

        solution_path = np.array(self.path) * self.plan_resolution
        plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="solution path", zorder=10)
        plt.scatter([self.x_init[0]*self.plan_resolution, self.x_goal[0]*self.plan_resolution], [self.x_init[1]*self.plan_resolution, self.x_goal[1]*self.plan_resolution], color="green", s=30, zorder=10)
        plt.annotate(r"$x_{init}$", np.array(self.x_init)*self.plan_resolution + np.array([.2, 0]), fontsize=16)
        plt.annotate(r"$x_{goal}$", np.array(self.x_goal)*self.plan_resolution + np.array([.2, 0]), fontsize=16)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

        plt.axis('equal')
        plt.show()

    # Solves the planning problem using the A* search algorithm. It places
    # the solution as a list of of tuples (each representing a state) that go
    # from self.x_init to self.x_goal inside the variable self.path
    # INPUT: None
    # OUTPUT: Boolean, True if a solution from x_init to x_goal was found
    def solve(self):
        while len(self.open_set)>0:
            x_current = self.find_best_f_score()
            if x_current == self.x_goal:
                self.path = self.reconstruct_path()
                return True
            self.open_set.remove(x_current)
            self.closed_set.append(x_current)
            neighbors = self.get_neighbors(x_current)
            for x_neigh in neighbors:
                if x_neigh in self.closed_set:
                    continue
                scale =1
                tentative_g_score = self.g_score[x_current] + self.distance(x_current, x_neigh) #+ abs(self.occupancy.find_cost_2D(x_neigh))*scale

                if x_neigh not in self.open_set:
                    self.open_set.append(x_neigh)
                elif tentative_g_score > self.g_score[x_neigh]: 
                    continue
                self.came_from.update({x_neigh : x_current})
                self.g_score.update({x_neigh : tentative_g_score})
                self.f_score.update({x_neigh : tentative_g_score + self.distance(x_neigh,self.x_goal)})
  
        return False


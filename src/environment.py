import os
import simplejson
import numpy as np


class Map:
    def __init__(self, map_json):
        '''load start and goal location as well as a list of obstacles'''
        self.start = None
        self.goal = None
        self.obstacles = []

        #TODO::to be implemented
        pass

    def is_in_collision(self, position):
        '''check whether given position is in collision with obstables'''
        #TODO::to be implemented
        pass

    def is_reachable(self, start_position, end_position):
        '''
        check whether end position can be reached from start position
        in a straight line fashion
        '''
        #TODO::to be implemented
        pass

    def compute_goal_gradient(self, position):
        '''return unit vector from given position to goal position'''
        #TODO::to be implemented
        pass

    def compute_obstacles_gradient(self, position):
        '''
        find all obstacles that has an effective on given position and
        compute the net gradient
        '''
        #TODO::to be implemented
        pass

    def compute_net_gradient(self, position, alpha, beta):
        '''
        combine goal gradient and obstacles' gradient
        net_grad = alpha * goal_grad + beta * obstacles_grad
        '''
        #TODO::to be implemented
        pass
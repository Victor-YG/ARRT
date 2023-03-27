import random
import numpy as np


class Tree_Node:
    def __init__(self, position, parent_node):
        self.position = position
        self.children = set()

        self.parent_node = parent_node
        self.cost = 0

        if parent_node is not None:
            v = parent_node.position - position
            self.cost = parent_node.cost + np.sqrt(np.dot(v, v))


    def add_child_node(self, node):
        self.children.add(node)


class Node_Collection:
    def __init__(self):
        self.node_set = []
        self.node_map = {}


    def size(self):
        return len(self.node_set)


    def choose(self):
        '''randomly select a node'''

        return random.choice(self.node_set)


    def add_node(self, node):
        key_x = int(np.floor(node.position[0]))
        key_y = int(np.floor(node.position[1]))
        key = (key_x, key_y)
        self.node_set.append(node)

        if key not in self.node_map:
            self.node_map[key] = []
        self.node_map[key].append(node)


    def get_search_keys(self, query_key):
        '''get all keys to regions to be searched'''

        key_x = query_key[0]
        key_y = query_key[1]

        keys = [
            (key_x + 0, key_y + 0),
            (key_x - 1, key_y + 0),
            (key_x + 1, key_y + 0),
            (key_x + 0, key_y + 1),
            (key_x + 0, key_y - 1),
            (key_x - 1, key_y + 1),
            (key_x - 1, key_y - 1),
            (key_x + 1, key_y + 1),
            (key_x + 1, key_y - 1)
        ]

        return keys

    def find_closest_in_region(self, env, position, key, min_dist):
        '''find closest reacheable node in a region given its key'''

        closest_node = None
        local_min_dist = min_dist

        if key not in self.node_map:
            return closest_node, local_min_dist

        for node in self.node_map[key]:
            dist = np.sqrt(np.dot(node.position - position, node.position - position))
            if dist < local_min_dist:
                if env.is_reachable(node.position, position):
                    local_min_dist = dist
                    closest_node = node

        return closest_node, local_min_dist


    def find_cloest_node(self, env, position):
        '''find closest reacheable node to given position'''

        key_x = int(np.floor(position[0]))
        key_y = int(np.floor(position[1]))

        keys = self.get_search_keys((key_x, key_y))

        min_dist = 1000
        closest_node = None

        for key in keys:
            tmp_node, tmp_dist = self.find_closest_in_region(env, position, key, min_dist)
            if tmp_dist < min_dist:
                min_dist = tmp_dist
                closest_node = tmp_node

        return closest_node, min_dist


    def find_neighbors_in_region(self, env, position, key, dist_thres):
        '''find all reachable node that is within distance threshold to given position'''
        neighbors = []

        for node in self.node_map[key]:
            dist = np.sqrt(np.dot(node.position - position, node.position - position))
            if dist < dist_thres:
                if env.is_reachable(node.position, position):
                    neighbors.append(node)

        return neighbors


    def find_neigbhors(self, env, position, dist_thres=1.0):
        '''find all reachable node that is within distance threshold to given position'''

        key_x = int(np.floor(position[0]))
        key_y = int(np.floor(position[1]))

        keys = self.get_search_keys((key_x, key_y))
        neigbhors = []

        for key in keys:
            neigbhors.extend(self.find_neighbors_in_region(env, position, key, dist_thres))

        return neigbhors
import random
import numpy as np


GOAL_FORCE_AMPLITUDE = 1.5
INITIAL_SPEED = 0.5


def distance(position_1, position_2):
    return np.sqrt(np.dot(position_1 - position_2, position_1 - position_2))


def cross_product_2d(position_1, position_2):
    return position_1[0] * position_2[1] - position_1[1] * position_2[0]


class Tree_Node:
    def __init__(self, position, parent_node, velocity=None, distance_to_goal=None):
        self.position = position
        self.children = []

        self.parent_node = parent_node
        self.velocity = velocity # [speed, angle, angle_search_range]
        self.distance_to_goal = distance_to_goal

        self.cost = 0
        if parent_node is not None:
            v = parent_node.position - position
            self.cost = parent_node.cost + distance(parent_node.position, position)


    def set_velocity(self, velocity):
        self.velocity = velocity


    def score(self):
        return self.velocity[0] / self.distance_to_goal


    def add_child_node(self, node):
        self.children.append(node)


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

    def get_search_keys_2(self, query_key, dist):
        '''get all keys to regions to be searched'''

        key_x = query_key[0]
        key_y = query_key[1]

        keys = [(key_x + 0, key_y + 0)]

        for i in range(int(np.floor(dist))):
            j = i+1
            keys.append((key_x - j, key_y + 0))
            keys.append((key_x + j, key_y + 0))
            keys.append((key_x + 0, key_y + j))
            keys.append((key_x + 0, key_y - j))
            keys.append((key_x - j, key_y + j))
            keys.append((key_x - j, key_y - j))
            keys.append((key_x + j, key_y + j))
            keys.append((key_x + j, key_y - j))

        return keys

    def find_best_in_region(self, env, position, key, dist_thres, root_node):
        best_node = None
        local_dist = float('inf')
        local_min_cost = float('inf')
        min_dist = float('inf')

        if key not in self.node_map:
            return best_node, local_dist, min_dist

        keys = self.get_search_keys_2(key, dist_thres)

        for k in keys:
            if k not in self.node_map:
                continue
            for node in self.node_map[k]:
                dist = distance(node.position, position)
                cost = node.cost + dist
                if dist < dist_thres:
                    if cost < local_min_cost:
                        if env.is_reachable(node.position, position):
                            local_min_cost = cost
                            best_node = node
                            local_dist = dist
                if dist < min_dist:
                    min_dist = dist

        node = root_node
        dist = distance(node.position, position)
        cost = node.cost + dist
        if dist < dist_thres:
            if cost < local_min_cost:
                if env.is_reachable(node.position, position):
                    local_min_cost = cost
                    best_node = node
                    local_dist = dist
        if dist < min_dist:
            min_dist = dist

        return best_node, local_dist, min_dist

    def find_closest_in_region(self, env, position, key, min_dist):
        '''find closest reacheable node in a region given its key'''

        closest_node = None
        local_min_dist = min_dist

        if key not in self.node_map:
            return closest_node, local_min_dist

        for node in self.node_map[key]:
            dist = distance(node.position, position)
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
            dist = distance(node.position, position)
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


class Node_Heap:
    '''data structure to keep track of a priority queue of nodes'''

    def __init__(self, branch_factor_thres=5):
        self.nodes = []
        self.branch_factor_thres = branch_factor_thres # limit on how many times each node can be expanded


    def add_node(self, node):
        '''add a new node and heapify'''

        self.nodes.append(node)
        i = len(self.nodes) - 1
        p = (i - 1) // 2

        while p >= 0:
            if self.nodes[i].score() <= self.nodes[p].score():
                break

            # swap node
            self.swap(i, p)
            i = p
            p = (i - 1) // 2


    def get_node(self):
        '''get the best node to expand on'''

        if len(self.nodes) == 0:
            print("[INFO]: No node left!")
            return None

        selected_node = self.nodes[0]

        if len(selected_node.children) >= self.branch_factor_thres:
            self.pop()

        return selected_node


    def swap(self, i, j):
        tmp_node = self.nodes[j]
        self.nodes[j] = self.nodes[i]
        self.nodes[i] = tmp_node


    def heapify(self, i):
        if i > len(self.nodes) or len(self.nodes) == 0:
            return

        c1 = i * 2 + 1
        c2 = i * 2 + 2

        v = self.nodes[i].score()
        if c1 < len(self.nodes):
            v1 = self.nodes[c1].score()
        else:
            v1 = 0
        if c2 < len(self.nodes):
            v2 = self.nodes[c2].score()
        else:
            v2 = 0

        if v1 > v2 and v1 > v:
            self.swap(i, c1)
            self.heapify(c1)

        if v2 > v1 and v2 > v:
            self.swap(i, c2)
            self.heapify(c2)


    def pop(self):
        self.nodes[0] = self.nodes[len(self.nodes) - 1]
        self.nodes = self.nodes[:-1]
        self.heapify(0)

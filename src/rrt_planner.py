import os
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt

from common import *
from environment import Map

random.seed(0)
np.random.seed(0)


class RRT_Planner():
    '''Implementation of the RRT algorithm'''

    def __init__(self, map, min_node_distance=0.5):
        self.env = map
        self.min_node_distance = min_node_distance
        self.nodes = Node_Collection()
        self.path  = []
        self.figure, self.ax = self.env.render()


    def path_finding(self, start_position, goal_position, max_iter=1000):
        '''find path from start position to goal position'''

        self.root_node = Tree_Node(start_position, parent_node=None)
        self.goal_node = None

        # add start position
        self.nodes.add_node(self.root_node)

        for iter in range(max_iter):
            # select a node from list of nodes
            selected_node = self.nodes.choose()

            # sample new node around selected node
            s     = np.random.uniform(self.min_node_distance, 2.0 * self.min_node_distance)
            theta = np.random.uniform(-np.pi, np.pi)
            step  = s * np.array([np.cos(theta), np.sin(theta)])
            position = selected_node.position + step

            if self.env.is_in_bound(position) == False:
                continue
            if self.env.is_in_collision(position):
                continue

            # find closest reachable neighbor
            parent, dist = self.nodes.find_cloest_node(self.env, position)

            if parent == None:
                continue
            if dist < self.min_node_distance:
                continue

            # configure parent and child relationship
            sampled_node = Tree_Node(position, parent)
            self.nodes.add_node(sampled_node)
            self.ax.plot([parent.position[0], sampled_node.position[0]], [parent.position[1], sampled_node.position[1]], color="cyan",  linewidth=1)
            self.ax.scatter(sampled_node.position[0], sampled_node.position[1], s=3, color="blue")

            # check termination condition
            if self.env.is_reachable(sampled_node.position, goal_position):
                self.goal_node = Tree_Node(goal_position, sampled_node)
                self.nodes.add_node(self.goal_node)
                print("[INFO]: Found a path after {} iterations with {} nodes.".format(iter, self.nodes.size()))
                return True

        return False


    def get_final_path(self):
        '''construct the path from the search tree'''

        position = self.goal_node.position
        self.path = [position]
        node = self.goal_node.parent_node

        while node is not None:
            self.ax.plot([node.position[0], position[0]], [node.position[1], position[1]], color="red", linewidth=5, alpha=0.4)
            position = node.position
            self.path.append(position)
            node = node.parent_node

        self.path.reverse()
        return self.path, self.goal_node.cost


    def dump_statistics(self, filename):
        '''save number of nodes, path length, cost etc. to file'''
        #TODO::to be implemented
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="Input map file (json).", default=None, required=True)
    args = parser.parse_args()

    if os.path.exists(args.input) is False:
        exit("Input file {} doesn't exist.".format(args.input))

    map = Map(args.input)
    planner = RRT_Planner(map)

    start_time = time.time()
    path_found = planner.path_finding(map.start, map.goal, max_iter=10000)
    total_time = time.time() - start_time

    if path_found:
        path, cost = planner.get_final_path()
        print("[INFO]: Found a path with {} nodes and total cost = {} in {} secs.".format(len(path), cost, total_time))
        # for position in path: print(position)
    else:
        print("[INFO]: Failed to find a path in {} secs.".format(total_time))

    # plot final path
    plt.show()


if __name__ == "__main__":
    main()
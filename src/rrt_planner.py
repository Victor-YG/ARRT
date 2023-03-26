import os
import random
import argparse
import numpy as np
import matplotlib.pyplot as plt

from common import Tree_Node
from environment import Map


class RRT_Planner():
    '''Implementation of the RRT algorithm'''

    def __init__(self, map, sample_range=1.0):
        self.env = map
        self.sample_range = sample_range
        self.nodes = []
        self.path  = []
        self.min_node_distance = 1.0

        self.figure, self.ax = self.env.render()


    def path_finding(self, start_position, goal_position, max_iter=1000):
        '''find path from start position to goal position'''

        path = []
        cost = 0

        self.root_node = Tree_Node(start_position, parent_node=None)
        self.goal_node = None

        # add start position
        self.nodes.append(self.root_node)

        for _ in range(max_iter):
            # select a node from list of nodes
            selected_node = random.choice(self.nodes)

            # sample new node around selected node
            x = np.random.uniform(
                max(                 0, selected_node.position[0] - self.sample_range),
                min(self.env.grid_size, selected_node.position[0] + self.sample_range))
            y = np.random.uniform(
                max(                 0, selected_node.position[1] - self.sample_range),
                min(self.env.grid_size, selected_node.position[1] + self.sample_range))
            position = np.array([x, y])

            if self.env.is_in_collision(position):
                continue

            # find closest reachable neighbor
            parent    = None
            min_dist2 = 1000
            for node in self.nodes:
                dist2 = np.dot(node.position - position, node.position - position)
                if dist2 < min_dist2:
                    if self.env.is_reachable(node.position, position):
                        min_dist2 = dist2
                        parent = node

            if parent == None: continue

            # configure parent and child relationship
            sampled_node = Tree_Node(position, parent)
            parent.add_child_node(sampled_node)
            self.nodes.append(sampled_node)
            self.ax.plot([parent.position[0], sampled_node.position[0]], [parent.position[1], sampled_node.position[1]], color="cyan",  linewidth=1)
            self.ax.scatter(sampled_node.position[0], sampled_node.position[1], s=3, color="blue")

            # check termination condition
            if self.env.is_reachable(sampled_node.position, goal_position):
                self.goal_node = Tree_Node(goal_position, sampled_node)
                sampled_node.add_child_node(self.goal_node)
                self.nodes.append(self.goal_node)
                print("[INFO]: Found a path after creating {} nodes.".format(len(self.nodes)))
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
    path_found = planner.path_finding(map.start, map.goal, max_iter=1000)
    if path_found:
        path, cost = planner.get_final_path()
        print("[INFO]: Found a path with {} nodes and total cost = {}.".format(len(path), cost))
        # for position in path: print(position)
    else:
        print("[INFO]: Failed to find a path!")

    # plot final path
    plt.show()


if __name__ == "__main__":
    main()
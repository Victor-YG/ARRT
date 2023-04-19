import os
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt

from common import *
from environment import Map

random.seed(0)
np.random.seed(0)

def distance(position_1, position_2):
    return np.sqrt(np.dot(position_1 - position_2, position_1 - position_2))

class RRTS_Planner():
    '''Implementation of the RRT* algorithm'''

    def __init__(self, map, min_node_distance=0.5, neighbour_distance=2.0):
        self.env = map
        self.min_node_distance = min_node_distance
        self.neighbour_distance = neighbour_distance
        self.nodes = Node_Collection()
        self.path  = []
        self.figure, self.ax = self.env.render()
        self.final_iteration = None

    def path_finding(self, start_position, goal_position, max_iter=1000):
        '''find path from start position to goal position'''
        path = []
        cost = 0

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

            # find lowest cost reachable neighbor in region
            key_x = int(np.floor(selected_node.position[0]))
            key_y = int(np.floor(selected_node.position[1]))
            key = (key_x, key_y)

            parent, dist, min_dist = self.nodes.find_best_in_region(self.env, position, key, self.neighbour_distance, self.root_node)

            if parent == None:
                continue
            if dist < self.min_node_distance:
                continue
            if min_dist < self.min_node_distance:
                continue

            # configure parent and child relationship
            sampled_node = Tree_Node(position, parent)
            self.nodes.add_node(sampled_node)

            # plot
            self.ax.plot([parent.position[0], sampled_node.position[0]], [parent.position[1], sampled_node.position[1]], color="cyan",  linewidth=1)
            self.ax.scatter(sampled_node.position[0], sampled_node.position[1], s=3, color="blue")

            # check for rewiring of nearest neighbours
            neighbours = self.nodes.find_neighbors_in_region(self.env, sampled_node.position, key, self.neighbour_distance)
            neighbours.append(self.root_node)

            for n_node in neighbours:
                #print(n_node.position)

                # Compare current cost to cost of rewire
                cur_cost = n_node.cost
                new_cost = sampled_node.cost + distance(sampled_node.position, n_node.position)

                if new_cost < cur_cost:
                    # Rewire n_node

                    # Remove it as a child of it's current parent
                    cur_parent = n_node.parent_node
                    i = 0;
                    for child in cur_parent.children:
                        if child == n_node:
                            cur_parent.children.pop(i)
                        i = i + 1

                    # Add it as a child of sampled_node
                    n_node.parent_node = sampled_node
                    n_node.cost = new_cost
                    sampled_node.children.append(n_node)

                    # plot
                    self.ax.plot([n_node.position[0], sampled_node.position[0]], [n_node.position[1], sampled_node.position[1]], color="green",  linewidth=1)


            # check termination condition
            if self.env.is_reachable(sampled_node.position, goal_position):
                self.goal_node = Tree_Node(goal_position, sampled_node)
                self.nodes.add_node(self.goal_node)
                self.final_iteration = iter
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

    def dump_statistics(self):
        '''output iteration count, number of nodes explored'''

        return self.final_iteration, self.nodes.size()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="Input map file (json).", default=None, required=True)
    args = parser.parse_args()

    if os.path.exists(args.input) is False:
        exit("Input file {} doesn't exist.".format(args.input))

    map = Map(args.input)
    planner = RRTS_Planner(map)

    start_time = time.time()
    path_found = planner.path_finding(map.start, map.goal, 10000)
    total_time = time.time() - start_time

    if path_found:
        path, cost = planner.get_final_path()
        print("[INFO]: Found a path with {} nodes and total cost = {} in {} secs.".format(len(path), cost, total_time))
        # for position in path: print(position)
    else:
        print("[INFO]: Failed to find a path in {} secs.".format(total_time))

    # plot final path
    plt.show()

    #TODO::save result


if __name__ == "__main__":
    main()

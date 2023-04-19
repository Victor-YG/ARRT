import os
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt

from common import *
from rrts_planner import *
from environment import Map, Point_Object

random.seed(0)
np.random.seed(0)


class ARRTS_Planner(RRTS_Planner):
    '''Implementation of the ARRT algorithm'''

    def __init__(self, map, min_node_distance=0.5, neighbour_distance=2.0):
        super().__init__(map, min_node_distance)
        self.nodes = Node_Collection()
        self.node_heap = Node_Heap()
        self.initial_speed = INITIAL_SPEED
        self.initial_angle = np.pi / 4
        self.final_iteration = None


    def damp_node_speed(self, node):
        node.velocity[0] *= 0.9
        if node.velocity[0] < 0.01:
            point_obstacle = Point_Object(node.position, False, collision_radius=0.0, field_radius=0.5)
            point_obstacle.render(self.figure, self.ax)
            self.env.obstacles.append(point_obstacle)
            self.node_heap.pop()
        if node.velocity[0] < 0.3:
            self.node_heap.heapify(0)


    def path_finding(self, start_position, goal_position, max_iter=1000):
        '''find path from start position to goal position'''

        distance_to_goal = distance(start_position, goal_position)
        velocity = [self.initial_speed, self.initial_angle, np.pi]
        self.root_node = Tree_Node(start_position, None, velocity, distance_to_goal)
        self.goal_node = None

        # add start position
        self.nodes.add_node(self.root_node)
        self.node_heap.add_node(self.root_node)

        for iter in range(max_iter):
            # select a node from list of nodes
            selected_node = self.node_heap.get_node()
            if selected_node == None:
                return False

            # sample new node around selected node
            # s     = np.random.uniform(self.min_node_distance, 2.0 * self.min_node_distance)
            theta = np.random.uniform(-np.pi, np.pi)
            s     = np.random.uniform(
                max(self.min_node_distance,       selected_node.velocity[0] * 1.0),
                max(self.min_node_distance * 2.0, selected_node.velocity[0] * 2.0))
            # theta = np.random.uniform(
            #     selected_node.velocity[1] - selected_node.velocity[2],
            #     selected_node.velocity[1] + selected_node.velocity[2])
            step  = s * np.array([np.cos(theta), np.sin(theta)])
            position = selected_node.position + step

            if self.env.is_in_bound(position) == False or self.env.is_in_collision(position):
                self.damp_node_speed(selected_node)
                continue

            # find closest reachable neighbor
            #parent, dist = self.nodes.find_cloest_node(self.env, position)

            # find lowest cost reachable neighbor in region
            key_x = int(np.floor(selected_node.position[0]))
            key_y = int(np.floor(selected_node.position[1]))
            key = (key_x, key_y)

            parent, dist, min_dist = self.nodes.find_best_in_region(self.env, position, key, self.neighbour_distance, self.root_node)

            if parent == None:
                # self.damp_node_speed(selected_node)
                continue
            if dist < self.min_node_distance:
                # self.damp_node_speed(selected_node)
                continue
            if min_dist < self.min_node_distance:
                continue

            # compute velocity
            force = self.env.compute_goal_force(position)

            # latest implementation
            scale = np.power(1.1, np.dot(force, step))
            speed = parent.velocity[0] * max(1.0, scale)
            angle_range = min(np.pi, max(np.pi / 4, parent.velocity[2] * self.initial_speed / speed))
            velocity = [speed, theta, angle_range]

            # configure parent and child relationship
            sampled_node = Tree_Node(position, parent, velocity, distance(position, goal_position))
            self.node_heap.add_node(sampled_node)
            self.nodes.add_node(sampled_node)
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
                    i = 0
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
                self.goal_node = Tree_Node(goal_position, sampled_node, 0, 0)
                self.nodes.add_node(self.goal_node)
                self.final_iteration = iter
                print("[INFO]: Found a path after {} iterations with {} nodes.".format(iter, self.nodes.size()))
                return True

        print("[INFO]: Failed to find a path after {} iterations with {} nodes.".format(iter, self.nodes.size()))
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="Input map file (json).", default=None, required=True)
    args = parser.parse_args()

    if os.path.exists(args.input) is False:
        exit("Input file {} doesn't exist.".format(args.input))

    map = Map(args.input)
    planner = ARRTS_Planner(map)

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

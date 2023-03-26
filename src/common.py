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

import os
import argparse
from environment import Map


class RRTS_Planner():
    '''Implementation of the RRT* algorithm'''

    def __init__(self, map):
        #TODO::to be implemented
        pass

    def path_finding(self, start_position, goal_position):
        '''find path from start position to goal position'''
        path = []
        cost = 0

        #TODO::to be implemented

        return path, cost


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="Input map file (json).", default=None, required=True)
    args = parser.parse_args()

    if os.path.exists(args.input) is False:
        exit("Input file {} doesn't exist.".format(args.input))

    map = Map(args.input)
    planner = RRTS_Planner(map)
    path, cost = planner.path_finding(map.start, map.goal)

    #TODO::plot final path

    #TODO::save result


if __name__ == "__main__":
    main()
import os
import argparse
import simplejson


def generate_maze(grid_size):
    '''generate maze usign Kruskal's algorithm'''
    #TODO::to be implemented
    pass


def save_map(folder, maze):
    '''save the maze as map (json) with list of line obstables (walls)'''
    #TODO::to be implemented
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--size", help="Grid Size for maze.", type=int, default=5)
    parser.add_argument("--output", help="Output folder to save map.", default="./")
    args = parser.parse_args()

    maze = generate_maze(args.size)
    save_map(args.output, maze)


if __name__ == "__main__":
    main()
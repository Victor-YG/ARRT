import os
import math
import random
import argparse
import datetime
import simplejson
import matplotlib.pyplot as plt


class Cell:
    def __init__(self, x, y, id):
        self.x = x
        self.y = y
        self.id = id
        self.set_id = id


class Wall:
    def __init__(self, cell_1, cell_2, is_horizontal):
        self.cell_1 = cell_1
        self.cell_2 = cell_2
        self.is_horizontal = is_horizontal
        if is_horizontal:
            self.start = (float(cell_2.x), float(cell_2.y))
            self.end   = (float(cell_2.x + 1), float(cell_2.y))
        else:
            self.start = (float(cell_2.x), float(cell_2.y))
            self.end   = (float(cell_2.x), float(cell_2.y + 1))


def generate_map(grid_size, output, visualize=False):
    '''generate maze using randomized Kruskal's algorithm'''

    assert(grid_size >= 3)

    walls = []
    final_walls = []
    cells = dict()

    # Initialize: create a set containing all walls and make each cell a set
    for u in range(grid_size):
        for v in range(grid_size):
            id = v * grid_size + u
            cell = Cell(u, v, id)
            cells[id] = []
            cells[id].append(cell)
            if u != 0:
                walls.append(Wall(cells[id - 1][0], cells[id][0], False))
            if v != 0:
                walls.append(Wall(cells[id - grid_size][0], cells[id][0], True))

    # Iterate over all walls and connect cells from different set add wall to
    # to final_walls if cells across the wall are from the same set
    while True:
        # select a wall at random
        wall = random.choice(walls)

        # if cell are connected - add the wall to final
        if wall.cell_1.set_id == wall.cell_2.set_id:
            final_walls.append(wall)
        # if cell not connected - merge the set (into set with lowest index)
        else:
            if wall.cell_1.set_id < wall.cell_2.set_id:
                set_id_keep = wall.cell_1.set_id
                set_id_omit = wall.cell_2.set_id
                wall.cell_2.set_id = set_id_keep
            else:
                set_id_keep = wall.cell_2.set_id
                set_id_omit = wall.cell_1.set_id
                wall.cell_1.set_id = set_id_keep

            for cell in cells[set_id_omit]:
                cell.set_id = set_id_keep

            cells[set_id_keep].extend(cells[set_id_omit])
            cells[set_id_omit] = None

        # remove wall; terminate if no wall left unprocessed
        walls.remove(wall)
        if len(walls) == 0:
            break

    # visualize the map
    fig, ax = plt.subplots(figsize=(grid_size, grid_size))
    ax.scatter(0.5, 0.5, color="green", marker="o")
    ax.scatter(grid_size - 0.5, grid_size - 0.5, color="red", marker="o")
    for wall in final_walls:
        ax.plot([wall.start[0], wall.end[0]], [wall.start[1], wall.end[1]], color="black", linewidth="2")
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    plt.savefig(output.replace(".json", ".png"))
    if visualize: plt.show()
    plt.cla()

    # save the map
    map_dict = {}
    map_dict["grid_size"] = grid_size
    map_dict["start"] = { "x": 0.5, "y": 0.5 }
    map_dict["goal"]  = { "x": grid_size - 0.5, "y": grid_size - 0.5 }
    map_dict["line_obstacles"] = []
    for wall in final_walls:
        map_dict["line_obstacles"].append(
            {
                "horizontal": wall.is_horizontal,
                "start": {
                    "x": wall.start[0],
                    "y": wall.start[1]
                },
                "end": {
                    "x": wall.end[0],
                    "y": wall.end[1]
                }
            }
        )

    with open(output, "w") as f:
        print(simplejson.dumps(map_dict, ensure_ascii=False, indent=4), file=f)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--size", help="Grid Size for maze.", type=int, default=5)
    parser.add_argument("--output", help="Output folder to save map.", default="")
    parser.add_argument("--count", help="How many map to generate.", type=int, default=1)
    parser.add_argument("-visualize", help="Plot the map while generating.", action="store_true", default=False)
    args = parser.parse_args()

    if os.path.exists(args.output) == False:
        os.mkdir(args.output)
        print("[INFO]: Created output directory {}".format(os.path.abspath(args.output)))

    for i in range(args.count):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        generate_map(args.size, os.path.join(args.output, "{}_{}.json".format(timestamp, i)), args.visualize)


if __name__ == "__main__":
    main()
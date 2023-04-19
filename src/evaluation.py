import os
import time
import argparse
import simplejson
import matplotlib.pyplot as plt

from environment import Map
from rrt_planner import RRT_Planner
from arrt_planner import ARRT_Planner


def find_all_maps(input_folder):
    '''find all maps in input folder'''

    map_json = []
    for f in os.listdir(input_folder):
        if ".json" in f:
            map_json.append(os.path.join(input_folder, f))
    return map_json


def execute_algorithm(map_json, planner_type="rrt", repeat=5, output_folder=None):
    '''execute planner in provided map and collect statistic'''

    if output_folder is not None:
        output_folder = os.path.join(output_folder, planner_type)
        if os.path.exists(output_folder) == False:
            os.makedirs(output_folder)

    results = []

    for i in range(repeat):
        map = Map(map_json)

        if planner_type == "rrt":
            planner  = RRT_Planner(map)
        elif planner_type == "arrt":
            planner  = ARRT_Planner(map)

        start_time = time.time()
        path_found = planner.path_finding(map.start, map.goal, max_iter=10000)
        total_time = time.time() - start_time

        iterations, num_of_nodes = planner.dump_statistics()

        path_node_length = None
        path_cost = None

        if path_found:
            path, path_cost = planner.get_final_path()
            path_node_length = len(path)

        output_path = None
        if output_folder is not None:
            output_path = os.path.join(output_folder, os.path.basename(map_json).replace(".json", "_{}.png".format(i)))

        if output_path is not None:
            plt.savefig(output_path)

        plt.clf()

        data = {}
        data["time"] = total_time
        data["iterations"] = iterations
        data["nodes"] = num_of_nodes
        data["path_length"] = path_node_length
        data["path_cost"] = path_cost
        results.append(data)

    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="Input folder with maps.", default="./maps")
    parser.add_argument("--output", help="Output folder for generated path.", default=None)
    parser.add_argument("--repeat", help="number of repeat on each map.", type=int, default=5)
    args = parser.parse_args()

    if os.path.exists(args.input) == False:
        exit("[FAIL]: Input folder doesn't exist.")

    maps = find_all_maps(args.input)

    results = {}
    results[ "rrt"] = {}
    results["arrt"] = {}

    # run evaluation
    for map_json in maps:
        map_result_rrt  = execute_algorithm(map_json, planner_type="rrt", repeat=args.repeat, output_folder=args.output)
        map_result_arrt = execute_algorithm(map_json, planner_type="arrt", repeat=args.repeat, output_folder=args.output)

        map_key = os.path.basename(map_json).replace(".json", "")
        results[ "rrt"][map_key] = map_result_rrt
        results["arrt"][map_key] = map_result_arrt

    # save results
    with open(os.path.join(args.output, "summary_rrt.json"), "w") as f:
        print(simplejson.dumps(results[ "rrt"], ensure_ascii=False, indent=4), file=f)
    with open(os.path.join(args.output, "summary_arrt.json"), "w") as f:
        print(simplejson.dumps(results["arrt"], ensure_ascii=False, indent=4), file=f)


if __name__ == "__main__":
    main()
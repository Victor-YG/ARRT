import os
import argparse
import simplejson
import matplotlib.pyplot as plt


def load_data_from_json(file_path):
    with open(file_path, "r") as f:
        result = simplejson.load(f)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", help="Input json", action="append")
    parser.add_argument("--algo", help="Algorithm name associate with each json.", action="append")
    parser.add_argument("--output", help="Output folder to save plots.")
    parser.add_argument("--tag", help="Tag to save the plots.", default="eval")
    args = parser.parse_args()

    for json_path in args.json:
        if os.path.exists(json_path) == False:
            exit("Json file {} doesn't exist.".format(json_path))

    # load results
    results = []
    for json_path in args.json:
        result = load_data_from_json(os.path.abspath(json_path))
        results.append(result)

    # parse results
    exec_time = {}
    iterations = {}
    nodes = {}
    length = {}
    cost = {}

    for i in range(len(results)):
        algo_name = args.algo[i]

        exec_time[algo_name] = []
        iterations[algo_name] = []
        nodes[algo_name] = []
        length[algo_name] = []
        cost[algo_name] = []

        result = results[i]

        for map in result:
            for repeat in result[map]:
                # print(repeat)
                exec_time[algo_name].append(repeat["time"])
                iterations[algo_name].append(repeat["iterations"])
                nodes[algo_name].append(repeat["nodes"])
                length[algo_name].append(repeat["path_length"])
                cost[algo_name].append(repeat["path_cost"])

    # plot results
    plt.boxplot([exec_time[algo] for algo in exec_time], labels=args.algo)
    plt.title("Execution Time (s)")
    plt.savefig(os.path.join(args.output, "{}_exec_time.png".format(args.tag)))
    plt.cla()

    plt.boxplot([iterations[algo] for algo in iterations], labels=args.algo)
    plt.title("Iterations")
    plt.savefig(os.path.join(args.output, "{}_iterations.png".format(args.tag)))
    plt.cla()

    plt.boxplot([nodes[algo] for algo in nodes], labels=args.algo)
    plt.title("nodes")
    plt.savefig(os.path.join(args.output, "{}_nodes.png".format(args.tag)))
    plt.cla()

    plt.boxplot([length[algo] for algo in length], labels=args.algo)
    plt.title("Path Length in number of Nodes")
    plt.savefig(os.path.join(args.output, "{}_length.png".format(args.tag)))
    plt.cla()

    plt.boxplot([cost[algo] for algo in cost], labels=args.algo)
    plt.title("Path cost")
    plt.savefig(os.path.join(args.output, "{}_cost.png".format(args.tag)))
    plt.cla()


if __name__ == "__main__":
    main()
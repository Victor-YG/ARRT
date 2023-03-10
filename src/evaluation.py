import os
import argparse

from environment import Map
from rrt_planner import RRT_Planner
from rrts_planner import RRTS_Planner
from arrt_planner import ARRT_Planner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="Input folder with maps.", default="./maps")
    args = parser.parse_args()

    #TODO::run each algorithms

    #TODO::summarize and plot results


if __name__ == "__main__":
    main()
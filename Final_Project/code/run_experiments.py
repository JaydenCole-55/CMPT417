#!/usr/bin/python
import argparse
import glob
import time
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from MA_CBS import MA_CBSSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

SOLVER = "MA_CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized,MACBS}), defaults to ' + str(SOLVER))
    parser.add_argument('--MA_numConflicts', type=int, default=1,
                        help='Number of conflicts for MA-CBS before merging')

    args = parser.parse_args()

    zero_depth = 0

    if args.MA_numConflicts == 1:
        merge_bounds = [1]
        merge_bound_incr_type = [3]
    elif args.MA_numConflicts == 5:
        merge_bounds = [5]
        merge_bound_incr_type = [1, 2, 3, 4, 5]
    elif args.MA_numConflicts == 20:
        merge_bounds = [20]
        merge_bound_incr_type = [1, 2, 3, 4, 5]
    elif args.MA_numConflicts == 10:
        merge_bounds = [10]
        merge_bound_incr_type = [1, 2, 3, 4, 5]
    else:
        merge_bounds = [100]
        merge_bound_incr_type = [1]

    # Create sequential running instances
    for merge_bound in merge_bounds:
        for merge_bound_incr in merge_bound_incr_type:

            if args.solver == "CBS":
                result_file = open("CBS_results.csv", "w", buffering=1)
            elif args.solver == "MA_CBS":
                result_file = open("MA_CBS_results_MB" + str(merge_bound) + "_incrType" + str(merge_bound_incr) + ".csv", "w", buffering=1)
            else:
                result_file = open("results.csv", "w", buffering=1)

            start_time = time.time()

            for file in sorted(glob.glob(args.instance)):

                file_start_time = time.time()

                print("***Import an instance***")
                my_map, starts, goals = import_mapf_instance(file)
                print_mapf_instance(my_map, starts, goals)

                if args.solver == "CBS":
                    print("***Run CBS***")
                    cbs = CBSSolver(my_map, starts, goals)
                    paths = cbs.find_solution(args.disjoint)
                elif args.solver == "Independent":
                    print("***Run Independent***")
                    solver = IndependentSolver(my_map, starts, goals)
                    paths = solver.find_solution()
                elif args.solver == "Prioritized":
                    print("***Run Prioritized***")
                    solver = PrioritizedPlanningSolver(my_map, starts, goals)
                    paths = solver.find_solution()
                elif args.solver == "MA_CBS":
                    print("***Run Meta-Agent CBS***")
                    try:
                        MAcbs = MA_CBSSolver(my_map, starts, goals, merge_bound, zero_depth, file_start_time, merge_bound_incr)
                        paths = MAcbs.find_solution(args.disjoint)
                    except:
                        paths = None
                else:
                    raise RuntimeError("Unknown solver!")

                if paths is not None:
                    cost = get_sum_of_cost(paths)

                file_total_time = time.time() - file_start_time

                if paths is not None:
                    result_file.write("{},{},{}\n".format(file, cost, file_total_time))
                else:
                    result_file.write("{},{},{}\n".format(file, -1, file_total_time))

                if not args.batch:
                    print("***Test paths on a simulation***")
                    print("\nTotal time: " + str(file_total_time) + "s")
                    animation = Animation(my_map, starts, goals, paths)
                    # animation.save("output.mp4", 1.0)
                    animation.show()

    if not args.batch:
        total_time = time.time() - start_time
        print(total_time)

    result_file.close()

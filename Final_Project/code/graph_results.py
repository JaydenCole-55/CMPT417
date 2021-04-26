# Graph the data from the compile_data file

from compile_data import compile_data
import matplotlib.pyplot as plt

data_indicies = ["File name", "Merge Bound", "Merge Increment", "% A5 Solved", "% A10 Solved", "% A15 Solved", "% A20 Solved", 
                 "% O00 Solved", "% O05 Solved", "% O10 Solved", "% O15 Solved",
                 "% S10 Solved", "% S12 Solved", "% S14 Solved", "% S16 Solved", "% S18 Solved", "% S20 Solved",
                 "Avg Time A5 Solved", "Avg Time A10 Solved", "Avg Time A15 Solved", "Avg Time A20 Solved",
                 "Avg Time O00 Solved", "Avg Time O05 Solved", "Avg Time O10 Solved", "Avg Time O15 Solved",
                 "Avg Time S10 Solved", "Avg Time S12 Solved", "Avg Time S14 Solved", "Avg Time S16 Solved", "Avg Time S18 Solved", "Avg Time S20 Solved"]

def plot_solved_vs_agents(result_file):
    lines = result_file.readlines()

    A05_index = data_indicies.index("% A5 Solved")
    A10_index = data_indicies.index("% A10 Solved")
    A15_index = data_indicies.index("% A15 Solved")
    A20_index = data_indicies.index("% A20 Solved")

    merge_incr_index  = data_indicies.index("Merge Increment")
    merge_bound_index = data_indicies.index("Merge Bound")

    i = 0
    for line in lines:
        if i == 0:
            i+=1
            continue

        split_line = line.split(',')

        if split_line[merge_incr_index] != "1":
            continue

        legend_string = "Merge Bound: " + split_line[merge_bound_index] + " Increment: " + split_line[merge_incr_index]

        perc_solved = [ float(split_line[A05_index]) * 100, 
                        float(split_line[A10_index]) * 100, 
                        float(split_line[A15_index]) * 100, 
                        float(split_line[A20_index]) * 100 ]
        plt.plot([5, 10, 15, 20], perc_solved, label=legend_string)

    plt.title("% Solved Instances vs Number of Agents")
    plt.legend(loc="lower left")
    plt.ylabel('% Of Instances Solved')
    plt.xlabel('Num Agents')
    plt.show()


def plot_tsolve_vs_agents(result_file):
    lines = result_file.readlines()

    A05t_index = data_indicies.index("Avg Time A5 Solved")
    A10t_index = data_indicies.index("Avg Time A10 Solved")
    A15t_index = data_indicies.index("Avg Time A15 Solved")
    A20t_index = data_indicies.index("Avg Time A20 Solved")

    merge_incr_index  = data_indicies.index("Merge Increment")
    merge_bound_index = data_indicies.index("Merge Bound")

    i = 0
    for line in lines:
        if i == 0:
            i+=1
            continue

        split_line = line.split(',')

        # if split_line[merge_incr_index] != "2":
        #     continue

        legend_string = "Merge Bound: " + split_line[merge_bound_index] + " Increment: " + split_line[merge_incr_index]

        perc_solved = [ float(split_line[A05t_index]), 
                        float(split_line[A10t_index]), 
                        float(split_line[A15t_index]), 
                        float(split_line[A20t_index]) ]
        plt.plot([5, 10, 15, 20], perc_solved, label=legend_string)

    plt.title("Avg Time (s) vs Number of Agents")
    plt.legend(loc="lower right")
    plt.ylabel('Avg Time (s) on Solved Intances')
    plt.xlabel('Num Agents')
    plt.show()


def plot_solved_vs_obst(result_file):
    lines = result_file.readlines()

    O00_index = data_indicies.index("% O00 Solved")
    O05_index = data_indicies.index("% O05 Solved")
    O10_index = data_indicies.index("% O10 Solved")
    O15_index = data_indicies.index("% O15 Solved")

    merge_incr_index  = data_indicies.index("Merge Increment")
    merge_bound_index = data_indicies.index("Merge Bound")

    i = 0
    for line in lines:
        if i == 0:
            i+=1
            continue

        split_line = line.split(',')

        if split_line[merge_bound_index] != "1":
            continue

        legend_string = "Merge Bound: " + split_line[merge_bound_index] + " Increment: " + split_line[merge_incr_index]

        perc_solved = [ float(split_line[O00_index]) * 100, 
                        float(split_line[O05_index]) * 100, 
                        float(split_line[O10_index]) * 100, 
                        float(split_line[O15_index]) * 100 ]
        plt.plot([0, 5, 10, 15], perc_solved, label=legend_string)

    plt.title("% Solved Instances vs % Coverage of Obstacles")
    plt.legend(loc="lower left")
    plt.ylabel('% Of Instances Solved')
    plt.xlabel('% Coverage of Obstacles')
    plt.show()


def plot_tsolved_vs_obst(result_file):
    lines = result_file.readlines()

    O00t_index = data_indicies.index("Avg Time O00 Solved")
    O05t_index = data_indicies.index("Avg Time O05 Solved")
    O10t_index = data_indicies.index("Avg Time O10 Solved")
    O15t_index = data_indicies.index("Avg Time O15 Solved")

    merge_incr_index  = data_indicies.index("Merge Increment")
    merge_bound_index = data_indicies.index("Merge Bound")

    i = 0
    for line in lines:
        if i == 0:
            i+=1
            continue

        split_line = line.split(',')

        legend_string = "Merge Bound: " + split_line[merge_bound_index] + " Increment: " + split_line[merge_incr_index]

        perc_solved = [ float(split_line[O00t_index]) * 100, 
                        float(split_line[O05t_index]) * 100, 
                        float(split_line[O10t_index]) * 100, 
                        float(split_line[O15t_index]) * 100 ]
        plt.plot([0, 5, 10, 15], perc_solved, label=legend_string)

    plt.title("Avg. Time (s) per solve vs % Coverage of Obstacles")
    plt.legend(loc="lower right")
    plt.ylabel('Avg. Time (s) per solve')
    plt.xlabel('% Coverage of Obstacles')
    plt.show()


def plot_solved_vs_size(result_file):
    lines = result_file.readlines()

    S10_index = data_indicies.index("% S10 Solved")
    S12_index = data_indicies.index("% S12 Solved")
    S14_index = data_indicies.index("% S14 Solved")
    S16_index = data_indicies.index("% S16 Solved")
    S18_index = data_indicies.index("% S18 Solved")
    S20_index = data_indicies.index("% S20 Solved")

    merge_incr_index  = data_indicies.index("Merge Increment")
    merge_bound_index = data_indicies.index("Merge Bound")

    i = 0
    for line in lines:
        if i == 0:
            i+=1
            continue

        split_line = line.split(',')

        if split_line[merge_incr_index] != "1":
            continue

        legend_string = "Merge Bound: " + split_line[merge_bound_index] + " Increment: " + split_line[merge_incr_index]

        perc_solved = [ float(split_line[S10_index]) * 100, 
                        float(split_line[S12_index]) * 100, 
                        float(split_line[S14_index]) * 100,
                        float(split_line[S16_index]) * 100,
                        float(split_line[S18_index]) * 100, 
                        float(split_line[S20_index]) * 100 ]

        plt.plot([10, 12, 14, 16, 18, 20], perc_solved, label=legend_string)

    plt.title("% Solved Instances vs Size of MAPF Instance")
    plt.legend(loc="lower right")
    plt.ylabel('% Of Instances Solved')
    plt.xlabel('Width and Height of Instance')
    plt.show()


def plot_tsolved_vs_size(result_file):
    lines = result_file.readlines()

    S10t_index = data_indicies.index("Avg Time S10 Solved")
    S12t_index = data_indicies.index("Avg Time S12 Solved")
    S14t_index = data_indicies.index("Avg Time S14 Solved")
    S16t_index = data_indicies.index("Avg Time S16 Solved")
    S18t_index = data_indicies.index("Avg Time S18 Solved")
    S20t_index = data_indicies.index("Avg Time S20 Solved")

    merge_incr_index  = data_indicies.index("Merge Increment")
    merge_bound_index = data_indicies.index("Merge Bound")

    i = 0
    for line in lines:
        if i == 0:
            i+=1
            continue

        split_line = line.split(',')

        legend_string = "Merge Bound: " + split_line[merge_bound_index] + " Increment: " + split_line[merge_incr_index]

        perc_solved = [ float(split_line[S10t_index]) * 100, 
                        float(split_line[S12t_index]) * 100, 
                        float(split_line[S14t_index]) * 100,
                        float(split_line[S16t_index]) * 100,
                        float(split_line[S18t_index]) * 100, 
                        float(split_line[S20t_index]) * 100 ]

        plt.plot([10, 12, 14, 16, 18, 20], perc_solved, label=legend_string)

    plt.title("Avg. Time (s) per Solve vs Size of MAPF Instance")
    plt.legend(loc="lower right")
    plt.ylabel('Avg. Time (s) per Solve')
    plt.xlabel('Width of Instance')
    plt.show()


def plot_data():
    # Update data in data file
    compile_data()

    result_file = open("compiled_results.csv", "r")

    #plot_solved_vs_agents(result_file)

    #plot_tsolve_vs_agents(result_file)
    
    plot_solved_vs_obst(result_file)

    #plot_tsolved_vs_obst(result_file)

    # plot_solved_vs_size(result_file)

    #plot_tsolved_vs_size(result_file)


if __name__ == "__main__":
    plot_data()
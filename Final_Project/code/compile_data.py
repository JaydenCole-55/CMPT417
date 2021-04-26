import matplotlib as m
import os

#############################################
#
# GLOBALS
#
#############################################

#############################################
#
# FUNCTIONS
#
#############################################

def get_size(filename, short_agent, short_obst):
    if short_agent and short_obst:
        # both agent amd obstacle are shorten, affects splice strings
        return int(filename[33:35])
    elif short_agent or short_obst:
        # only need to offset by one index
        return int(filename[34:36])
    else:
        # Both agent and obstacle are two indicies, 
        return int(filename[35:37])


def get_obst(filename, short_agent):
    # Two splices are possible based on length of agent,
    if not short_agent:
        # Determine if the obst is one or two digits in length
        first_digit = int(filename[27:28])
        if first_digit == 0 or first_digit == 5:
            return first_digit
        else:
            return int(filename[27:29])
    else:
        # The agent was only one index long, change splice parameters
        first_digit = int(filename[26:27])
        if first_digit == 0 or first_digit == 5:
            return first_digit
        else:
            return int(filename[26:28])


def get_agents(filename):
    if int(filename[20:21]) == 5:
        # Only 5 agents, only slice that one index
        return 5
    else:
        return int(filename[20:22])


def get_data(line):
    line = line[:-2] # Remove new line character

    # Split the data from the line
    [filename, cost, time] = line.split(',')

    # Number of agents is stored in test file name
    agents = get_agents(filename)

    # Slicing is different for agent = 5
    short_agent = False
    if agents == 5:
        short_agent = True

    obst = get_obst(filename, short_agent)
    
    short_obst = False
    if obst < 10:
        short_obst = True

    size = get_size(filename, short_agent, short_obst)

    cost = int(cost)
    time = float(time)

    return [agents, obst, size, cost, time]


def good_line(line):
    # Check that the line of data makes sense
    try:
        # Try to split the data from the line
        [filename, cost, time] = line.split(',')

        # Access one point where data read may fail in certain cases
        if 'FinalProjInstances\\A5' not in filename:
            int(filename[20:22])
        
        return True
    except:
        return False


def get_merge_details(filename):
    # Get the merge bound and the increment type for the test output file

    # Merge incr is easy pickings
    merge_incr = int(filename[-5])

    split_filename = filename.split('_')
    merge_bound = int(split_filename[3][2:])

    return [merge_bound, merge_incr]


def compile_data():

    RESULT_FILE_PREFIX = "MA_CBS_results_MB"
    cur_path = os.path.dirname(__file__)
    
    # Open a csv to store all the data
    result_file = open("compiled_results.csv", "w")

    # Set up headers
    result_file.write("{},{},{},{},{},{},{},".format("File name", "Merge Bound", "Merge Increment", "% A5 Solved", "% A10 Solved", "% A15 Solved", "% A20 Solved"))
    result_file.write("{},{},{},{},".format("% O00 Solved", "% O05 Solved", "% O10 Solved", "% O15 Solved"))
    result_file.write("{},{},{},{},{},{},".format("% S10 Solved", "% S12 Solved", "% S14 Solved", "% S16 Solved", "% S18 Solved", "% S20 Solved",))
    result_file.write("{},{},{},{},".format("Avg Time A5 Solved", "Avg Time A10 Solved", "Avg Time A15 Solved", "Avg Time A20 Solved"))
    result_file.write("{},{},{},{},".format("Avg Time O00 Solved", "Avg Time O05 Solved", "Avg Time O10 Solved", "Avg Time O15 Solved"))
    result_file.write("{},{},{},{},{},{}\n".format("Avg Time S10 Solved", "Avg Time S12 Solved", "Avg Time S14 Solved", "Avg Time S16 Solved", "Avg Time S18 Solved", "Avg Time S20 Solved",))
    
    # Get all result file csvs in the directory and iteratively go through them
    for files in os.listdir(cur_path):
        
        if RESULT_FILE_PREFIX not in files:
            continue

        [merge_bound, merge_incr] = get_merge_details(files)
        
        file = open(files, 'r')
        
        TOTAL_INSTANCES = 0.0

        A05_SOLVED  = 0
        A10_SOLVED = 0
        A15_SOLVED = 0
        A20_SOLVED = 0

        A05_SOLVED_TIME = 0
        A10_SOLVED_TIME = 0
        A15_SOLVED_TIME = 0
        A20_SOLVED_TIME = 0

        O00_SOLVED = 0
        O05_SOLVED = 0
        O10_SOLVED = 0
        O15_SOLVED = 0
        
        O00_SOLVED_TIME = 0
        O05_SOLVED_TIME = 0
        O10_SOLVED_TIME = 0
        O15_SOLVED_TIME = 0

        S10_SOLVED = 0
        S12_SOLVED = 0
        S14_SOLVED = 0
        S16_SOLVED = 0
        S18_SOLVED = 0
        S20_SOLVED = 0

        S10_SOLVED_TIME = 0
        S12_SOLVED_TIME = 0
        S14_SOLVED_TIME = 0
        S16_SOLVED_TIME = 0
        S18_SOLVED_TIME = 0
        S20_SOLVED_TIME = 0

        lines = file.readlines()

        for line in lines:
            if not good_line(line):
                continue

            TOTAL_INSTANCES += 1

            # Get the data for the line
            [agents, obst, size, cost, time] = get_data(line)

            if cost == -1:
                continue

            if agents == 5:
                A05_SOLVED += 1
                A05_SOLVED_TIME += time
            elif agents == 10:
                A10_SOLVED += 1
                A10_SOLVED_TIME += time
            elif agents == 15:
                A15_SOLVED += 1
                A15_SOLVED_TIME += time
            else:
                A20_SOLVED += 1
                A20_SOLVED_TIME += time

            if obst == 0:
                O00_SOLVED += 1
                O00_SOLVED_TIME = time
            elif obst == 5:
                O05_SOLVED += 1
                O05_SOLVED_TIME += time
            elif obst == 10:
                O10_SOLVED += 1
                O10_SOLVED_TIME += time
            else:
                O15_SOLVED += 1
                O15_SOLVED_TIME += time

            if size == 10:
                S10_SOLVED += 1
                S10_SOLVED_TIME += time
            elif size == 12:
                S12_SOLVED += 1
                S12_SOLVED_TIME += time
            elif size == 14:
                S14_SOLVED += 1
                S14_SOLVED_TIME += time
            elif size == 16:
                S16_SOLVED += 1
                S16_SOLVED_TIME += time
            elif size == 18:
                S18_SOLVED += 1
                S18_SOLVED_TIME += time
            else:
                S20_SOLVED += 1
                S20_SOLVED_TIME += time

        percent_A05_solved = A05_SOLVED/(TOTAL_INSTANCES/4.0)
        percent_A10_solved = A10_SOLVED/(TOTAL_INSTANCES/4.0)
        percent_A15_solved = A15_SOLVED/(TOTAL_INSTANCES/4.0)
        percent_A20_solved = A20_SOLVED/(TOTAL_INSTANCES/4.0)

        avg_A05_time = A05_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_A10_time = A10_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_A15_time = A15_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_A20_time = A20_SOLVED_TIME/(TOTAL_INSTANCES/4)

        percent_O00_solved = O00_SOLVED/(TOTAL_INSTANCES/4)
        percent_O05_solved = O05_SOLVED/(TOTAL_INSTANCES/4)
        percent_O10_solved = O10_SOLVED/(TOTAL_INSTANCES/4)
        percent_O15_solved = O15_SOLVED/(TOTAL_INSTANCES/4)

        avg_O00_time = O00_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_O05_time = O05_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_O10_time = O10_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_O15_time = O15_SOLVED_TIME/(TOTAL_INSTANCES/4)

        percent_S10_solved = S10_SOLVED/(TOTAL_INSTANCES/6)
        percent_S12_solved = S12_SOLVED/(TOTAL_INSTANCES/6)
        percent_S14_solved = S14_SOLVED/(TOTAL_INSTANCES/6)
        percent_S16_solved = S16_SOLVED/(TOTAL_INSTANCES/6)
        percent_S18_solved = S18_SOLVED/(TOTAL_INSTANCES/6)
        percent_S20_solved = S20_SOLVED/(TOTAL_INSTANCES/6)

        avg_S10_time = S10_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_S12_time = S12_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_S14_time = S14_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_S16_time = S16_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_S18_time = S18_SOLVED_TIME/(TOTAL_INSTANCES/4)
        avg_S20_time = S20_SOLVED_TIME/(TOTAL_INSTANCES/4)

        # Write the data to the result file
        result_file.write("{},{},{},".format(           files, merge_bound, merge_incr))

        result_file.write("{},{},{},{},".format(        percent_A05_solved, percent_A10_solved, percent_A15_solved, percent_A20_solved))
        result_file.write("{},{},{},{},".format(        percent_O00_solved, percent_O05_solved, percent_O10_solved,percent_O15_solved))
        result_file.write("{},{},{},{},{},{},".format(  percent_S10_solved, percent_S12_solved, percent_S14_solved, percent_S16_solved, percent_S18_solved, percent_S20_solved))

        result_file.write("{},{},{},{},".format(        avg_A05_time, avg_A10_time, avg_A15_time, avg_A20_time))
        result_file.write("{},{},{},{},".format(        avg_O00_time, avg_O05_time, avg_O10_time, avg_O15_time))
        result_file.write("{},{},{},{},{},{}\n".format( avg_S10_time, avg_S12_time, avg_S14_time, avg_S16_time, avg_S18_time, avg_S20_time))
        
        print("Done solving instances")
            
    result_file.close()


if __name__ == "__main__":
    compile_data()
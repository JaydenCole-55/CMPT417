import random
import os

# Create and save the test instances for the MA-CBS and CBS final report
cur_path = os.path.dirname(__file__)
test_directory = os.path.join(cur_path, "FinalProjInstances")

def find_good_cell(MAPF_map):
    # Find a good_cell in the map
    good_cell = '.'
    found = False
    while not found:
        beg = random.randint(0, len(MAPF_map))

        # Try to find an index of a good cell on the map to replace
        index = MAPF_map.find(good_cell, beg)

        # If no good cell in this range, try again
        if index != -1:
            found = True

    return index


def generate_map(size, p_obst):

    good_cell = '.'
    bad_cell  = "@"
    EOL       = "\n"

    line = (good_cell + " ") * size + EOL
    MAPF_map = line * size

    num_obst = int(size * size * 0.01 * p_obst)

    i = 0
    while i < num_obst:
        # Replace a good cell with an obstical
        index = find_good_cell(MAPF_map)

        before = MAPF_map[0:index]
        after  = MAPF_map[index+1:]

        MAPF_map = before + bad_cell + after

        i += 1

    return MAPF_map


def make_MAPF(agents, p_obst, size, filename):
    # Make a single MAPF instance and save to filename
    file = open(filename, 'w')

    # Write the soze of the intance to the file
    file.write(str(size) + " " + str(size) + "\n")

    # Create the map for the instance
    MAPF_map = generate_map(size, p_obst)

    # Write the map to the file
    file.write(MAPF_map)

    file.write(str(agents) + "\n")

    start_indicies = []
    goal_indicies  = []

    for agent in range(agents):
        # Find a good cells to make a start and goal nodes
        start_index = -1
        goal_index  = start_index

        while (start_index == goal_index and start_index == -1) or (start_index in start_indicies) or (goal_index in goal_indicies):
            start_index = find_good_cell(MAPF_map)        
            goal_index  = find_good_cell(MAPF_map)

        start_indicies.append(start_index)
        goal_indicies.append(goal_index)

        # Convert indicies to x and y coordinates
        line_length = 2*(size-1)+3

        start_x = str( int(start_index / line_length)  )
        start_y = str( int( (start_index % (line_length))/2 ) )
        end_x   = str( int(goal_index  / line_length) )
        end_y   = str( int( (goal_index  % (line_length))/2 ) )

        assert int(start_x) < size
        assert int(start_y) < size
        assert int(end_x) < size
        assert int(end_y) < size

        assert int(start_x) > -1
        assert int(start_y) > -1
        assert int(end_x)   > -1
        assert int(end_y)   > -1

        file.write(start_x + " " + start_y + " " + end_x + " " + end_y + "\n")

    file.close()

def main():
    ''' 
    Create MAPF test instances with a number of agents, % obstacles, and a size range
    '''

    num_instances = 20

    num_agents    = 5
    while num_agents <= 20:

        percent_obst = 0
        while percent_obst <= 15:

            size = 10
            while size <= 20:

                i = 1
                while i <= num_instances:

                    filename = "A" + str(num_agents) + "_obst" + str(percent_obst) + "_size_" + str(size) + "x" + str(size) + "_inst_" + str(i) + ".txt"
                    abs_path_to_file = os.path.join(test_directory, filename)
                    # Make a MAPF instance with the above parameters
                    make_MAPF(num_agents, percent_obst, size, abs_path_to_file)

                    i += 1
                size += 2
            percent_obst += 5
        num_agents += 5


if __name__ == "__main__":
   main() 
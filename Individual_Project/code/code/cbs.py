import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    # Vertex collisions will have the form {'a1':agent#1, 'a2':agent#2, 'loc':[(#,#)], 'timestep':#}
    # Edge collisions will have the form {'a1':agent#1, 'a2':agent#2, 'loc':[(#,#), (#,#)], 'timestep':#} 
    
    for timestep in range(max(len(path1), len(path2))):
        a1_curr_loc = get_location(path1, timestep)
        a2_curr_loc = get_location(path2, timestep)

        # Check for vertex constraint
        if a1_curr_loc == a2_curr_loc:
            return [timestep, [a1_curr_loc]]

        a1_next_loc = get_location(path1, timestep+1)
        a2_next_loc = get_location(path2, timestep+1)

        # Check for edge constraint
        if a1_curr_loc == a2_next_loc and a1_next_loc == a2_curr_loc:
            return [timestep, [a1_curr_loc, a1_next_loc]]

    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    # Recursive funtion that checks one path at a time against the rest of the paths
    collisions = []

    # Base case for the recursion 
    if len(paths) == 1:
        return collisions

    # Get last path agent number, then pop off path list temporarily
    agent = len(paths)-1
    path = paths.pop(-1)

    # Iterate through the rest of the paths
    for next_agent in range(len(paths)):
        agent_dict = {'a1': agent, 'a2': next_agent}

        path_collision = detect_collision(path, paths[next_agent])
        if path_collision is not None:
            agent_dict['timestep'] = path_collision[0]
            agent_dict['loc'] =  path_collision[1]
            collisions.append(agent_dict)
    
    # Recursively call detect collisions until base case
    collisions += detect_collisions(paths)

    # Add back removed path
    paths.append(path)

    # Return all collisions previous checked paths
    return collisions



def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    # Detect if it is a vertex collision
    if(len(collision['loc']) == 1):
        return [ 
            {'agent' : collision['a1'], 'loc' : collision['loc'], 'timestep' : collision['timestep']},
            {'agent' : collision['a2'], 'loc' : collision['loc'], 'timestep' : collision['timestep']}
        ]
    else:
        return [
            {'agent' : collision['a1'], 'loc' : [ collision['loc'][0], collision['loc'][1] ], 'timestep' : collision['timestep']},
            {'agent' : collision['a2'], 'loc' : [ collision['loc'][1], collision['loc'][0] ], 'timestep' : collision['timestep']}
        ]


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


#
# Please insert this function into "cbs.py" before "class CBSSolver"
# is defined.
#

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def display_node(self, node, expanded):
        if(expanded):
            print("\nExpanded node cost: {}".format(node['cost']))
            print("Expanded node constraints: {}".format(node['constraints']))
            print("Expanded node collisions: {}".format(node['collisions']))
            print("Expanded node paths: {}\n".format(node['paths']))
        else:
            print("\nConstrained node cost: {}".format(node['cost']))
            print("Constrained node constraints: {}".format(node['constraints']))
            print("Constrained node collisions: {}".format(node['collisions']))
            print("Constrained node paths: {}\n".format(node['paths']))

    def create_new_node(self, parent, new_constraint):
        new_node = { 'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': []}

        for path in parent['paths']:
            new_node['paths'].append(path)

        for constraint in parent['constraints']:
            new_node['constraints'].append(constraint)

        new_node['constraints'] += [new_constraint]

        return new_node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        #print(root['collisions'])

        # Task 3.2: Testing
        #for collision in root['collisions']:
            #print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while(len(self.open_list) != 0):
            next_node = self.pop_node()

            #self.display_node(next_node, True)

            if len(next_node['collisions']) == 0:
                return next_node['paths']

            collision = next_node['collisions'][-1]

            #print(standard_splitting(collision))

            constraints = standard_splitting(collision)

            i = 1
            for constraint in constraints:
                # Create a new node with the new constaint

                new_node = self.create_new_node(next_node, constraint)

                # Get which agent is constrained
                agent_constrained = constraint['agent']

                # Determine the agent's new path
                path = a_star(self.my_map, self.starts[agent_constrained], self.goals[agent_constrained], 
                            self.heuristics[agent_constrained], agent_constrained, new_node['constraints'])
                
                # Path with no length, don't add it to lists
                if path is None:
                    continue

                # Update new node's information based on new path
                new_node['paths'][agent_constrained] = path
                new_node['collisions'] = detect_collisions(new_node['paths'])
                new_node['cost'] = get_sum_of_cost(new_node['paths'])

                #print("Adding constrained node {} of {}".format(i, len(constraints)) )
                #self.display_node(new_node, False)

                i += 1

                # Push the new node to the open list
                self.push_node(new_node)


        #self.print_results(root)
        #return root['paths']
        raise BaseException('No solutions')

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

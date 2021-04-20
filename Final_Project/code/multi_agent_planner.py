import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)] 
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraint_table = []

    # Temporary table to hold constraints due to agents in their final square
    indefinite_constraints = []
    max_indefinite_constraint_timestep = 0

    for constraint in constraints:
        # Check if the constraint applies to this agent
        if (not constraint['positive'] and agent != constraint['agent']):
            continue

        # Dynamically add more time step indicies to the table if needed
        if constraint['timestep'] >= len(constraint_table):
            constraint_table += [[]] * (constraint['timestep'] - len(constraint_table) + 1)

        # A negative timestep indicates this constraint extends indefinitely after abs(timestep), take care of these constraints last
        if constraint['timestep'] < 0:
            indefinite_constraints += [constraint]
            if abs(constraint['timestep']) > max_indefinite_constraint_timestep:
                max_indefinite_constraint_timestep = abs(constraint['timestep'])

        # Add constraint to constraint table, determine if it is a pos constraint for another agent or a constraint on itself
        if constraint['positive'] and agent != constraint['agent']:
            constraint_table[constraint['timestep']] = constraint_table[constraint['timestep']] + [[constraint['loc'], False]]
        else:
            constraint_table[constraint['timestep']] = constraint_table[constraint['timestep']] + [[constraint['loc'], constraint['positive']]]

    # If the max indefinite constraint timestep is bigger than current table size, increase the table to fit them
    if max_indefinite_constraint_timestep >= len(constraint_table):
        constraint_table += [[]] * (max_indefinite_constraint_timestep - len(constraint_table) + 1)
    else:
        constraint_table += [[]] # Stores indefinite constraints in extra row in the table

    # Insert indefinite constraints from when they occur to the length of the table
    for constraint in indefinite_constraints:
        for i in range( abs(constraint['timestep']), len(constraint_table) ):
            constraint_table[i] = constraint_table[i] + [[constraint['loc'], constraint['positive']]]

    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_pos_constraint(constraint):
    return constraint[1]


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    # Check if this move interupts an agent in its final location
    if next_time >= len(constraint_table):
        return [[next_loc], False] in constraint_table[-1]

    # Determine if there is a constraint on this move
    is_pos_v_constraint = [[next_loc], True] in constraint_table[next_time]
    is_pos_e_constraint = [[(curr_loc), (next_loc)], True] in constraint_table[next_time-1] or [[(next_loc), (curr_loc)], True] in constraint_table[next_time-1]
    is_neg_v_constraint = [[next_loc], False] in constraint_table[next_time]
    is_neg_e_constraint = [[(curr_loc), (next_loc)], False] in constraint_table[next_time-1] or [[(next_loc), (curr_loc)], False] in constraint_table[next_time-1]

    if is_pos_v_constraint:
        return is_neg_e_constraint # Check that a positive v constraint does not break a negative edge contraint

    if is_pos_e_constraint:
        return is_neg_v_constraint # Check that a poitive edge constraint does not break a negative vertex constraint

    # Does the next location break a negative vertex constraint
    if is_neg_v_constraint:
        return True

    # Check for a negative edge constraint
    if is_neg_e_constraint:
        return True

    # No constraint for this move
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def constrained_in_future(node, constraint_table):
    """ Return True if the node is in a negative constraint at some timestep in the future"""

    # Check if the timestep is greater than greatest timestep of constraints
    if node['timestep'] >= len(constraint_table):
        return [[node['loc']], False] in constraint_table[-1]

    # Check for every timestep in the future if goal node is vertex constrained
    for i in range( node['timestep'], len(constraint_table) ):
        if [[node['loc']], False] in constraint_table[i]:
            return True

    return False


def joint_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agents that are being re-planned
        constraints - constraints defining where robots should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    latest_goal_timestep = 2 * ( len(my_map) )^4 # Upper bound on number of timesteps searched
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent) # Build constraint table
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0} # Add timestep of 0 to root
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root  # Make indexing of closed list the location and time step
    while len(open_list) > 0:
        curr = pop_node(open_list)
        if curr['timestep'] > latest_goal_timestep: # Number of timesteps generated exceeds maximum number of timesteps permitted to be searched
            break
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints --> Added function to check if the goal node was constrained in the future
        if curr['loc'] == goal_loc and not constrained_in_future(curr, constraint_table):
            return get_path(curr)
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]) or my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1} # Add child timestep as 1 greater than parents
            if is_constrained(curr['loc'], child['loc'], child['timestep'], constraint_table ): # Prune children that are constrained
                continue
            if (child['loc'], child['timestep']) in closed_list: # Check if location and time is already occupied in closed list
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions

import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_first_collision(path1, path2):
    ##############################
    #           Return the first collision that occurs between two robot paths (or None if there is no collision)
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


def detect_collisions(paths, metaAgentGroups):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_first_collision function to find a collision between two robots.

    # Recursive function that checks one path at a time against the rest of the paths
    collisions = []

    # Base case for the recursion, no collisions can happen on one path
    if len(paths) == 1:
        return collisions

    # Get last path agent number, then pop off its path from the path list temporarily
    agent = len(paths)-1
    path = paths.pop(-1)

    # Iterate through the rest of the paths detecting any collisions with this path
    for next_agent in range(len(paths)):
        # Get the meta agent each low level agent belongs to
        MA1 = which_meta_agent(agent, metaAgentGroups)
        MA2 = which_meta_agent(next_agent, metaAgentGroups)

        # Store the collision as a collision of their meta agents
        agent_dict = {'a1': MA1, 'a2': MA2}

        # Find if there is a collision
        path_collision = detect_first_collision(path, paths[next_agent])

        # Ensure that a collision is only between differing Meta-Agents
        if (path_collision is not None) and (MA1 != MA2):            
            agent_dict['timestep'] = path_collision[0]
            agent_dict['loc'] =  path_collision[1]
            collisions.append(agent_dict)
        # Collisions should not happen within a Meta agent
        elif (MA1 == MA2) and (path_collision is not None):
            raise(ValueError)
    
    # Recursively call detect collisions with one less path until base case
    collisions += detect_collisions(paths, metaAgentGroups)

    # Add back removed path
    paths.append(path)

    # Return all collisions previous checked paths
    return collisions


#################################################
#
# META-AGENT CBS FUNCTIONS
#
#################################################
def detect_num_collisions(path1, path2):
    ##############################
    # Detect the number of collisions between these two paths

    num_cols = 0

    for timestep in range(max(len(path1), len(path2))):
        a1_curr_loc = get_location(path1, timestep)
        a2_curr_loc = get_location(path2, timestep)

        # Check for vertex constraint
        if a1_curr_loc == a2_curr_loc:
            num_cols += 1

        a1_next_loc = get_location(path1, timestep+1)
        a2_next_loc = get_location(path2, timestep+1)

        # Check for edge constraint
        if a1_curr_loc == a2_next_loc and a1_next_loc == a2_curr_loc:
            num_cols += 1

    return num_cols


def calc_collision_mat(paths, MA_groups):
    ##############################
    # Calculates the collision matrix for between meta agents

    num_MA = len(MA_groups)
    num_agents = len(paths)

    # Create an empty collision matrix
    collision_mat = [ [ 0 for i in range(num_MA) ] for j in range(num_MA) ]

    # For all meta agents, detect number of collisions with all other meta agents and store in collision matrix
    for agent1 in range(num_agents-1):
        # Get the meta agent for this agent
        MA1 = which_meta_agent(agent1, MA_groups)

        for agent2 in range(agent1+1, num_agents):
            # Get the meta agent for this agent
            MA2 = which_meta_agent(agent2, MA_groups)

            # Calculate the number of collisions between the two lower level agents
            if(MA1 == MA2):
                num_cols = 0
            else:
                num_cols = detect_num_collisions( paths[agent1], paths[agent2] )
            
            # Add the collisions to their meta agent collision matrix
            collision_mat[MA1][MA2] += num_cols
            collision_mat[MA2][MA1] += num_cols

    return collision_mat


def should_merge(a1, a2, collisionMatrix, mergeBound):
    ##############################
    # Detects if two agents should merge into a meta agent based on the number of conflicts
    return collisionMatrix[a1][a2] >= mergeBound


def which_meta_agent(agent, metaAgentGroups):
    ###############################
    # Finds which meta agent group an agent belongs to

    for meta_agent in range( len(metaAgentGroups) ):
        if agent in metaAgentGroups[meta_agent]:
            return meta_agent

    # Agent is not in a meta agent, this should be impossible. Throw error
    raise(ValueError)


def combine_constraints(MA1, MA2, MA_groups, constraints):
    ##############################
    # Combine the constraints between the two Meta agent groups
    #   Determine if the constraints are internal constraints (beteween these two meta agents), 
    #   or external constraints (between one of these agents and another agent)

    # Combine the constraints that apply to agents in MA2 to MA1
    for constraint in constraints:
        # If the constraint applies to MA2, make it now apply to MA1
        if constraint['agent'] == MA2:
            constraint['agent'] = MA1

    return constraints


def merge(MA1, MA2, MA_groups, node_contraints):
    ###############################
    # Merge two meta agents into one meta agent

    # Combine the contraints of the two meta-agents
    node_constraints = combine_constraints(MA1, MA2, MA_groups, node_contraints)

    # Combine the two meta agents into one
    MA_groups[MA1] += MA_groups[MA2]

    # Clear the second meta agent group
    MA_groups.pop(MA2)

    # Return the updated group and contraints
    return [ MA_groups, node_contraints ]


def path_violates_constraint(constraint, paths):
    ''' Determine if any paths violate given constraint '''
    #assert constraint['positive'] is True
    agent = constraint['agent']
        
    curr = get_location(paths[agent], constraint['timestep'])
    next = get_location(paths[agent], constraint['timestep'] + 1)
    if len(constraint['loc']) == 1:  # vertex constraint
        if constraint['loc'][0] == curr:
            return True
    else:  # edge constraint
        if constraint['loc'] == [next, curr] or constraint['loc'] == [curr, next]:
            return True
    return False

#################################################
#
# END META-AGENT CBS FUNCTIONS
#
#################################################

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
            {'agent' : collision['a1'], 'loc' : collision['loc'], 'timestep' : collision['timestep'], 'positive' : False},
            {'agent' : collision['a2'], 'loc' : collision['loc'], 'timestep' : collision['timestep'], 'positive' : False}
        ]
    else:
        return [
            {'agent' : collision['a1'], 'loc' : [ collision['loc'][0], collision['loc'][1] ], 'timestep' : collision['timestep'], 'positive' : False},
            {'agent' : collision['a2'], 'loc' : [ collision['loc'][1], collision['loc'][0] ], 'timestep' : collision['timestep'], 'positive' : False}
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
    agent = 'a1'

    if random.randint(0, 1):
        agent = 'a1'
    else:
        agent = 'a2'

    if(len(collision['loc']) == 1):
        return [ 
            {'agent' : collision[agent], 'loc' : collision['loc'], 'timestep' : collision['timestep'], 'positive' : False},
            {'agent' : collision[agent], 'loc' : collision['loc'], 'timestep' : collision['timestep'], 'positive' : True}
        ]
    else:
        return [
            {'agent' : collision[agent], 'loc' : [ collision['loc'][0], collision['loc'][1] ], 'timestep' : collision['timestep'], 'positive' : False},
            {'agent' : collision[agent], 'loc' : [ collision['loc'][0], collision['loc'][1] ], 'timestep' : collision['timestep'], 'positive' : True}
        ]


#
# Please insert this function into "cbs.py" before "class MA_CBSSolver"
# is defined.
#
def disjoint_paths_violate_constraint(constraint, paths):
    ''' Determine if any paths violate given constraint '''
    #assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        next = get_location(paths[i], constraint['timestep'] + 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == curr or constraint['loc'][1] == next \
                    or constraint['loc'] == [next, curr]:
                rst.append(i)
    return rst


class MA_CBSSolver(object):
    """The high-level search of MACBS."""

    def __init__(self, my_map, starts, goals, mergeBound=1, depth=0):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.mergeBound = mergeBound

        self.depth = depth

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.goal_node = None

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def display_node(self, node, expanded):
        pass
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
                    'collisions': [],
                    'collisionMatrix': [],
                    'metaAgentGroups': []
                    }

        for path in parent['paths']:
            new_node['paths'].append(path)

        for constraint in parent['constraints']:
            new_node['constraints'].append(constraint)

        new_node['constraints'] += [new_constraint]

        for MA_G in parent['metaAgentGroups']:
            new_node['metaAgentGroups'].append(MA_G)

        return new_node

    def lower_level_solver(self, MA, node, disjoint, merging):
        """ Calls MA_CBS on a meta Agent to solve
            all paths in that meta agent
        """
        metaAgentGroups = node['metaAgentGroups']
        constraints = node['constraints']

        # Determine the start and goal locations of the agents in the Meta Agent
        MA_starts = []
        MA_goals = []
        LL_agent_list = [] # List to keep track of which agent numbers in the lower level correspond to agent numbers in the upper level
        for agent in MA:
            MA_starts.insert(0, self.starts[agent]) 
            MA_goals.insert(0, self.goals[agent])
            LL_agent_list.insert(0, agent)

        # Determine which constraints apply to this MA for the lower level solver to use
        MA_constraints = []
        for constraint in constraints:
            if constraint['agent'] in MA:
                # Convert the upper agent number to the lower agent number
                LL_agent_num = LL_agent_list.index( constraint['agent'] )

                constraint['agent'] = LL_agent_num 

                # Now add the updated agent number into a new constraint
                MA_constraints.insert(0, constraint)

        # Calculate the new merge bound if this was called from a merge call
        if(merging):
            LL_mergeBound = self.mergeBound + 1
        else:
            LL_mergeBound = self.mergeBound

        # Use MA-CBS with an increased bound on the merge conflicts to solve the meta agent's paths
        LowLevel_MA_CBS = MA_CBSSolver(self.my_map, MA_starts, MA_goals, LL_mergeBound, self.depth + 1)
        LL_paths = LowLevel_MA_CBS.find_solution(disjoint, MA_constraints)

        # Convert the constraint LL agent numbers to upper constraint numbers
        for constraint in MA_constraints:
            H_agent_num = LL_agent_list[ constraint['agent'] ]

            constraint['agent'] = H_agent_num

        # Put new paths into meta agent, else prune the node if no path was found
        i = len(LL_paths)-1
        for agent in MA:
            if LL_paths[i] is not None:
                node['paths'][agent] = LL_paths[i]
            else:
                return None
            i-=1

        return node

    def find_solution(self, disjoint=True, init_constraints=[]):
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
                'constraints': init_constraints,
                'paths': [],
                'collisions': [], # Store which agents collide
                'collisionMatrix': [], # Store the number of collisions between meta agents
                'metaAgentGroups': []} # Store individual agents that are a part of a meta agent

        for i in range(self.num_of_agents):  # Find initial path for each agent -- for MACBS as low level, need to keep track which agents these are for, constraints dont apply to the same agents anymore
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        # Set up each agent as a meta agent
        root['metaAgentGroups'] = [[agent] for agent in range(self.num_of_agents)]

        # Calculate the cost, collisions, and collision matrix for the root
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'], root['metaAgentGroups'])
        root['collisionMatrix'] = calc_collision_mat(root['paths'], root['metaAgentGroups'])

        self.push_node(root)

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while(len(self.open_list) > 0):
            next_node = self.pop_node()

            # See if this is a goal node
            if len(next_node['collisions']) == 0:
                self.goal_node = next_node
                break

            # Pick a collision from the collision list to solve
            collision = next_node['collisions'][-1]

            # These are the low level agents that have collided, they will be associated with a meta agent
            a1  = collision['a1']
            a2  = collision['a2']
            MA1 = which_meta_agent( a1, next_node['metaAgentGroups'] )
            MA2 = which_meta_agent( a2, next_node['metaAgentGroups'] )

            #**************** Detect if merging should occur ****************#
            if(should_merge(MA1, MA2, next_node['collisionMatrix'], self.mergeBound)):

                # Merge the two agents into a meta agent
                [next_node['metaAgentGroups'], next_node['constraints']] = merge(MA1, MA2, next_node['metaAgentGroups'], next_node['constraints'])

                # Get all the agents in the new meta agent
                MA_num = which_meta_agent(a1, next_node['metaAgentGroups'])
                MA = next_node['metaAgentGroups'][MA_num]

                # Find the new paths for all agents in the meta agent after merge
                LL_paths = self.lower_level_solver(MA, next_node['metaAgentGroups'], next_node['constraints'], disjoint, True)

                # Put the new paths into correct agents paths
                i = len(LL_paths)-1
                for agent in MA:
                    next_node['paths'][agent] = LL_paths[i]
                    i-=1

                # Update this node's information based on new path(s)
                next_node['collisions'] = detect_collisions(next_node['paths'], next_node['metaAgentGroups'])
                next_node['cost'] = get_sum_of_cost(next_node['paths'])
                next_node['collisionMatrix'] = calc_collision_mat(next_node['paths'], next_node['metaAgentGroups'])

                # Put node back into open list
                self.push_node(next_node)

                # Continue to the next best node in the open list
                continue

                #****************        END OF MERGING        ****************#

            constraints = []

            # Determine what splitting to use
            constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)

            for constraint in constraints:
                # Create a new node with an additional new constraint
                new_node = self.create_new_node(next_node, constraint)

                # Meta-Agent who's path is contrained needs to get a new path
                MA_constrained = [ which_meta_agent(constraint['agent'], new_node['metaAgentGroups']) ]

                # For disjoint splitting calculate new paths for all agents whos path violates the new positive constraint
                if disjoint and constraint['positive']:
                    MA_constrained += disjoint_paths_violate_constraint(constraint, new_node['paths'])
 
                # This is not the merging of two meta agents
                MERGE = False
 
                # Find the new paths for all agents in the meta agent after merge
                new_node = self.lower_level_solver(MA_constrained, new_node, disjoint, MERGE)

                # Paths could not be found based on contraints, prune
                if(new_node is None):
                    continue

                # Update new node's information based on new path(s)
                new_node['collisions'] = detect_collisions(new_node['paths'], new_node['metaAgentGroups'])
                new_node['cost'] = get_sum_of_cost(new_node['paths'])
                new_node['collisionMatrix'] = calc_collision_mat(new_node['paths'], root['metaAgentGroups'])
                
                # Push the new node to the open list
                self.push_node(new_node)

        if self.goal_node is not None:
            #self.print_results(self.goal_node)
            return self.goal_node['paths']
        else:
            return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

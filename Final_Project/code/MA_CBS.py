import time as timer
import heapq
import random
import copy
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost

TIME_LIMIT = 20

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

        # Store the collision as a collision of the agents
        agent_dict = {'a1': agent, 'a2': next_agent}

        # Find if there is a collision
        path_collision = detect_first_collision(path, paths[next_agent])

        # Ensure that a collision is only between differing Meta-Agents
        if (path_collision is not None) and (MA1 != MA2):            
            agent_dict['timestep'] = path_collision[0]
            agent_dict['loc'] =  path_collision[1]
            collisions.append(agent_dict)
        # Collisions should not happen within a Meta agent, otherwise this solver is not complete and optimal
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
    # Finds which meta agent group an agent belongs to, else returns None

    for meta_agent in range( len(metaAgentGroups) ):
        if agent in metaAgentGroups[meta_agent]:
            return meta_agent

    # Agent is not in a meta agent, this should be impossible. Throw error
    return None


def combine_constraints(MA1, MA2, MA_groups, constraints):
    ##############################
    # Combine the constraints between the two Meta agent groups
    #   Determine if the constraints are internal constraints (beteween these two meta agents), 
    #   or external constraints (between one of these agents and another agent)

    internal_constraints = []

    # Combine the constraints that apply to agents in MA2 to MA1
    for constraint in constraints:

        # Does the constraint apply to the mergeing metaagents?
        if constraint['metaAgent'] != MA1 and constraint['metaAgent'] != MA2:
            continue
        
        constraint_removed = False

        # Find and remove internal constraints based on which agents caused the constraint
        for agent in constraint['otherAgents']:
            MA_of_collision = which_meta_agent(agent, MA_groups)

            # If the collided meta agent was not found, this is an external constraint from a higher up recursive call
            if MA_of_collision is None:
                break

            if MA_of_collision == MA2 or MA_of_collision == MA1:
                # This is an internal constraint, must remove it
                internal_constraints.insert(0, constraint)
                constraint_removed = True
                break

        if constraint_removed:
            continue

        # If the external constraint applies to MA2, make it now apply to MA1
        if constraint['metaAgent'] == MA2:
            constraint['metaAgent'] = MA1

    for constraint in internal_constraints:
        constraints.remove(constraint)

    return constraints


def merge(minMA, maxMA, MA_groups):
    ###############################
    # Merge two meta agents into one meta agent

    # First create copy of MA_groups
    merged_groups = copy.deepcopy(MA_groups)

    # Combine the two meta agents into one
    merged_groups[minMA] += merged_groups[maxMA]

    # Clear the second meta agent group
    merged_groups.pop(maxMA)

    # Return the updated group and contraints
    return merged_groups


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


def update_constraints(node_constraints, MA1, MA2):
    # Take all contraints that apply to MA2 and apply them to MA1
    for constraint in node_constraints:
        if constraint['metaAgent'] == MA2:
            constraint['metaAgent'] = MA1

    return node_constraints


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
            {
                'agents' : [collision['a1']],
                'metaAgent' : [],
                'loc' : collision['loc'],
                'timestep' : collision['timestep'],
                'positive' : False,
                'otherAgents' : [collision['a2']]
            },
            {   'agents' : [collision['a2']],
                'metaAgent' : [],
                'loc' : collision['loc'],
                'timestep' : collision['timestep'],
                'positive' : False,
                'otherAgents' : [collision['a1']]
            }
        ]
    else:
        return [
            {'agents' : [collision['a1']], 'metaAgent' : [], 'loc' : [ collision['loc'][0], collision['loc'][1] ], 'timestep' : collision['timestep'], 'positive' : False, 'otherAgents' : [collision['a2']]},
            {'agents' : [collision['a2']], 'metaAgent' : [], 'loc' : [ collision['loc'][1], collision['loc'][0] ], 'timestep' : collision['timestep'], 'positive' : False, 'otherAgents' : [collision['a1']]}
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
            {'agents' : [collision[agent]], 'metaAgent' : [], 'loc' : collision['loc'], 'timestep' : collision['timestep'], 'positive' : False, 'otherAgents' : [collision['a2']]},
            {'agents' : [collision[agent]], 'metaAgent' : [], 'loc' : collision['loc'], 'timestep' : collision['timestep'], 'positive' : True, 'otherAgents' : [collision['a1']]}
        ]
    else:
        return [
            {'agents' : [collision[agent]], 'metaAgent' : [], 'loc' : [ collision['loc'][0], collision['loc'][1] ], 'timestep' : collision['timestep'], 'positive' : False, 'otherAgents' : [collision['a2']]},
            {'agents' : [collision[agent]], 'metaAgent' : [], 'loc' : [ collision['loc'][0], collision['loc'][1] ], 'timestep' : collision['timestep'], 'positive' : True, 'otherAgents' : [collision['a1']]}
        ]


#################################################
#
# NODE SANITY CHECK FUNCTIONS
#
#################################################
def disjoint_paths_violate_constraint(constraint, paths):
    ''' Determine if any paths violate given constraint '''
    assert constraint['positive'] is True
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


def ensure_dif_paths(node):
    num_paths = len(node['paths'])
    j = 0
    while j < num_paths:
        k = j+1
        while k < num_paths:
            if node['paths'][j][0] == node['paths'][k][0] or node['paths'][j][-1] == node['paths'][k][-1]:
                raise Exception # Check how the paths are the same start and goal locations here
            k+=1
        j+=1


def ensure_no_duplicate_agents(node):
    # Detect that there is the correct number of agents in the meta agent list
    list_of_agents = []
    for MA in node['metaAgentGroups']:
        for agent in MA:
            list_of_agents.append(agent)

    # Compare the sizes of the total agents in a meta agent to the number of agents for the solver
    if len(list_of_agents) != len(node['paths']):
        raise Exception


def ensure_no_collisions_in_MA(node):
    detect_collisions(node['paths'], node['metaAgentGroups'])

#################################################
#
# SOLVER CLASS
#
#################################################
class MA_CBSSolver(object):
    """The high-level search of MACBS."""

    def __init__(self, my_map, starts, goals, mergeBound=1, depth=0, file_start_time=0, merge_bound_incr=1):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.start_time = file_start_time
        self.merge_bound_incr = merge_bound_incr

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

        new_node['paths'] = copy.deepcopy(parent['paths'])

        new_node['constraints'] = copy.deepcopy(parent['constraints'])

        new_node['constraints'] += [new_constraint]

        new_node['metaAgentGroups'] = copy.deepcopy(parent['metaAgentGroups'])

        return new_node

    def ensure_path_ends_match_goals(self, node):
        for agent in range(self.num_of_agents):
            assert(node['paths'][agent][0] == self.starts[agent])
            assert(node['paths'][agent][-1] == self.goals[agent])

    def lower_level_solver(self, MA, node, disjoint, merging, merge_bound_incr):
        """ Calls MA_CBS on a meta Agent to solve
            all paths in that meta agent
        """
        metaAgentGroups = node['metaAgentGroups']
        constraints = node['constraints']

        ensure_dif_paths(node)
        ensure_no_duplicate_agents(node)

        # Determine the start and goal locations of the agents in the Meta Agent
        MA_starts = []
        MA_goals = []
        LL_agent_list = [] # List to keep track of which agent numbers in the lower level correspond to agent numbers in the upper level
        
        # Get all agents in the MA list that need to have paths recalculated
        for meta_agent in MA:
            agent_list = metaAgentGroups[meta_agent]
            for agent in agent_list:
                MA_starts.append(self.starts[agent]) 
                MA_goals.append(self.goals[agent])
                LL_agent_list.append(agent)

        ensure_dif_paths(node)
        ensure_no_duplicate_agents(node)

        # Determine which constraints apply to these MA for the lower level solver to use
        MA_constraints = []
        for constraint in constraints:
            # Copy constraint to a new constraint container
            constraint_copy = copy.deepcopy(constraint)

            if constraint_copy['metaAgent'] in MA:
                
                # Convert the upper solver agent numbers to the lower solver agent numbers for the constraint
                LL_constrained_agents = []
                for agent in constraint_copy['agents']:
                    LL_constrained_agents.append( LL_agent_list.index( agent ) )

                constraint_copy['agents'] = LL_constrained_agents

                # Now add the updated constraint to those passed to the low level
                MA_constraints.append(constraint_copy)

        ensure_dif_paths(node)
        ensure_no_duplicate_agents(node)

        # Calculate the new merge bound if this was called from a merge agents call
        if(merging):
            if merge_bound_incr == 1:
                LL_mergeBound = self.mergeBound + 1
            elif merge_bound_incr == 2:
                LL_mergeBound = self.mergeBound + 2
            elif merge_bound_incr == 3:
                LL_mergeBound = self.mergeBound + 3
            elif merge_bound_incr == 4:
                LL_mergeBound = self.mergeBound + 4
            else:
                LL_mergeBound = self.mergeBound + 5
        else:
            LL_mergeBound = self.mergeBound

        # Use MA-CBS with an increased bound on the merge conflicts to solve the meta agent's paths
        LowLevel_MA_CBS = MA_CBSSolver(self.my_map, MA_starts, MA_goals, LL_mergeBound, self.depth + 1, self.start_time, merge_bound_incr)
        LL_paths = LowLevel_MA_CBS.find_solution(disjoint, MA_constraints)

        ensure_dif_paths(node)
        ensure_no_duplicate_agents(node)

        # Put new paths into meta agent, else prune the node if no path was found
        if LL_paths is not None:
            # First sanity check the number of paths
            assert(len(LL_paths) == len(LL_agent_list))

            # Iterate through each LL agent and update the higher level agent path
            for LL_agent in range( len(LL_paths) ):
                H_agent = LL_agent_list[LL_agent]
                if LL_paths[LL_agent] is not None:
                    node['paths'][H_agent] = copy.deepcopy(LL_paths[LL_agent])
                else:
                    return None

            # Ensure all the new paths are different start and end locations    
            ensure_dif_paths(node)
            ensure_no_duplicate_agents(node)
            self.ensure_path_ends_match_goals(node)
            ensure_no_collisions_in_MA(node)
        else:
            # No paths found for node, prune
            node = None

        if node is not None:
            ensure_dif_paths(node)
            ensure_no_duplicate_agents(node)

        return node

    def find_solution(self, disjoint=True, init_constraints=[]):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

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

        # Set up each agent as a meta agent
        root['metaAgentGroups'] = [[agent] for agent in range(self.num_of_agents)]

        # Ensure any input constraints have correct meta agent value
        for constraint in root['constraints']:
            constraint['metaAgent'] = which_meta_agent(constraint['agents'][0], root['metaAgentGroups'])

        for i in range(self.num_of_agents):  # Find initial path for each agent -- for MACBS as low level, need to keep track which agents these are for, constraints dont apply to the same agents anymore
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                return None
                raise BaseException('No solutions')
            root['paths'].append(path)

        # Calculate the cost, collisions, and collision matrix for the root
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'], root['metaAgentGroups'])
        root['collisionMatrix'] = calc_collision_mat(root['paths'], root['metaAgentGroups'])

        ensure_dif_paths(root)
        ensure_no_duplicate_agents(root)

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

            if timer.time() - self.start_time > TIME_LIMIT:
                return None

            # Pick a collision from the collision list to solve
            collision = next_node['collisions'][-1]

            # These are the low level agents that have collided, they will be associated with a meta agent
            a1  = collision['a1']
            a2  = collision['a2']
            MA1 = which_meta_agent( a1, next_node['metaAgentGroups'] )
            MA2 = which_meta_agent( a2, next_node['metaAgentGroups'] )

            #**************** Detect if merging should occur ****************#
            if(should_merge(MA1, MA2, next_node['collisionMatrix'], self.mergeBound)):
                
                # Combine the agents into the min MA group
                minMA = min(MA1, MA2)
                maxMA = max(MA1, MA2)

                # Merge the constraints of the two agents
                node_constraints = combine_constraints(minMA, maxMA, next_node['metaAgentGroups'], next_node['constraints'])

                # Merge the two meta agent agents into one meta agent
                next_node['metaAgentGroups'] = merge(minMA, maxMA, next_node['metaAgentGroups'])

                # With the new meta agent groups, need to update the contraints to apply to the correct meta agent
                node_constraints = update_constraints(node_constraints, minMA, maxMA)

                # This is a lower level solver for a merge case
                MERGE = True

                # Find the new paths for all agents in the meta agent after merge
                next_node = self.lower_level_solver([minMA], next_node, disjoint, MERGE, self.merge_bound_incr)

                #print("\t"*self.depth + "Returning from merge")

                # If the lower level solver could not find a solution for the merged node, it gets pruned
                if next_node == None:
                    continue

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

                # Add the meta agent to the constraint
                constraint['metaAgent'] = which_meta_agent(constraint['agents'][0], new_node['metaAgentGroups'])

                # Meta-Agent who's path is contrained needs to get a new path
                MA_constrained = [ constraint['metaAgent'] ]

                # For disjoint splitting calculate new paths for all agents whos path violates the new positive constraint
                if disjoint and constraint['positive']:
                    MA_constrained += disjoint_paths_violate_constraint(constraint, new_node['paths']) # Ensure the meta agent of the paths is added to MA_constrained
 
                # This is not the merging of two meta agents
                MERGE = False
 
                # Find the new paths for all agents in the meta agent after merge
                new_node = self.lower_level_solver(MA_constrained, new_node, disjoint, MERGE, self.merge_bound_incr)

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

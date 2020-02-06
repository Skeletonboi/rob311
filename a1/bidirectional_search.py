from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem

def bidirectional_search(problem):
    """
        Implement a bidirectional search algorithm that takes instances of SimpleSearchProblem (or its derived
        classes) and provides a valid and optimal path from the initial state to the goal state.

        :param problem: instance of SimpleSearchProblem
        :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
                 num_nodes_expanded: number of nodes expanded by your search
                 max_frontier_size: maximum frontier size during search
        """
    frontier_s = []  # Queue: Append to end, Extract from 0th index
    frontier_g = []  # Queue: Append to end, Extract from 0th index
    explored_s = {}
    explored_g = {}
    init_s = Node(None, problem.init_state, None, 0)
    init_g = Node(None, problem.goal_states[0],None,0)
    if type(init_s.state) is list:
        init_s.state = init_s.state[0]
    # Check if we've already reached desired goal state
    if init_state == problem.goal_states[0]:
        return [init.state], 1, len(problem.neighbours[init.state])
    # Find starting node's neighbours
    init_s_neighbours = problem.neighbours[init_s.state]
    init_g_neighbours = problem.neighbours[init_g.state]
    # Add starting neighbours to queues
    for i in init_s_neighbours:
        temp = Node(init_s, i, (init_s.state, i), init_s.path_cost + 1)
        frontier_s.append(temp)
    for i in init_g_neighbours:
        temp = Node(init_g, i, (init_g.state, i), init_g.path_cost + 1)
        frontier_g.append(temp)
    while len(frontier_s) != 0 and len(frontier_g) != 0:
        # Source Node BFS
        node = frontier_s.pop(0)
        explored_s[node.state] = node
        print("S explored:", node.state)
        print('possible actions: ', problem.get_actions(node.state))
        intersect = {}
        for i in problem.get_actions(node.state):
            child = problem.get_child_node(node, i)
            if (child not in frontier_s) or (child.state not in explored_s):
                # if problem.goal_test(child.state):
                #     path = problem.trace_path(child, init_s.state)
                #     return path, len(explored), len(frontier_s) + len(explored)
                if child.state in explored_g:
                    explored_s[child.state] = child
                    intersect[child.state] = explored_g[child.state]
                frontier_s.append(child)
        if len(intersect) != 0:
            # Intersection has been found
            # Extract min-path-cost node from intersecting nodes
            key_min = min(intersect.keys(), key=(lambda k: intersect[k].path_cost))
            child = intersect[key_min]
            path_s = problem.trace_path(explored_s[child.state], init_s.state)
            path_g = problem.trace_path(explored_g[child.state], init_g.state)
            print('S_DFS')
            print('path_s',path_s)
            print('path_g', path_g)
            return path_s + path_g[::-1][1:], 0 ,0

        # Goal Node BFS
        node = frontier_g.pop(0)
        explored_g[node.state] = node
        print("G explored:", node.state)
        for i in problem.get_actions(node.state):
            child = problem.get_child_node(node, i)
            if (child not in frontier_g):
            # if (child not in frontier_g) or (child.state not in explored_g):
                # if (child.state == init_s.state):
                #     path = problem.trace_path(child,init_g.state)
                #     return path, len(explored), len(frontier_g) + len(explored)
                # if child.state in explored_s:
                #     path_s = problem.trace_path(explored_s[child.state], init_s.state)
                #     path_g = problem.trace_path(child, init_g.state)
                #     print('G_DFS')
                #     print('path_s', path_s)
                #     print('path_g', path_g)
                #     return path_s+path_g, 0, 0
                frontier_g.append(child)


    max_frontier_size = 0
    num_nodes_expanded = 0
    path = []
    return path, num_nodes_expanded, max_frontier_size


if __name__ == '__main__':
    # Simple example
    goal_states = [0]
    init_state = 9
    V = np.arange(0, 10)
    E = np.array([[0, 1],
                  [1, 2],
                  [2, 3],
                  [3, 4],
                  [4, 5],
                  [5, 6],
                  [6, 7],
                  [7, 8],
                  [8, 9],
                  [0, 6],
                  [1, 7],
                  [2, 5],
                  [9, 4]])
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = bidirectional_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)

    # Use stanford_large_network_facebook_combined.txt to make your own test instances
    E = np.loadtxt('stanford_large_network_facebook_combined.txt', dtype=int)
    V = np.unique(E)
    goal_states = [349]
    init_state = [0]
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = bidirectional_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)

    # Be sure to compare with breadth_first_search!
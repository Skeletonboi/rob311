from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem

def breadth_first_search(problem):
    """
    Implement a simple breadth-first search algorithm that takes instances of SimpleSearchProblem (or its derived
    classes) and provides a valid and optimal path from the initial state to the goal state. Useful for testing your
    bidirectional and A* search algorithms.

    :param problem: instance of SimpleSearchProblem
    :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
             num_nodes_expanded: number of nodes expanded by your search
             max_frontier_size: maximum frontier size during search
    """
    frontier = [] # Queue: Append to end, Extract from 0th index
    explored = []
    init = Node(None,problem.init_state,None,0)
    if type(init.state) is list:
        init.state = init.state[0]
    init_neighbours = problem.neighbours[init.state]

    if init_state == problem.goal_states[0]:
        return [init.state], 1 , len(problem.neighbours[init.state])
    for i in init_neighbours:
        temp = Node(init,i,(init.state,i),init.path_cost+1)
        frontier.append(temp)
    while len(frontier) != 0:
        node = frontier.pop(0)
        # print(node.state)
        explored.append(node)
        for i in problem.get_actions(node.state):
            child = problem.get_child_node(node, i)
            if child not in (frontier or explored):
                if problem.goal_test(child.state):
                    path = problem.trace_path(child,init.state)
                    return path, len(explored), len(frontier)+len(explored)
                frontier.append(child)



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
    path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)

    # Use stanford_large_network_facebook_combined.txt to make your own test instances
    E = np.loadtxt('stanford_large_network_facebook_combined.txt', dtype=int)
    V = np.unique(E)
    goal_states = [349]
    init_state = [0]
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)

    E_twitter = np.load('twitter_edges_project_01.npy')
    V_twitter = np.unique(E_twitter)
    problem = GraphSearchProblem([59999], 0, V_twitter, E_twitter)
    path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)
import sys
import puzz
import pdqpq


MAX_SEARCH_ITERS = 100000
GOAL_STATE = puzz.EightPuzzleBoard("012345678")


def solve_puzzle(start_state, strategy):
    """Perform a search to find a solution to a puzzle.
    
    Args:
        start_state: an EightPuzzleBoard object indicating the start state for the search
        flavor: a string indicating which type of search to run.  Can be one of the following:
            'bfs' - breadth-first search
            'ucost' - uniform-cost search
            'greedy-h1' - Greedy best-first search using a misplaced tile count heuristic
            'greedy-h2' - Greedy best-first search using a Manhattan distance heuristic
            'greedy-h3' - Greedy best-first search using a weighted Manhattan distance heuristic
            'astar-h1' - A* search using a misplaced tile count heuristic
            'astar-h2' - A* search using a Manhattan distance heuristic
            'astar-h3' - A* search using a weighted Manhattan distance heuristic
    
    Returns: 
        A dictionary containing describing the search performed, containing the following entries:
            'path' - a list of 2-tuples representing the path from the start state to the goal state 
                (both should be included), with each entry being a (str, EightPuzzleBoard) pair 
                indicating the move and resulting state for each action.  Omitted if the search 
                fails.
            'path_cost' - the total cost of the path, taking into account the costs associated 
                with each state transition.  Omitted if the search fails.
            'frontier_count' - the number of unique states added to the search frontier at any
                point during the search.
            'expanded_count' - the number of unique states removed from the frontier and expanded 
                (successors generated).
    """

    results = {
        'frontier_count': 0,
        'expanded_count': 0,
    } 

    #bfs
    if strategy == 'bfs':
        results = bfs(start_state)
    #ucost
    elif strategy == 'ucost':
        results = ucost(start_state)
    #greedy
    elif strategy.startswith('greedy'):
        results = greedy(start_state, strategy)
    #astar
    elif strategy.startswith('astar'):
        results = astar(start_state, strategy)
    return results


#bfs
def bfs(start_state):
    results = {
        'frontier_count': 0,
        'expanded_count': 0,
    }
    #check if start = goal
    if start_state.__eq__(GOAL_STATE):
        return results
    #create frontier and explored and parents
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state)
    results['frontier_count'] += 1
    explored = set()
    parents = {}
    directions = {}
    path_cost = 0
    while not frontier.empty():
        node = frontier.pop()
        results['expanded_count'] += 1
        explored.add(node)
        successors = node.successors()
        for n in successors:
            if (successors[n] not in frontier) and (successors[n] not in explored):
                if (successors[n]).__eq__(GOAL_STATE):
                    path = [(n, successors[n])]
                    current = node
                    path_cost += (int(successors[n].__str__()[(current.__str__().index('0'))])) ** 2
                    results['path_cost'] = path_cost
                    while parents.get(current):
                        path = [(directions[current], current)] + path
                        # results['path_cost'] += (int) current[parents[current].index('0')]
                        results['path_cost'] += (int(current.__str__()[(parents[current].__str__().index('0'))])) ** 2
                        current = parents[current]
                    path = [('start', current)] + path
                    results['path'] = path
                    return results
                else:
                    frontier.add(successors[n])
                    results['frontier_count'] += 1
                    parents[successors[n]] = node
                    directions[successors[n]] = n
    return results

#ucost
def ucost(start_state):
    results = {
        'frontier_count': 0,
        'expanded_count': 0,
    }
    #check if start = goal
    if start_state.__eq__(GOAL_STATE):
        return results
    #create frontier and explored and parents
    frontier = pdqpq.PriorityQueue()
    costs = pdqpq.PriorityQueue()
    frontier.add(start_state, 0)
    costs.add(start_state, 0)
    results['frontier_count'] += 1
    explored = set()
    parents = {}
    directions = {}
    path_cost = 0
    while not frontier.empty():
        node = frontier.pop()
        cum_cost = costs.get(node)
        if node.__eq__(GOAL_STATE):
            path = []
            current = node
            path_cost += cum_cost
            results['path_cost'] = path_cost
            while parents.get(current):
                path = [(directions[current], current)] + path
                current = parents[current]
            path = [('start', current)] + path
            results['path'] = path
            return results
        explored.add(node)
        results['expanded_count'] += 1
        successors = node.successors()
        for n in successors:
            cost = (int(successors[n].__str__()[(node.__str__().index('0'))])) ** 2 
            if (successors[n] not in frontier) and (successors[n] not in explored):
                frontier.add(successors[n], cum_cost + cost)
                costs.add(successors[n], cum_cost + cost)
                parents[successors[n]] = node
                directions[successors[n]] = n
                results['frontier_count'] += 1
            elif (successors[n] in frontier) and (frontier.get(successors[n]) > (cum_cost + cost)):
                frontier.add(successors[n], cum_cost + cost)
                costs.add(successors[n], cum_cost + cost)
                parents[successors[n]] = node
                directions[successors[n]] = n
    return results

#greedy
def greedy(start_state, strategy):
    results = {
        'frontier_count': 0,
        'expanded_count': 0,
    }
    #check if start = goal
    if start_state.__eq__(GOAL_STATE):
        return results
    #create frontier and explored and parents
    frontier = pdqpq.PriorityQueue()
    costs = pdqpq.PriorityQueue()
    if strategy.endswith('h1'):
        frontier.add(start_state, h1(start_state))
        costs.add(start_state, 0)
    elif strategy.endswith('h2'):
        frontier.add(start_state, h2(start_state))
        costs.add(start_state, 0)
    elif strategy.endswith('h3'):
        frontier.add(start_state, h3(start_state))
        costs.add(start_state, 0)
    results['frontier_count'] += 1
    explored = set()
    parents = {}
    directions = {}
    path_cost = 0
    while not frontier.empty():
        node = frontier.pop()
        cum_cost = costs.get(node)
        if (node.__eq__(GOAL_STATE)):
            path = []
            current = node
            path_cost = cum_cost
            results['path_cost'] = path_cost
            while parents.get(current):
                path = [(directions[current], current)] + path
                current = parents[current]
            path = [('start', current)] + path
            results['path'] = path
            return results
        explored.add(node)
        results['expanded_count'] += 1
        successors = node.successors()
        for n in successors:
            hcost = 0
            if strategy.endswith('h1'):
                hcost = h1(successors[n])
            elif strategy.endswith('h2'):
                hcost = h2(successors[n])
            elif strategy.endswith('h3'):
                hcost = h3(successors[n])
            cost = (int(successors[n].__str__()[(node.__str__().index('0'))])) ** 2 
            if (successors[n] not in frontier) and (successors[n] not in explored):
                frontier.add(successors[n], hcost)
                costs.add(successors[n], cum_cost + cost)
                parents[successors[n]] = node
                directions[successors[n]] = n
                results['frontier_count'] += 1
            elif (successors[n] in frontier) and (frontier.get(successors[n]) > cost):
                frontier.add(successors[n], hcost)
                costs.add(successors[n], cum_cost + cost)
                parents[successors[n]] = node
                directions[successors[n]] = n
    return results

#astar
def astar(start_state, strategy):
    results = {
        'frontier_count': 0,
        'expanded_count': 0,
    }
    #check if start = goal
    if start_state.__eq__(GOAL_STATE):
        return results
    #create frontier and explored and parents
    frontier = pdqpq.PriorityQueue()
    costs = pdqpq.PriorityQueue()
    frontier.add(start_state, 0)
    costs.add(start_state, 0)
    results['frontier_count'] += 1
    explored = set()
    parents = {}
    directions = {}
    path_cost = 0
    while not frontier.empty():
        node = frontier.pop()
        cum_cost = costs.get(node)
        if (node.__eq__(GOAL_STATE)):
            path = []
            current = node
            path_cost += cum_cost
            results['path_cost'] = path_cost
            while parents.get(current):
                path = [(directions[current], current)] + path
                current = parents[current]
            path = [('start', current)] + path
            results['path'] = path
            return results
        explored.add(node)
        results['expanded_count'] += 1
        successors = node.successors()
        for n in successors:
            hcost = 0
            if strategy.endswith('h1'):
                hcost = h1(successors[n])
            elif strategy.endswith('h2'):
                hcost = h2(successors[n])
            elif strategy.endswith('h3'):
                hcost = h3(successors[n])
            cost = (int(successors[n].__str__()[(node.__str__().index('0'))])) ** 2 
            if (successors[n] not in frontier) and (successors[n] not in explored):
                 frontier.add(successors[n], (cum_cost + hcost + cost))
                 costs.add(successors[n], cum_cost + cost)
                 parents[successors[n]] = node
                 directions[successors[n]] = n
                 results['frontier_count'] += 1
            elif (successors[n] in frontier) and (frontier.get(successors[n]) > (cum_cost + hcost + cost)):
                frontier.add(successors[n], cum_cost + hcost + cost)
                costs.add(successors[n], cum_cost + cost)
                parents[successors[n]] = node
                directions[successors[n]] = n
    return results

#HEURISTIC 1
def h1(state):
    num = 0
    for i in range(1,9):
        if i != int(state.__str__()[i]):
            num += 1
    return num

#HEURISTIC 2
def h2(state):
    distance = 0
    x, y = state.find('1')
    distance += (abs(x-1) + abs(y-2))
    x, y = state.find('2')
    distance += (abs(x-2) + abs(y-2))
    x, y = state.find('3')
    distance += (abs(x-0) + abs(y-1))
    x, y = state.find('4')
    distance += (abs(x-1) + abs(y-1))
    x, y = state.find('5')
    distance += (abs(x-2) + abs(y-1))
    x, y = state.find('6')
    distance += (abs(x-0) + abs(y-0))
    x, y = state.find('7')
    distance += (abs(x-1) + abs(y-0))
    x, y = state.find('8')
    distance += (abs(x-2) + abs(y-0))
    return distance

#HEURISTIC 3
def h3(state):
    distance = 0
    num = (1 ** 2)
    x, y = state.find('1')
    dist = (abs(x-1) + abs(y-2))
    distance += (dist * num)
    num = (2 ** 2)
    x, y = state.find('2')
    dist = (abs(x-2) + abs(y-2))
    distance += (dist * num)
    num = (3 ** 2)
    x, y = state.find('3')
    dist = (abs(x-0) + abs(y-1))
    distance += (dist * num)
    num = (4 ** 2)
    x, y = state.find('4')
    dist = (abs(x-1) + abs(y-1))
    distance += (dist * num)
    num = (5 ** 2)
    x, y = state.find('5')
    dist = (abs(x-2) + abs(y-1))
    distance += (dist * num)
    num = (6 ** 2)
    x, y = state.find('6')
    dist = (abs(x-0) + abs(y-0))
    distance += (dist * num)
    num = (7 ** 2)
    x, y = state.find('7')
    dist = (abs(x-1) + abs(y-0))
    distance += (dist * num)
    num = (8 ** 2)
    x, y = state.find('8')
    dist = (abs(x-2) + abs(y-0))
    distance += (dist * num)
    return distance

def print_summary(results):
    if 'path' in results:
        print("found solution of length {}, cost {}".format(len(results['path']), 
                                                            results['path_cost']))
        for move, state in results['path']:
            print("  {:5} {}".format(move, state))
    else:
        print("no solution found")
    print("{} states placed on frontier, {} states expanded".format(results['frontier_count'], 
                                                                    results['expanded_count']))


############################################

if __name__ == '__main__':

    start = puzz.EightPuzzleBoard(sys.argv[1])
    method = sys.argv[2]

    print("solving puzzle {} -> {}".format(start, GOAL_STATE))
    results = solve_puzzle(start, method)
    print_summary(results)
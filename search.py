"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import heapq
import util
import problems


from game import Directions
from problems import SearchProblem
from problems import MultiFoodSearchProblem
from util import manhattanDistance
from util import PriorityQueue
from util import Queue
from util import Stack



n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


def depthFirstSearch(problem:problems):
    '''
    return a path to the goal
    '''
    # TODO 17
    fringe = util.Stack()
    expanded = set()
    fringe.push((problem.getStartState(),[],0))
    
    while not fringe.isEmpty():
        curState, curMoves, curCost = fringe.pop()
        
        if(curState in expanded):
            continue
        
        expanded.add(curState)
        
        if problem.isGoalState(curState):
            return curMoves
        
        for state, direction, cost in problem.getSuccessors(curState):
            fringe.push((state, curMoves+[direction], curCost))
    return []
    


def breadthFirstSearch(problem:problems):
    '''
    return a path to the goal
    '''
    # TODO 18
    fringe = util.Queue()
    fringe.push((problem.getStartState(), []))
    expanded = set()
    
    while not fringe.isEmpty():
        curState, curMoves = fringe.pop()
        
        if curState in expanded:
            continue
        
        expanded.add(curState)
        
        if problem.isGoalState(curState):
            return curMoves
        
        for state, direction, cost in problem.getSuccessors(curState):
            fringe.push((state, curMoves + [direction]))
    
    return []


def uniformCostSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 19
    fringe = util.PriorityQueue()
    fringe.push((problem.getStartState(),[],0), 0)
    expanded = set()

    while not fringe.isEmpty():
        curState, curMoves, curCost = fringe.pop()
        
        if(curState in expanded):
            continue
            
        expanded.add(curState)
        
        if problem.isGoalState(curState):
            return curMoves
        
        for state, direction, cost in problem.getSuccessors(curState):
            fringe.push((state, curMoves+[direction], curCost + cost), curCost + cost)
    return []


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    if problem.isGoalState(state):
        return 0

    return float('inf')


def singleFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of single food search
    """
    # TODO 20
    goalState = problem.food.asList()[0]
    x1, y1 = state
    x2, y2 = goalState
    heuristic = manhattanDistance(state,goalState)
    return heuristic

def multiFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of multi-food search
    """
    # TODO 21
    position, foodGrid = state
    heuristic = 0
    for x in range(foodGrid.width):
        for y in range(foodGrid.height):
            if foodGrid[x][y]:
                dist = manhattanDistance(position, (x, y))
                heuristic = max(heuristic, dist)
    return heuristic


def aStarSearch(problem:problems, heuristic=multiFoodSearchHeuristic):
    '''
    return a path to the goal
    '''
    # TODO 22
    fringe = util.PriorityQueue()
    fringe.push( (problem.getStartState(), [], 0), heuristic(problem.getStartState(), problem) )
    expanded = []

    while not fringe.isEmpty():
        node, actions, curCost = fringe.pop()

        if(not node in expanded):
            expanded.append(node)

            if problem.isGoalState(node):
                return actions

            for child, direction, cost in problem.getSuccessors(node):
                g = curCost + cost
                fringe.push((child, actions+[direction], curCost + cost), g + heuristic(child, problem))

    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

import util
import signa
from pacman import GameState
from game import Actions
from game import Directions


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


class SingleFoodSearchProblem(SearchProblem):
    def __init__(self, startGameState):
        # TODO 1
        self.startGameState = startGameState
        self.walls = startGameState.getWalls()
        self.startState = startGameState.getPacmanPosition()
        self.food = startGameState.getFood()
        self.costFn = lambda x: 1

    def getStartState(self):
        # TODO 1
        return self.startState


    def isGoalState(self, state):
        # TODO 3
        if state in self.food.asList():
            return True
        else:
            return False

    def getSuccessors(self, state):
        # TODO 4
        successors = []
        
        for action in Actions._directions:
            nextState = (state[0] + Actions._directions[action][0], state[1] + Actions._directions[action][1])
            if not self.walls[nextState[0]][nextState[1]]:
                successors.append((nextState, action, 1))
            
        return successors


    def getCostOfActions(self, actions):
        # TODO 5
        cost = 0
        for action in actions:
            cost += self.costFn(action)
        return cost



class MultiFoodSearchProblem(SearchProblem):
    def __init__(self, startingGameState):
        # TODO 6
        self.startingGameState = startingGameState
        self.walls = startingGameState.getWalls()
        self.startState = (startingGameState.getPacmanPosition(), startingGameState.getFood())
        self._expanded = 0

    def getStartState(self):
        # TODO 7
        return self.startState

        
    def isGoalState(self, state):
        # TODO 8
        return state[1].count() == 0
  
    def getSuccessors(self, state):
        # TODO 9
        successors = []
        self._expanded += 1
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append( ( ((nextx, nexty), nextFood), direction, 1) )
        return successors


    def getCostOfActions(self, actions):
        # TODO 10
        x,y= self.getStartState()[0]
        cost = 0
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost

import random
import problems

from game import Agent
from game import Directions
from pacman import GameState
from search import bfs
from search import dfs
from search import ucs
from search import astar
from problems import MultiFoodSearchProblem
from problems import SingleFoodSearchProblem


class GoWestAgent(Agent):
    def getAction(self, state):
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP


class RandomAgent(Agent):
    def getAction(self, state):
        actions = state.getLegalPacmanActions()
        random.shuffle(actions)
        return actions[0]


class SearchAgent(Agent):
    def registerInitialState(self, state: GameState):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        # TODO 11
        problem = MultiFoodSearchProblem(state)
        self.actions = bfs(problem)
        print("Total number of steps taken: ", problem.getCostOfActions(self.actions))

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        # TODO 12
        if 'actionIndex' not in dir(self): 
            self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]    
        else:
            return Directions.STOP
        


class BFSFoodSearchAgent(SearchAgent):
    # TODO 13
    def registerInitialState(self, state):
        problem = MultiFoodSearchProblem(state)
        self.actions = bfs(problem)
        print("Total number of steps taken: ", problem.getCostOfActions(self.actions))
    


class DFSFoodSearchAgent(SearchAgent):
    # TODO 14
    def registerInitialState(self, state):
        problem = MultiFoodSearchProblem(state)
        self.actions = dfs(problem)
        print("Total number of steps taken: ", problem.getCostOfActions(self.actions))
    


class UCSFoodSearchAgent(SearchAgent):
    # TODO 15
    def registerInitialState(self, state):
        problem = MultiFoodSearchProblem(state)
        self.actions = ucs(problem)
        print("Total number of steps taken: ", problem.getCostOfActions(self.actions))
    


class AStarFoodSearchAgent(SearchAgent):
    # TODO 16
    def registerInitialState(self, state):
        problem = MultiFoodSearchProblem(state)
        self.actions = astar(problem)
        print("Total number of steps taken: ", problem.getCostOfActions(self.actions))
   


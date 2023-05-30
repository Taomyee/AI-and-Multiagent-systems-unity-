# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

from captureAgents import CaptureAgent
import distanceCalculator
import random, time, util, sys
from game import Directions
import game
from util import nearestPoint
from game import Actions
import copy
from util import PriorityQueue


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first='DummyAgent', second='DefensiveAgent'):
    """
    This function should return a list of two agents that will form the
    team, initialized using firstIndex and secondIndex as their agent
    index numbers.  isRed is True if the red team is being created, and
    will be False if the blue team is being created.

    As a potentially helpful development aid, this function can take
    additional string-valued keyword arguments ("first" and "second" are
    such arguments in the case of this function), which will come from
    the --redOpts and --blueOpts command-line arguments to capture.py.
    For the nightly contest, however, your team will be created without
    any extra arguments, so you should make sure that the default
    behavior is what you want for the nightly contest.
    """
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


class DummyAgent(CaptureAgent):
    """
    A Dummy agent to serve as an example of the necessary agent structure.
    You should look at baselineTeam.py for more details about how to
    create an agent as this is the bare minimum.
    """

    def registerInitialState(self, gameState):
        """
        This method handles the initial setup of the
        agent to populate useful fields (such as what team
        we're on).

        A distanceCalculator instance caches the maze distances
        between each pair of positions, so your agents can use:
        self.distance.getDistance(p1, p2)

        IMPORTANT: This method may run for at most 15 seconds.
        """

        '''
        Make sure you do not delete the following line. If you would like to
        use Manhattan distances instead of maze distances in order to save
        on initialization time, please take a look at
        CaptureAgent.registerInitialState in captureAgents.py.
        '''
        CaptureAgent.registerInitialState(self, gameState)

        '''
        Your initialization code goes here, if you need any.
        '''

    def chooseAction(self, gameState):
        """
        Picks among actions randomly.
        """
        actions = gameState.getLegalActions(self.index)

        '''
        You should change this in your own agent.
        '''

        return random.choice(actions)


#######################
#  defender   #
#######################

class DefensiveAgent(CaptureAgent):
    def getSuccessor(self, gameState, action):
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def chooseAction(self, gameState):
        actions = gameState.getLegalActions(self.index)
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        invaders = [a for a in enemies if a.isPacman]
        knowninvaders = [a for a in enemies if a.isPacman and a.getPosition() is not None]
        self.lastEatenFoodPosition = self.getPosOfLastEatenFood(gameState)

        if len(invaders) == 0 or gameState.getAgentPosition(self.index) == self.lastEatenFoodPosition or len(
                knowninvaders) > 0:
            self.lastEatenFoodPosition = None

        if len(invaders) < 1:
            if gameState.getAgentState(self.index).numCarrying < 3 and len(
                    self.getFood(gameState).asList()) != 0 and not (
                    self.distToGhost(gameState) is not None and self.distToGhost(gameState)[0] < 4 and
                    self.distToGhost(gameState)[1].scaredTimer < 2):
                search = Search(gameState, self, self.index, "searchfood")
                path = aStarSearch(search, gameState, self.heuristic)
                return path[0]
            else:
                search = Search(gameState, self, self.index, "backhome")
                if len(aStarSearch(search, gameState, self.heuristic)) == 0:
                    return 'Stop'
                else:
                    path = aStarSearch(search, gameState, self.heuristic)
                    return path[0]
        else:
            if len(knowninvaders) == 0 and self.lastEatenFoodPosition is not None and gameState.getAgentState(
                    self.index).scaredTimer == 0:
                search = Search(gameState, self, self.index, "searchlasteatenfood")
                path = aStarSearch(search, gameState, self.heuristic)
                return path[0]

            if len(knowninvaders) > 0 and gameState.getAgentState(self.index).scaredTimer == 0:
                search = Search(gameState, self, self.index, "searchinvader")
                path = aStarSearch(search, gameState, self.heuristic)
                return path[0]
        return random.choice(actions)

    def distToGhost(self, gameState):
        myPosition = gameState.getAgentState(self.index).getPosition()
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        ghosts = [a for a in enemies if not a.isPacman and a.getPosition() is not None]
        if len(ghosts) > 0:
            min_dist = float('inf')
            for a in ghosts:
                temp = self.getMazeDistance(myPosition, a.getPosition())
                if temp < min_dist:
                    min_dist = temp
                    ghostState = a
            return [min_dist, ghostState]
        else:
            return None

    def getBoundary(self, gameState):
        if self.red:
            i = (int)(gameState.data.layout.width / 2 - 1)
        else:
            i = (int)(gameState.data.layout.width / 2 + 1)
        boundaries = [(i, j) for j in range(gameState.data.layout.height)]
        validPositions = []
        for i in boundaries:
            if not gameState.hasWall(i[0], i[1]):
                validPositions.append(i)
        return validPositions

    def getPosOfLastEatenFood(self, gameState):
        if len(self.observationHistory) > 1:
            prevState = self.getPreviousObservation()
            prevFoodList = self.getFoodYouAreDefending(prevState).asList()
            currentFoodList = self.getFoodYouAreDefending(gameState).asList()
            if len(prevFoodList) != len(currentFoodList):
                for food in prevFoodList:
                    if food not in currentFoodList:
                        return food
        return None

    def heuristic(self, state, gameState):
        heuristic = 0
        nearest_ghost_distance = -1
        myPosition = gameState.getAgentState(self.index).getPosition()
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        ghosts = [a for a in enemies if not a.isPacman and a.getPosition() is not None]
        if len(ghosts) > 0:
            dists = [self.getMazeDistance(myPosition, a.getPosition()) for a in ghosts]
            nearest_ghost_distance = min(dists)
        if nearest_ghost_distance != -1:
            ghosts = [a for a in enemies if not a.isPacman and a.scaredTimer < 2 and a.getPosition() is not None]

            if ghosts and len(ghosts) > 0:
                ghost_positions = [ghost.getPosition() for ghost in ghosts]
                ghost_distances = [self.getMazeDistance(state, ghost_position) for ghost_position in ghost_positions]
                ghost_dist = min(ghost_distances)

                if ghost_dist < 2:
                    heuristic = 1.0 / (ghost_dist + 0.0001)

        return heuristic


def aStarSearch(search, gameState, heuristic):
    start_state = search.startState
    fringe = PriorityQueue()
    h = heuristic(start_state, gameState)
    g = 0
    f = g + h
    start_node = (start_state, [], g)
    fringe.push(start_node, f)
    explored = []
    while not fringe.isEmpty():
        current_node = fringe.pop()
        state = current_node[0]
        path = current_node[1]
        current_cost = current_node[2]
        if state not in explored:
            explored.append(state)
            if search.isGoalState(state):
                return path
            successors = search.getSuccessors(state)
            for successor in successors:
                current_path = list(path)
                successor_state = successor[0]
                move = successor[1]
                g = successor[2] + current_cost
                h = heuristic(successor_state, gameState)
                if successor_state not in explored:
                    current_path.append(move)
                    f = g + h
                    successor_node = (successor_state, current_path, g)
                    fringe.push(successor_node, f)
    return []


class Search:
    def __init__(self, gameState, agent, agentIndex, goal=""):
        self.startState = gameState.getAgentState(agentIndex).getPosition()
        self.food = agent.getFood(gameState)
        self.capsule = agent.getCapsules(gameState)
        self.walls = gameState.getWalls()
        self.goal = goal
        self.homeBoundary = agent.getBoundary(gameState)

        self.enemies = [gameState.getAgentState(agentIndex) for agentIndex in agent.getOpponents(gameState)]
        self.invaders = [a for a in self.enemies if a.isPacman and a.getPosition is not None]
        if len(self.invaders) > 0:
            self.invadersPosition = [invader.getPosition() for invader in self.invaders]
        else:
            self.invadersPosition = None

        if goal == "searchlasteatenfood":
            self.lasteatenfoodpos = agent.lastEatenFoodPosition

    def isGoalState(self, state):
        if self.goal == "searchfood":
            return state in self.food.asList()
        elif self.goal == "backhome":
            return state in self.homeBoundary or state in self.capsule
        elif self.goal == "searchinvader":
            return state in self.invadersPosition
        elif self.goal == "searchlasteatenfood":
            return state == self.lasteatenfoodpos

    def getSuccessors(self, state):
        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = 1
                successors.append((nextState, action, cost))
        return successors

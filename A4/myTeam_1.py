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
               first='OffensiveAgent', second='DefensiveAgent'):
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
        self.distancer.getDistance(p1, p2)

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


"""
Offensive agent
"""


class OffensiveAgent(CaptureAgent):

    def registerInitialState(self, gameState):
        """
        This method handles the initial setup of the
        agent to populate useful fields (such as what team
        we're on).

        A distanceCalculator instance caches the maze distances
        between each pair of positions, so your agents can use:
        self.distancer.getDistance(p1, p2)

        IMPORTANT: This method may run for at most 15 seconds.
        """

        '''
        Make sure you do not delete the following line. If you would like to
        use Manhattan distances instead of maze distances in order to save
        on initialization time, please take a look at
        CaptureAgent.registerInitialState in captureAgents.py.
        '''
        CaptureAgent.registerInitialState(self, gameState)

    def chooseAction(self, gameState):
        if gameState.getAgentState(self.index).numCarrying == 0 and len(self.getFood(gameState).asList()) == 0:
            return 'Stop'

        if len(self.getCapsules(gameState)) != 0:
            problem = Search(gameState, self, self.index, "searchcapsule")
            return aStarSearch(problem, gameState, self.heuristic)[0]

        if gameState.getAgentState(self.index).numCarrying < 1 and len(self.getFood(gameState).asList()) > 0:
            problem = Search(gameState, self, self.index, "searchfood")
            return aStarSearch(problem, gameState, self.heuristic)[0]

        if gameState.getAgentState(self.index).numCarrying < 1 and len(self.getFood(gameState).asList()) == 0:
            problem = Search(gameState, self, self.index, "searchfood")
            return aStarSearch(problem, gameState, self.heuristic)[0]

        if self.distToGhost(gameState) is not None and self.distToGhost(gameState)[0] < 6 and \
                self.distToGhost(gameState)[1].scaredTimer < 5:
            problem = Search(gameState, self, self.index, "escape")
            if len(aStarSearch(problem, gameState, self.heuristic)) == 0:
                return 'Stop'
            else:
                return aStarSearch(problem, gameState, self.heuristic)[0]

        if self.opponentscaredTime(gameState) is not None:
            if self.opponentscaredTime(gameState) > 20 and len(self.getFood(gameState).asList()) > 0:
                problem = Search(gameState, self, self.index, "searchfood")
                return aStarSearch(problem, gameState, self.heuristic)[0]

        if len(self.getFood(gameState).asList()) < 3 or gameState.data.timeleft < self.distToHome(gameState) + 60 \
                or gameState.getAgentState(self.index).numCarrying > 15:
            problem = Search(gameState, self, self.index, "backhome")
            if len(aStarSearch(problem, gameState, self.heuristic)) == 0:
                return 'Stop'
            else:
                return aStarSearch(problem, gameState, self.heuristic)[0]

        problem = Search(gameState, self, self.index, "searchfood")
        return aStarSearch(problem, gameState, self.heuristic)[0]

    def distToHome(self, gameState):
        myState = gameState.getAgentState(self.index)
        myPosition = myState.getPosition()
        validPositions = self.getBoundary(gameState)
        min_dist = float('inf')
        for validPosition in validPositions:
            tempDist = self.getMazeDistance(validPosition, myPosition)
            if tempDist < min_dist:
                min_dist = tempDist
        return min_dist

    def distToCapsule(self, gameState):
        if len(self.getCapsules(gameState)) > 1:
            min_dist = float('inf')
            for i in self.getCapsules(gameState):
                tempDist = self.getMazeDistance(gameState.getAgentState(self.index).getPosition(), i)
                if tempDist < min_dist:
                    min_dist = tempDist
            return min_dist
        elif len(self.getCapsules(gameState)) == 1:
            distToCapsule = self.getMazeDistance(gameState.getAgentState(self.index).getPosition(),
                                                 self.getCapsules(gameState)[0])
            return distToCapsule

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

    def opponentscaredTime(self, gameState):
        opponents = self.getOpponents(gameState)
        scaredTimes = [gameState.getAgentState(opponent).scaredTimer for opponent in opponents]
        validScaredTimes = [time for time in scaredTimes if time > 1]
        if validScaredTimes:
            return max(validScaredTimes)
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


#######################
#  defender   #
#######################

class DefensiveAgent(CaptureAgent):
    """
    A reflex agent that keeps its side Pacman-free. Again,
    this is to give you an idea of what a defensive agent
    could be like. It is not the best or only way to make
    such an agent.
    """
    def evaluate(self, gameState, action):
        """
        Computes a linear combination of features and feature weights
        """
        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        return features * weights

    def getWeights(self, gameState, action):
        """
        Normally, weights do not depend on the gamestate.  They can be either
        a counter or a dictionary.
        """
        return {'successorScore': 1.0}

    def getSuccessor(self, gameState, action):
        """
        Finds the next successor (Game state object)
        """
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def chooseAction(self, gameState):
        actions = gameState.getLegalActions(self.index)
        values = [self.evaluate(gameState, a) for a in actions]
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        invaders = [a for a in enemies if a.isPacman]
        knowninvaders = [a for a in enemies if a.isPacman and a.getPosition() is not None]
        self.locationOfLastEatenFood(gameState)  # detect last eaten food

        if len(invaders) == 0 or gameState.getAgentPosition(self.index) == self.lastEatenFoodPosition or len(
                knowninvaders) > 0:
            self.lastEatenFoodPosition = None

        # if number of invaders is less than two, we can go out and try to eat some food
        if len(invaders) < 1:
            # eat maximum 3 food
            if gameState.getAgentState(self.index).numCarrying < 3 and len(self.getFood(gameState).asList()) != 0 and not (
                    self.distToGhost(gameState) is not None and self.distToGhost(gameState)[0] < 4 and
                    self.distToGhost(gameState)[1].scaredTimer < 2):
                problem = Search(gameState, self, self.index, "searchfood")
                return aStarSearch(problem, gameState, self.heuristic)[0]
            else:
                problem = Search(gameState, self, self.index, "backhome")
                if len(aStarSearch(problem, gameState, self.heuristic)) == 0:
                    return 'Stop'
                else:
                    return aStarSearch(problem, gameState, self.heuristic)[0]

        # when number of invader > 0, we execute defense strategy
        else:
            if len(knowninvaders) == 0 and self.lastEatenFoodPosition != None and gameState.getAgentState(
                    self.index).scaredTimer == 0:
                problem = Search(gameState, self, self.index, "searchlasteatenfood")
                return aStarSearch(problem, gameState, self.heuristic)[0]

            if len(knowninvaders) > 0 and gameState.getAgentState(self.index).scaredTimer == 0:
                problem = Search(gameState, self, self.index, "searchinvader")
                return aStarSearch(problem, gameState, self.heuristic)[0]

        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]
        return random.choice(bestActions)


    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)

        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        features['dead'] = 0

        # Computes whether we're on defense (1) or offense (0)
        features['onDefense'] = 1
        if myState.isPacman: features['onDefense'] = 0

        # Computes distance to invaders we can see
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
        features['numInvaders'] = len(invaders)
        if len(invaders) > 0 and gameState.getAgentState(self.index).scaredTimer > 0:
            dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
            features['invaderDistance'] = -1 / min(dists)

        if action == Directions.STOP: features['stop'] = 1
        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev: features['reverse'] = 1
        features['DistToBoundary'] = - self.distToHome(successor)
        return features

    def distToHome(self, gameState):
        myState = gameState.getAgentState(self.index)
        myPosition = myState.getPosition()
        validPositions = self.getBoundary(gameState)
        min_dist = float('inf')
        for validPosition in validPositions:
            tempDist = self.getMazeDistance(validPosition, myPosition)
            if tempDist < min_dist:
                min_dist = tempDist
        return min_dist


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

    def locationOfLastEatenFood(self, gameState):
        ''''
        return the location of the last eaten food
        '''
        if len(self.observationHistory) > 1:
            prevState = self.getPreviousObservation()
            prevFoodList = self.getFoodYouAreDefending(prevState).asList()
            currentFoodList = self.getFoodYouAreDefending(gameState).asList()
            if len(prevFoodList) != len(currentFoodList):
                for food in prevFoodList:
                    if food not in currentFoodList:
                        self.lastEatenFoodPosition = food

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


def aStarSearch(problem, gameState, heuristic):
    start_state = problem.startState
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
            if problem.isGoalState(state):
                return path
            successors = problem.getSuccessors(state)
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
        elif self.goal == "searchcapsule":
            return state in self.capsule
        elif self.goal == "escape":
            return state in self.homeBoundary or state in self.capsule
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

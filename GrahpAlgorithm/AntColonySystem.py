import util

import numpy as np
import random
from numpy.random import choice as np_choice
from typing import Optional

from GridsMap.GridsMap import GridsMap, Node
from GridsMap.CollisionChecker import CollisionChecker
from PathFindingAlgorithm.Astar import Astar
from Graph2 import Graph
from GraphNode import GraphNode
from LineBuilder.SplineBuilder import SplineBuilder


from OCC.Core.gp import gp_Pnt
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import Viewer3d


class ACS:
    def __init__(self, distance: np.ndarray, degree: np.ndarray, ants=10, iteration=10, decay=0.1, beta=2, dst: int = 4) -> None:
        d1, d2 = degree[degree != 1e9], distance[distance != 1e9]
        self.Degree: np.ndarray = degree / np.max(d1)
        self.Distance: np.ndarray = distance / np.max(d2)
        self.Pheromone: np.ndarray = np.ones(distance.shape) / len(distance)
        self.inds = range(len(distance))
        self.Ants = ants
        self.Iteration = iteration
        self.Beta = beta
        self.Decay = decay
        self.q0 = 0.5
        self.dst = dst
        self.NewSpline = None

    def Run(self):
        globalShortestPath = [(), 1e9]
        for _ in range(self.Iteration):
            ret = self.__AntSimulation()

            if (ret == None):
                print(">?")
                continue

            shortestPath = min(ret, key=lambda x: x[1])
            print(shortestPath)
            if (shortestPath[1] < globalShortestPath[1]):
                globalShortestPath = shortestPath
            else:
                continue

            self.Pheromone = self.Pheromone * \
                (1 - self.Decay)

            for i in globalShortestPath[0]:
                gv = globalShortestPath[1]
                self.Pheromone[i] += (self.Decay * (gv ** -1))
        return globalShortestPath

    def __AntSimulation(self):
        candidate_path = []

        for _ in range(self.Ants):
            path = self.__AntWork(0)
            if (path):
                candidate_path.append((path, self.__EstimatePath(path)))
        return candidate_path

    def __AntWork(self, start):
        path = []
        visited: set[int] = set()
        visited.add(start)
        prev = start
        while True:
            move = self.__AntMove(0, visited)
            path.append((prev, move))
            prev = move
            if (not move in visited):
                visited.add(move)

                if (move == self.dst):
                    return path
            else:
                return None

    def __AntMove(self, src: int, visited: set[int]) -> int:
        q = random.uniform(0, 1)
        pheromone = np.copy(self.Pheromone[src])
        pheromone[list(visited)] = 0
        distance = np.copy(self.Distance[src])

        if (q > self.q0):
            row = (pheromone * ((1.0 / distance) ** self.Beta))
            row /= sum(row)
            move: int = np_choice(self.inds, 1, p=row)[0]
        else:
            row = (pheromone) * ((1.0 / distance) ** self.Beta)
            move: int = np.argmax(row)

        return move

    def __EstimatePath(self, path):
        totalDist = 0

        for i in path:
            totalDist += self.Distance[i]

        return totalDist


if (__name__ == "__main__"):
    a, b, c, d = init_display()
    random.seed(13)
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 30)
    grids.InitRandomBoxObstacle(20)
    grids.DisplayGridsMapInObstacle(a, _color="black", _transparency=0)
    N = len(grids.NodeMap) - 1
    startNode, endNode = grids.NodeMap[0][0][0], grids.NodeMap[N][N][N]

    ast = Astar()
    ast.Run(startNode, endNode, grids.NodeMap)
    ast._PostSmoothing(startNode, endNode, grids.NodeMap)

    gi = Graph()

    sb = SplineBuilder(ast.GetPathPoints(), 3)
    sb.DisplaySplineShape(a)
    col = CollisionChecker(grids, sb.SplineShape)
    if (col.Run()):
        gi.InitVisibiltyGraph(sb.SplineShape, grids, ast)

        acs = ACS(np.array(gi.Distance), np.array(gi.Degree),
                  10, 1000, 0.9, 2, gi.NodeNumber - 1)

        ret = acs.Run()
        print(ret)
        for i in range(len(ret[0])):
            node = gi.NodeTable[ret[0][i][1]]
            if (node in ast.PathNodeList):
                continue
            ast.PathNodeList.append(node)

        gi._SortNodeByDist(startNode, ast.PathNodeList)
        sb2 = SplineBuilder(ast.GetPathPoints(), 3)

        sb2.DisplaySplineShape(a, _color="green")

    b()

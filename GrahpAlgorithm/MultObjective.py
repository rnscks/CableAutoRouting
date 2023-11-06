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
from matplotlib import cm

from OCC.Core.AIS import AIS_LightSource
from OCC.Core.Graphic3d import Graphic3d_CLight, Graphic3d_TypeOfLightSource

from OCC.Core.gp import gp_Pnt
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import Viewer3d
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_TOC_RGB,
    Quantity_NOC_WHITE,
    Quantity_NOC_GRAY
)

import matplotlib.pyplot as plt
import numpy as np


class ACS:
    def __init__(self, distance: np.ndarray, degree: np.ndarray, ants=10, iteration=10, beta=2, dst: int = 4) -> None:
        d1, d2 = degree[degree != 1e9], distance[distance != 1e9]
        # degree[degree != 1e9] *= np.max(d2)

        self.d1 = np.max(d1)
        self.d2 = np.max(d2)

        degree[degree != 1e9] = (degree[degree != 1e9] - np.min(degree[degree != 1e9])) / (
            np.max(degree[degree != 1e9]) - np.min(degree[degree != 1e9]))
        self.Degree: np.ndarray = degree
        distance[distance != 1e9] = (distance[distance != 1e9] - np.min(distance[distance != 1e9])) / (
            np.max(distance[distance != 1e9]) - np.min(distance[distance != 1e9]))
        degree[degree == 0] = 0.001
        distance[distance == 0] = 0.001
        # distance[distance != 1e9] = distance[distance != 1e9] ** 1.08
        self.Distance: np.ndarray = distance

        # # # 그래프 파트
        # x = np.arange(len(degree[degree != 1e9]))  # x 축 레이블을 생성하기 위한 배열

        # # 그래프 그리기
        # plt.plot(x, sorted(degree[degree != 1e9]),
        #          label='degree', color='b')
        # plt.plot(x + 0.4, sorted(distance[distance != 1e9]),
        #          label='distance', color='g')

        # # 그래프 제목 및 레이블 설정
        # plt.title('data distribution', fontsize=20)
        # plt.xlabel('number of data', fontsize=16)
        # plt.ylabel('value', fontsize=16)
        # plt.xticks(fontsize=12)
        # plt.yticks(fontsize=12)

        # # 범례 표시
        # plt.legend(fontsize=16)

        # # 그래프 표시
        # plt.show()

        self.Pheromone1: np.ndarray = np.ones(distance.shape) / len(distance)
        self.Pheromone2: np.ndarray = np.ones(distance.shape) / len(distance)
        self.inds = range(len(distance))
        self.Ants = ants
        self.Iteration = iteration
        self.Beta = beta
        self.dst = dst
        self.SolutionSet = []
        self.NewSpline = None

    def Run(self):
        paretoSet = []
        for iter in range(self.Iteration):
            ret = self.__AntSimulation()
            print(iter)
            if (ret == None or len(ret) == 0):
                continue
            x = []
            y = []

            for i in ret:
                # if (i[1][0] >= 500 or i[1][1] >= 500):
                #     continue
                x.append(i[1][0])
                y.append(i[1][1])
            color = (iter / self.Iteration, 0, 0)

            # plt.scatter(x, y, color=[color], )

            paretoSet.extend(ret)

            ie = np.asarray(self.SolutionSet)
            self.__ParetoSampling(ie, self.SolutionSet, paretoSet)

            for j in range(len(paretoSet)):
                for i in paretoSet[j][0]:
                    self.Pheromone1[i] += (len(paretoSet) ** -1)
                    self.Pheromone2[i] += (len(paretoSet) ** -1)

        print(paretoSet[0][1][0], paretoSet[0][1][1])
        return paretoSet[0]

    def __AntSimulation(self):
        candidate_path = []

        for k in range(self.Ants):
            path = self.__AntWork(0, k + 1)
            if (path):
                solution = self.__EstimatePath(path)
                if (not solution in self.SolutionSet):
                    self.SolutionSet.append(solution)
                    candidate_path.append((path, solution))
        return candidate_path

    def __AntWork(self, start: int, k: int):
        path = []
        visited: set[int] = set()
        visited.add(start)
        prev = start
        while True:
            move = self.__AntMove(prev, k, visited)
            path.append((prev, move))
            prev = move
            if (not move in visited):
                visited.add(move)
                if (move == self.dst):
                    return path
            else:
                return None

    def __AntMove(self, src: int, k: int, visited: set[int]) -> int:
        pheromone1 = np.copy(self.Pheromone1[src])
        pheromone1[list(visited)] = 0
        pheromone2 = np.copy(self.Pheromone1[src])
        pheromone2[list(visited)] = 0

        distance = np.copy(self.Distance[src])
        degree = np.copy(self.Degree[src])

        # for i in range(len(degree)):
        #     if (degree[i] == 0):
        #         degree[i] = 0.001

        lam = (k - 1) / (self.Ants - 1)

        t1 = (pheromone1 ** lam)
        t2 = (pheromone2 ** (1 - lam))
        w1 = (1.0 / distance) ** (self.Beta * (lam))
        w2 = (1.0 / degree) ** (self.Beta * (1 - lam))

        row = t1 * t2 * w1 * w2

        row /= sum(row)
        # print(row)

        move: int = np_choice(self.inds, 1, p=row)[0]

        return move

    def __EstimatePath(self, path):
        totalDist = 0
        totalDegree = 0
        for i in path:
            totalDist += self.Distance[i]
            totalDegree += self.Degree[i]
        return (totalDist, totalDegree)

    def __ParetoSampling(self, costs, ref1, ref2):
        is_efficient = np.ones(costs.shape[0], dtype=bool)
        for i, c in enumerate(costs):
            if is_efficient[i]:
                # Keep any point with a lower cost
                is_efficient[is_efficient] = np.any(
                    costs[is_efficient] < c, axis=1)
                is_efficient[i] = True  # And keep self

        for i in range(len(is_efficient)):
            if (not is_efficient[i]):
                ref1.remove(tuple(costs[i]))
                for r in ref2:
                    if (r[1] == tuple(costs[i])):
                        ref2.remove(r)

        return


if (__name__ == "__main__"):
    a, b, c, d = init_display()
    a.View.SetBackgroundColor(Quantity_TOC_RGB, 0, 0, 0)
    a.View.SetBgGradientColors(
        Quantity_Color(Quantity_NOC_WHITE),
        Quantity_Color(Quantity_NOC_WHITE),
        2,
        True,
    )

    random.seed(7)
    ast = Astar()
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 20)
    grids.InitRandomBoxObstacle(20)
    grids.DisplayGridsMapInObstacle(a, _color="black", _transparency=0)

    N = len(grids.NodeMap) - 1
    startNode, endNode = grids.NodeMap[0][0][0], grids.NodeMap[N][N][N]
    ast.Run(startNode, endNode, grids.NodeMap)
    ast._PostSmoothing(startNode, endNode, grids.NodeMap)

    sb = SplineBuilder(ast.GetPathPoints(), 5)
    sb.DisplaySplineShape(a)
    if (sb.SplineShape is not None):
        chk = CollisionChecker(grids, sb.SplineShape)
    else:
        print("multi objective optimization spline shape error..")
        exit(0)

    if (chk.Run()):
        gi = Graph()

        nodes = chk.GetCollisionNode()
        nodes.extend(chk.GetCollisionNeiNode())
        for node in nodes:
            node.DisplayBoxShape(a, _color="red")
        gi.InitVisibiltyGraph(sb.SplineShape, grids, ast)
        if (gi.StartNode == None or gi.EndNode == None):
            exit(0)

        gi.StartNode.DisplayBoxShape(a, _color="blue")
        gi.EndNode.DisplayBoxShape(a, _color="blue")
        # acs = ACS(np.asarray(gi.Distance), np.asarray(
        #     gi.Degree), 10, 1000, 2, gi.NodeNumber - 1)

        # ret = acs.Run()
        # print(ret)

        # # plt.colorbar()
        # plt.show()

    b()

import time
import numpy as np
from numpy.random import choice as np_choice
import util
import math

import random

from OCC.Core.gp import gp_Pnt, gp_Vec
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import Viewer3d

from LineBuilder.SplineBuilder import SplineBuilder
from GridsMap.Node import Node
from CableRouting.Routing import NodeSeacher
from GridsMap.CollisionChecker.CollisionChecker import CollisionChecker
from PathFindingAlgorithm.Astar import Astar
from PathFindingAlgorithm.Agent import Agent
from GridsMap.GridsMap import GridsMap
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_TOC_RGB,
    Quantity_NOC_WHITE,
    Quantity_NOC_GRAY
)

ex = []
ex2 = []


class Graph:
    def __init__(self) -> None:
        self.NodeSet: dict[Node, list[Node]] = dict()
        self.NodeTable: dict[int, Node] = dict()
        self.Distance: list[list[float]] = [[]]
        self.Degree: list[list[float]] = [[]]
        self.NodeNumber = 0
        self.StartNode = None
        self.EndNode = None
        pass

    def __InitDistanceTable(self):
        self.Distance = [[np.inf for _ in range(
            self.NodeNumber)] for _ in range(self.NodeNumber)]
        for i in range(self.NodeNumber):
            for j in range(self.NodeNumber):
                if (i == j):
                    continue
                self.Distance[i][j] = self.NodeTable[i].CenterPoint.Distance(
                    self.NodeTable[j].CenterPoint)

    def __CalDegree(self, i, j):
        if (self.NodeTable[j] == self.StartNode or self.NodeTable[j] == self.EndNode):
            return 0
        g1 = self.NodeTable[i].CenterPoint.Coord()
        g2 = self.NodeTable[j].CenterPoint.Coord()

        v1 = gp_Vec(g1[0], g1[1], g1[2])
        v2 = gp_Vec(g2[0], g2[1], g2[2])
        dotVal: float = v1.Dot(v2)
        magnitude: float = v1.Magnitude() * v2.Magnitude()

        dotVal = round(dotVal, 8)
        magnitude = round(magnitude, 8)
        if (magnitude != 0):
            degree: float = math.degrees(math.acos(dotVal / magnitude))
            if (degree):
                return 1 / degree
            else:
                return 0
        else:
            degree = 0
            return 0

    def __InitDegreeTable(self):
        self.Degree = [[np.inf for _ in range(
            self.NodeNumber)] for _ in range(self.NodeNumber)]
        for i in range(self.NodeNumber):
            for j in range(self.NodeNumber):
                if (i == j):
                    continue
                self.Degree[i][j] = self.__CalDegree(i, j)

    def __PercentChance(self, percent: float) -> bool:
        if random.random() < percent:
            return True
        else:
            return False

    def __InitNodeSet(self, agent: Agent, grids: GridsMap):
        for node1 in self.NodeSet:
            self.NodeTable[self.NodeNumber] = node1
            for node2 in self.NodeSet:
                if (node1 == node2):
                    continue
                # if (agent._LineOfSight3D(node1, node2, grids.NodeMap)):
                self.NodeSet[node1].append(node2)
                self.NodeSet[node2].append(node1)
            self.NodeNumber += 1

    def InitRandomNodeByPercent(self, percent: float, agent: Agent, grids: GridsMap):
        for i in range(grids.MapSize):
            for j in range(grids.MapSize):
                for k in range(grids.MapSize):
                    if (i == 0 and j == 0 and k == 0):
                        continue
                    elif (i == grids.MapSize - 1 and j == grids.MapSize - 1 and k == grids.MapSize - 1):
                        continue
                    elif (k == grids.MapSize - 1 and j == 0):
                        continue
                    elif (k == grids.MapSize - 1 and i == grids.MapSize - 1):
                        continue
                    elif (j == 0 and i == 0):
                        continue
                    if (self.__PercentChance(percent) and not grids.NodeMap[i][j][k].Obstacle):
                        self.NodeSet[grids.NodeMap[i][j][k]] = []
        self.__InitNodeSet(agent, grids)

    def __InitedCollapseArea(self, x, y, z, grids: GridsMap):
        ret = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                for k in range(-1, 2):
                    if (i == j == k == 0):
                        continue
                    nx = i + x
                    ny = j + y
                    nz = k + z

                    if (nx >= grids.MapSize or ny >= grids.MapSize or nz >= grids.MapSize or nx < 0 or ny < 0 or nz < 0):
                        continue
                    if (grids.NodeMap[nx][ny][nz].Obstacle == False and not grids.NodeMap[nx][ny][nz] in ret):
                        ret.append(grids.NodeMap[nx][ny][nz])
                return ret
        return ret

    def _GetNodeDist(self, n1: Node, n2: Node, src: Node):
        return (n1.CenterPoint.Distance(src.CenterPoint) > n2.CenterPoint.Distance(src.CenterPoint))

    def _SortNodeByDist(self, startNode, nodeList: list[Node]) -> None:
        for i in range(len(nodeList) - 1):
            min_idx = i
            for j in range(i + 1, len(nodeList)):
                if (self._GetNodeDist(nodeList[min_idx], nodeList[j], startNode)):
                    min_idx = j
            nodeList[i], nodeList[min_idx] = nodeList[min_idx], nodeList[i]

    def InitVisibiltyGraph(self, splineShape, grids, agent: Agent):
        chk = CollisionChecker(grids, splineShape)
        nodes = chk.GetCollisionNode()
        self._SortNodeByDist(agent.PathNodeList[0], nodes)
        tmp = 0
        tmp2 = 0
        for i in range(1, len(agent.PathNodeList)):
            if (agent.PathNodeList[0].CenterPoint.Distance(agent.PathNodeList[i].CenterPoint) > agent.PathNodeList[0].CenterPoint.Distance(nodes[0].CenterPoint)):
                tmp = i
            if (tmp != 0 and agent.PathNodeList[0].CenterPoint.Distance(agent.PathNodeList[i].CenterPoint) > agent.PathNodeList[0].CenterPoint.Distance(nodes[len(nodes) - 1].CenterPoint)):
                tmp2 = i
        self.StartNode = agent.PathNodeList[tmp - 1]
        self.EndNode = agent.PathNodeList[tmp2]

        ret = []
        for node in nodes:
            ret.extend(self.__InitedCollapseArea(
                node.I, node.J, node.K, grids))
        ret.append(self.StartNode)
        ret.append(self.EndNode)
        self._SortNodeByDist(agent.PathNodeList[0], ret)
        for node in ret:
            self.NodeSet[node] = []

        self.__InitNodeSet(Agent(), grids)
        self.__InitDistanceTable()
        self.__InitDegreeTable()

    def DispalyGraph(self, display: Viewer3d):
        flag = True
        for node1 in self.NodeSet:
            if (flag or node1 == self.NodeTable[int((self.NodeNumber - 3))]):
                node1.DisplayBoxShape(display, _color="red")
                flag = False
            else:
                node1.DisplayBoxShape(display, _color="red")
            for node2 in self.NodeSet:
                if (node1 == node2):
                    continue
                # edge = BRepBuilderAPI_MakeEdge(
                #     node1.CenterPoint, node2.CenterPoint).Edge()
                # display.DisplayShape(edge, transparency=0.5)


class AntColonySystem:
    def __init__(self, distances, n_ants=10, n_iteration=2500, decay=0.1, beta=2, startNode: Node = None, endNode: Node = None) -> None:
        self.distances = distances
        self.all_inds = range(len(distances))
        self.n_ants = n_ants
        self.n_iteration = n_iteration
        self.beta = beta
        self.decay = decay
        self.pheromone1 = np.ones(self.distances.shape) / len(self.distances)
        self.pheromone2 = np.ones(self.distances.shape) / len(self.distances)
        self.InitPheromone()
        self.tu = np.copy(self.pheromone)
        self.q0 = 0.1
        self.NewSpline = None
        self.StartNode = startNode
        self.EndNode = endNode

        pass

    def _GetNodeDist(self, n1: Node, n2: Node, src: Node):
        return (n1.CenterPoint.Distance(src.CenterPoint) > n2.CenterPoint.Distance(src.CenterPoint))

    def _SortNodeByDist(self, startNode, nodeList: list[Node]) -> None:
        for i in range(len(nodeList) - 1):
            min_idx = i
            for j in range(i + 1, len(nodeList)):
                if (self._GetNodeDist(nodeList[min_idx], nodeList[j], startNode)):
                    min_idx = j
            nodeList[i], nodeList[min_idx] = nodeList[min_idx], nodeList[i]

    def _PathNodeInsert(self, path: list[tuple[int, int]]) -> None:
        nodes = [graph.NodeTable[path[0][0]]]
        for i in range(len(path)):
            nodes.append(graph.NodeTable[path[i][1]])
        for i in nodes:
            if (i == self.StartNode or i == self.EndNode or i in ret_path):
                continue
            astar.PathNodeList.append(i)
        self._SortNodeByDist(grids.NodeMap[0][0][0], astar.PathNodeList)

    def _PathNodeDelete(self, path: list[tuple[int, int]]) -> None:
        nodes = [graph.NodeTable[path[0][0]]]
        for i in range(len(path)):
            nodes.append(graph.NodeTable[path[i][1]])

        for i in nodes:
            if (i == self.StartNode or i == self.EndNode or i in ret_path):
                continue
            if (i in astar.PathNodeList):
                astar.PathNodeList.remove(i)

    def _GetPathGpPnt(self) -> list[gp_Pnt]:
        ret = []
        for i in astar.PathNodeList:
            ret.append(i.CenterPoint)
        return ret

    def Run(self, stop):
        self.stop = stop
        startTime = time.time()
        global_shortest_path = ((), 1e9)
        # get global shorest path
        cnt = 0
        for _ in range(self.n_iteration):
            # print(cnt)
            cnt += 1
            ret = self.AntSimulation()

            if (not (ret is None or len(ret) == 0)):
                shorest_path = min(ret, key=lambda x: x[1])
                ex.append(shorest_path[1])
                print(shorest_path)
                if (shorest_path[1] < global_shortest_path[1]):
                    global_shortest_path = shorest_path

            self.pheromone = self.pheromone * (1 - self.decay)

            for i in global_shortest_path[0]:
                self.pheromone1[i] += (0.1 * (global_shortest_path[1] ** -1))
            ex2.append(global_shortest_path[1])
            print("done", global_shortest_path[0])

        print(time.time()-startTime)

        self._PathNodeInsert(global_shortest_path[0])
        pnts = self._GetPathGpPnt()

        r = 0
        for i in range(len(pnts) - 1):
            r += pnts[i].Distance(pnts[i + 1])
        print(r)
        print(len(pnts))
        print(global_shortest_path)
        self.NewSpline = SplineBuilder(pnts, 3).SplineShape
        a.DisplayShape(self.NewSpline, color="green")

        return global_shortest_path

    def AntMove(self, pheromone1, pheromone2, dist, visited, ant):
        q = random.uniform(0, 1)
        pheromone1 = np.copy(pheromone1)
        pheromone2 = np.copy(pheromone2)
        # tu = np.copy(self.tu[prev])
        pheromone1[list(visited)] = 0
        pheromone2[list(visited)] = 0
        lam = (ant - 1) / (self.n_ants - 1)
        row = ((pheromone1) ** lam) * ((pheromone2) ** (1 - lam)) * \
            ((1.0 / dist) ** (self.beta * lam) *
             (1.0 / dist) ** (self.beta * (1 - lam)))
        row /= sum(row)
        move = np_choice(self.all_inds, 1, p=row)[0]

        return move

    def AntWork(self, start, ant):
        path = []
        visited = set()
        visited.add(start)
        prev = start
        while True:
            move = self.AntMove(
                self.pheromone1[prev], self.pheromone2[prev], self.distances[prev], visited, ant)
            path.append((prev, move))
            prev = move

            if (not move in visited):
                visited.add(move)
                if (move == len(self.distances) - 1):
                    return path
            else:
                return None

    def EstimatePath(self, path):
        total_dist = 0
        for i in path:
            total_dist += self.distances[i]
        return total_dist

    def AntSimulation(self):
        candidate_path = []
        pareto_set = []
        for ant in range(self.n_ants):
            path = self.AntWork(0, ant + 1)
            if (path):
                try:
                    self._PathNodeInsert(path)
                    pnts = self._GetPathGpPnt()
                    self._PathNodeDelete(path)

                    sb = SplineBuilder(pnts, 3).SplineShape
                    chk = CollisionChecker(grids, sb)
                    if (chk.Run()):
                        # for i in path[0]:
                        #     self.pheromone[i] = self.pheromone[i] * (1 - 0.01)
                        pass
                    else:
                        d = self.EstimatePath(path)
                        # for i in path:
                        #     self.pheromone[i] = self.pheromone[i] + \
                        #         (0.001 * (d ** -1))
                        candidate_path.append((path, d))

                    del sb

                except RuntimeError:
                    continue

        return candidate_path

    def InitPheromone(self):
        for i in range(len(self.distances)):
            tu = (min(self.distances[i]) * len(self.distances)) ** -1
            self.pheromone1[i] = np.array(
                [tu for _ in range(len(self.distances))])


if (__name__ == "__main__"):
    a, b, c, d = init_display()
    a.View.SetBackgroundColor(Quantity_TOC_RGB, 0, 0, 0)
    a.View.SetBgGradientColors(
        Quantity_Color(Quantity_NOC_WHITE),
        Quantity_Color(Quantity_NOC_WHITE),
        2,
        True,
    )

    # seed : 23, map size : 20
    # ant : 10, q0 : 0.5, decay(collision)
    # 10, 15
    # 13, 15
    # 23, 20

    random.seed(23)
    graph = Graph()
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 20)

    grids.InitRandomBoxObstacle(grids.MapSize - 5)

    grids.DisplayGridsMapInObstacle(a, _transparency=0, _color="black")
    astar = Astar()

    astar.Run(grids.NodeMap[0][0][0], grids.NodeMap[grids.MapSize - 1]
              [grids.MapSize - 1][grids.MapSize - 1], grids.NodeMap)
    astar._PostSmoothing(grids.NodeMap[0][0][0], grids.NodeMap[grids.MapSize - 1]
                         [grids.MapSize - 1][grids.MapSize - 1], grids.NodeMap)

    ret_path = []
    for i in astar.PathNodeList:
        ret_path.append(i)
    sb = SplineBuilder(astar.GetPathPoints(), 3)
    sb.DisplaySplineShape(a, _transparency=0, _color="black")
    graph.InitVisibiltyGraph(sb.SplineShape, grids, astar)
    print(graph.Degree)
    chk = CollisionChecker(grids, sb.SplineShape)
    w2 = chk.GetCollisionNode()
    for node in w2:
        node.DisplayBoxShape(a, _color="red", _transparency=0)
    # graph.DispalyGraph(a)

    ant = AntColonySystem(np.array(graph.Distance),
                          n_iteration=100, decay=0.1, startNode=graph.StartNode, endNode=graph.EndNode)
    ret = ant.Run(len(w2))

    print(len(astar.PathNodeList))
    print("iteration")
    for i in ex:
        print(i)
    print("best tour")
    for i in ex2:
        print(i)

    b()
    pass

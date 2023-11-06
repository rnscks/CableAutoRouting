import util

import numpy as np
from typing import Optional

from GraphNode import GraphNode

from OCC.Core.gp import gp_Pnt
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import Viewer3d

from PathFindingAlgorithm.Agent import Agent
from GridsMap.Node import Node
from GridsMap.GridsMap import GridsMap
from PathFindingAlgorithm.Astar import Astar
from GridsMap.CollisionChecker import CollisionChecker
from LineBuilder.SplineBuilder import SplineBuilder


from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_TOC_RGB,
    Quantity_NOC_WHITE,
)


class Graph:
    def __init__(self) -> None:
        self.NodeSet: dict[Node, list[Node]] = dict()
        self.NodeTable: dict[int, Node] = dict()
        self.Distance: list[list[float]] = [[]]
        self.Degree: list[list[float]] = [[]]
        self.NodeNumber = 0
        self.StartNode: Optional[Node] = None
        self.EndNode: Optional[Node] = None
        self.n1: Optional[Node] = None
        self.n2: Optional[Node] = None
        pass

    def __InitDistanceTable(self, agent: Agent, grids: GridsMap):
        self.Distance = [[1e9 for _ in range(
            self.NodeNumber)] for _ in range(self.NodeNumber)]
        self.Degree = [[1e9 for _ in range(
            self.NodeNumber)] for _ in range(self.NodeNumber)]

        for i in range(self.NodeNumber):
            for j in range(self.NodeNumber):
                if (i == j):
                    continue
                if (i == 0 and j == self.NodeNumber - 1):
                    continue
                if (not (agent._LineOfSight3D(self.NodeTable[i], self.NodeTable[j], grids.NodeMap) and agent._LineOfSight3D(self.NodeTable[j], self.NodeTable[i], grids.NodeMap))):
                    continue

                self.Distance[i][j] = self.NodeTable[i].CenterPoint.Distance(
                    self.NodeTable[j].CenterPoint)
                self.Degree[i][j] = 0

                if (self.EndNode is not None and self.NodeTable[j] != self.EndNode):
                    self.Degree[i][j] = self.NodeTable[j].GetDegree(
                        self.NodeTable[i], self.EndNode)

    def __InitNodeSet(self, agent: Agent, grids: GridsMap):
        for node1 in self.NodeSet:
            self.NodeTable[self.NodeNumber] = node1
            for node2 in self.NodeSet:
                if (node1 == node2):
                    continue
                if (agent._LineOfSight3D(node1, node2, grids.NodeMap)):
                    self.NodeSet[node1].append(node2)
                    self.NodeSet[node2].append(node1)
            self.NodeNumber += 1

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
        if (not chk.Run()):
            return
        ret = chk.GetCollisionNeiNode()
        N = len(agent.PathNodeList) - 1
        tmp1 = 1e9
        tmp2 = 1e9
        node1 = None
        node2 = None

        startNode = agent.PathNodeList[0]
        endNode = agent.PathNodeList[N]

        for node in ret:
            if (tmp2 > startNode.CenterPoint.Distance(node.CenterPoint) and node != startNode):
                tmp2 = startNode.CenterPoint.Distance(node.CenterPoint)
                node2 = node

            if (tmp1 > endNode.CenterPoint.Distance(node.CenterPoint) and node != endNode):
                tmp1 = endNode.CenterPoint.Distance(node.CenterPoint)
                node1 = node

        tmp1 = 0
        tmp2 = 1e9
        self.n1 = node1
        self.n2 = node2
        if (node1 is None or node2 is None):
            return
        start = None
        end = None

        for node in agent.PathNodeList:
            if (tmp2 > startNode.CenterPoint.Distance(node.CenterPoint) and node is not startNode):
                print("start to path node:", startNode.CenterPoint.Distance(
                    node.CenterPoint), end='|')
                print("start to node:",
                      node2.CenterPoint.Distance(startNode.CenterPoint))
                if (node2.CenterPoint.Distance(startNode.CenterPoint) > startNode.CenterPoint.Distance(node.CenterPoint)):
                    tmp2 = startNode.CenterPoint.Distance(node.CenterPoint)
                    start = node

            if (tmp1 < endNode.CenterPoint.Distance(node.CenterPoint) and node is not endNode):
                if (node1.CenterPoint.Distance(endNode.CenterPoint) > endNode.CenterPoint.Distance(node.CenterPoint)):
                    tmp1 = endNode.CenterPoint.Distance(node.CenterPoint)
                    end = node

        if (start is None):
            start = startNode
        if (end is None):
            end = endNode

        self.StartNode = start
        self.EndNode = end
        if (not start in ret):
            ret.append(start)
        if (not end in ret):
            ret.append(end)

        self._SortNodeByDist(agent.PathNodeList[0], ret)
        for node in ret:
            self.NodeSet[node] = []

        self.__InitNodeSet(agent, grids)
        self.__InitDistanceTable(agent, grids)

    def InitGraphByNodeGrids(self, agent: Agent, grids: GridsMap):

        N = len(grids.NodeMap) - 1
        self.StartNode = grids.NodeMap[0][0][0]
        self.EndNode = grids.NodeMap[N][N][N]

        ret = []
        for i in range(N):
            for j in range(N):
                for k in range(N):
                    if (grids.NodeMap[i][j][k].Obstacle):
                        continue
                    ret.append(grids.NodeMap[i][j][k])

        self._SortNodeByDist(self.StartNode, ret)
        for node in ret:
            self.NodeSet[node] = []

        self.__InitNodeSet(agent, grids)
        self.__InitDistanceTable(agent, grids)


if (__name__ == "__main__"):
    a, b, c, d = init_display()
    a.View.SetBackgroundColor(Quantity_TOC_RGB, 0, 0, 0)
    a.View.SetBgGradientColors(
        Quantity_Color(Quantity_NOC_WHITE),
        Quantity_Color(Quantity_NOC_WHITE),
        2,
        True,
    )

    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(100, 100, 100), 20)

    grids.InitRandomBoxObstacle(20)

    astar = Astar()
    N = len(grids.NodeMap) - 1
    startNode = grids.NodeMap[0][0][0]
    endNode = grids.NodeMap[N][N][N]
    astar.Run(startNode, endNode, grids.NodeMap)
    astar._PostSmoothing(startNode, endNode, grids.NodeMap)

    graph = Graph()
    sb = SplineBuilder(astar.GetPathPoints(), 0.1)

    graph.InitVisibiltyGraph(sb.SplineShape, grids, astar)
    print(graph.Degree)
    print(graph.Distance)

    b()
    pass

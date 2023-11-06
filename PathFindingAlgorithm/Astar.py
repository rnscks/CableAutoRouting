import numba
import util

from time import time
import heapq

from OCC.Core.gp import gp_Pnt
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_TOC_RGB,
    Quantity_NOC_WHITE
)
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere

from OCC.Display.SimpleGui import init_display
import random
from Agent import Agent
from LineBuilder.SplineBuilder import SplineBuilder
from LineBuilder.PipeBuilder import PipeBuilder
from GridsMap.GridsMap import GridsMap
from GridsMap.Node import Node
from SplineOptimizer.BruteForce import BruteForce
from SplineOptimizer.BackTracking import BackTracking
from GridsMap.CollisionChecker import CollisionChecker
import matplotlib.pyplot as plt


def IsObstacle(src: Node):
    return src.Obstacle


class Astar(Agent):
    def __init__(self, name: str = "agent") -> None:
        super().__init__(name)
        self.NodeMap = None

    def Run(self, src: Node, dst: Node, nodeMap: list[list[list[Node]]]) -> None:
        startTime: float = time()
        openList: list[Node] = []
        closedList: set[Node] = set()
        src.Parent = src
        heapq.heappush(openList, src)
        n = len(nodeMap)

        while (openList):
            curNode: Node = heapq.heappop(openList)
            closedList.add(curNode)
            # print(time(), startTime)

            if (curNode == dst):

                self.CalTime = time() - startTime
                self._InitPathNodeList(src, dst)
                self._InitDegree()
                self._InitDistance()
                return

            for i in range(-1, 2):
                for j in range(-1, 2):
                    for k in range(-1, 2):
                        if (i == 0 and j == 0 and k == 0):
                            continue
                        nx: int = i + curNode.I
                        ny: int = j + curNode.J
                        nz: int = k + curNode.K

                        if (self._IsOutOfRange(nx, ny, nz, n)):
                            continue

                        if (nodeMap[nx][ny][nz].Obstacle or nodeMap[nx][ny][nz] in closedList):
                            continue

                        cost: float = (
                            max(-i, i) + max(-j, j) + max(-k, k)) ** 0.5
                        ng: float = curNode.G + cost
                        # nextNode: Node = nodeMap[nx][ny][nz]
                        # if (nextNode.Parent == None or ng < nextNode.G):
                        #     nextNode.G = ng
                        #     nextNode.H = self._CalEuclidDistance(nextNode, dst)
                        #     nextNode.F = nextNode.G + nextNode.H
                        #     nextNode.Parent = curNode

                        #     heapq.heappush(openList, nextNode)

                        if (nodeMap[nx][ny][nz].Parent == None or ng < nodeMap[nx][ny][nz].G):
                            nodeMap[nx][ny][nz].G = ng
                            nodeMap[nx][ny][nz].F = self._CalEuclidDistance(
                                nodeMap[nx][ny][nz], dst) + nodeMap[nx][ny][nz].G
                            nodeMap[nx][ny][nz].Parent = curNode
                            heapq.heappush(openList, nodeMap[nx][ny][nz])


if (__name__ == "__main__"):
    a, b, c, d = init_display()
    a.View.SetBackgroundColor(Quantity_TOC_RGB, 0, 0, 0)
    a.hide_triedron()

    a.View.SetBgGradientColors(
        Quantity_Color(Quantity_NOC_WHITE),
        Quantity_Color(Quantity_NOC_WHITE),
        2,
        True,
    )
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 10)
    random.seed(322133)
    grids.InitRandomBoxObstacle(10)
    grids.DisplayGridsMapInObstacle(a, _color="black", _transparency=0)

    ast = Astar()
    ast.Run(grids.NodeMap[0][0][0], grids.NodeMap[grids.MapSize - 1]
            [grids.MapSize - 1][grids.MapSize - 1], grids.NodeMap)
    ast._PostSmoothing(grids.NodeMap[0][0][0], grids.NodeMap[grids.MapSize - 1]
                       [grids.MapSize - 1][grids.MapSize - 1], grids.NodeMap)
    sb = SplineBuilder(ast.GetPathPoints(), 3)
    a.DisplayShape(sb.SplineShape)

    if (sb.SplineShape):
        sb.DisplaySplineShape(a, _color="black", _transparency=0)
        chk: CollisionChecker = CollisionChecker(grids, sb.SplineShape)
        nodes = chk.GetCollisionNode()

        for node in nodes:
            node.DisplayBoxShape(a, _color="red")

        if (sb.SplineShape):
            if (grids.CollisionCheck(sb.SplineShape)):
                bk = BackTracking(grids.NodeMap[0][0][0], grids.NodeMap[grids.MapSize - 1][grids.MapSize - 1]
                                  [grids.MapSize - 1], ast.PathNodeList, sb.SplineShape, grids, len(nodes) + 1)
                bk.Run()
                print("Done")
                if (bk.NewSpline[0]):
                    print("Visualization")
                    a.DisplayShape(bk.NewSpline[0], color="green")
                    for i in bk.Ret:
                        s = BRepPrimAPI_MakeSphere(i.CenterPoint, 1).Shape()
                        a.DisplayShape(s, color="red")

    a.FitAll()
    b()

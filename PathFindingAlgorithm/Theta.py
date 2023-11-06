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
from GridsMap.CollisionChecker.CollisionChecker import CollisionChecker
import matplotlib.pyplot as plt


class Theta(Agent):
    def __init__(self, name: str = "agent") -> None:
        super().__init__(name)

    def Run(self, src: Node, dst: Node, nodeMap: list[list[list[Node]]]) -> None:
        startTime: float = time()
        openList: list[Node] = []
        closedList: set[Node] = set()

        src.Parent = src
        heapq.heappush(openList, src)

        while (openList):
            curNode: Node = heapq.heappop(openList)
            closedList.add(curNode)

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

                        if (self._IsOutOfRange(nx, ny, nz, len(nodeMap))):
                            continue

                        nextNode: Node = nodeMap[nx][ny][nz]
                        if (nextNode.Obstacle or nextNode in closedList):
                            continue

                        if (self._LineOfSight3D(curNode.Parent, nextNode, nodeMap)):
                            ng: float = curNode.Parent.G + \
                                self._CalEuclidDistance(
                                    nextNode, curNode.Parent)
                            if (nextNode.Parent == None or ng < nextNode.G):
                                nextNode.G = ng
                                nextNode.F = ng + \
                                    self._CalEuclidDistance(nextNode, dst)
                                nextNode.Parent = curNode.Parent
                                heapq.heappush(openList, nextNode)
                        else:
                            cost: float = (
                                max(-i, i) + max(-j, j) + max(-k, k)) ** 0.5
                            ng = curNode.G + cost
                            if (nextNode.Parent == None or ng < nextNode.G):
                                nextNode.G = ng
                                nextNode.F = self._CalEuclidDistance(
                                    nextNode, dst) + ng
                                nextNode.Parent = curNode
                                heapq.heappush(openList, nextNode)


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

    startTime = time()
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 10)
    # 1081, 9, 1002, 9
    # random.seed(1213)
    # random.seed(223)
    grids.InitRandomBoxObstacle(10)
    grids.DisplayGridsMapInObstacle(a, _color="black", _transparency=0)

    start = grids.NodeMap[0][0][0]

    end = grids.NodeMap[9][9][9]

    astar = Theta("astar")
    astar.Run(start, end, grids.NodeMap)
    # astar._PostSmoothing(start, end, grids.NodeMap)
    # astar._PostSmoothing(start, end, grids.NodeMap)
    pnts = astar.GetPathPoints()
    for i in pnts:
        sh = BRepPrimAPI_MakeSphere(i, 10).Shape()
        a.DisplayShape(sh, color="blue")
    sb = SplineBuilder(pnts, 3)
    sb.DisplaySplineShape(a, _color="black")
    # chk: CollisionChecker = CollisionChecker(grids, sb.SplineShape)
    # nodes = chk.GetCollisionNode()
    # bk = BackTracking(start, end, astar.PathNodeList,
    #                   sb.SplineShape, grids, len(nodes) + 1)
    # for i in nodes:
    #     i.DisplayBoxShape(a, _color="red", _transparency=0)
    #     ret = bk._InitedCollapseArea(i.I, i.J, i.K)
    # for j in ret:
    # sh = BRepPrimAPI_MakeSphere(j.CenterPoint, 10).Shape()
    # a.DisplayShape(sh, color="red")

    # for i in pnts:
    #     sh = BRepPrimAPI_MakeSphere(i, 10).Shape()
    #     a.DisplayShape(sh, color="blue")
    # pipe = PipeBuilder(astar.GetPathPoints(), 3)
    # pipe.DisplayPipeShape(a)
    # sb = SplineBuilder(astar.GetPathPoints(), 3)
    # sb.DisplaySplineShape(a, _color="black", _transparency=0)
    # chk: CollisionChecker = CollisionChecker(grids, sb.SplineShape)
    # nodes = chk.GetCollisionNode()
    # for node in nodes:
    #     node.DisplayBoxShape(a, _color="red")

    # if (sb.SplineShape):
    #     sb.DisplaySplineShape(a, _color="black", _transparency=0)
    #     chk: CollisionChecker = CollisionChecker(grids, sb.SplineShape)
    #     nodes = chk.GetCollisionNode()
    #     for node in astar.PathNodeList:
    #         s = BRepPrimAPI_MakeSphere(node.CenterPoint, 10).Shape()
    #         a.DisplayShape(s, color="blue")

    #     for node in nodes:
    #         node.DisplayBoxShape(a, _color="red")

    #     if (sb.SplineShape):
    #         if (grids.CollisionCheck(sb.SplineShape)):
    #             for node in astar.PathNodeList:
    #                 s = BRepPrimAPI_MakeSphere(node.CenterPoint, 10).Shape()
    #                 a.DisplayShape(s, color="blue")

    #             bf = BackTracking(start, end, astar.PathNodeList,
    #                               sb.SplineShape, grids, len(nodes) + 1)
    #             bf.Run()
    #             print("Done")
    #             if (bf.NewSpline[0]):
    #                 print("Visualization")
    #                 a.DisplayShape(bf.NewSpline[0], color="green")
    #                 for i in bf.Ret:
    #                     s = BRepPrimAPI_MakeSphere(i.CenterPoint, 10).Shape()
    #                     a.DisplayShape(s, color="red")

    a.FitAll()
    b()

    pass

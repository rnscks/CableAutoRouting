import util

import numpy as np
import random

from GridsMap.GridsMap import GridsMap
from GridsMap.CollisionChecker import CollisionChecker
from PathFindingAlgorithm.Astar import Astar
from Graph2 import Graph
from LineBuilder.SplineBuilder import SplineBuilder


from OCC.Core.gp import gp_Pnt
from OCC.Display.SimpleGui import init_display
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_TOC_RGB,
    Quantity_NOC_WHITE,
)
import numpy as np


class ACOR:
    def __init__(self, columnOfArchive,q, zeta, iteration, start: gp_Pnt, end: gp_Pnt,gpList: list[gp_Pnt], grids:GridsMap) -> None:
        self.q = q
        self.zeta = zeta
        self.N = len(gpList) * 3
        self.K = columnOfArchive
        self.SolutionArchive = np.zeros((self.K, self.N + 2))
        self.OmegaList = np.zeros((self.K, ))
        self.GpList:list[gp_Pnt] = gpList
        self.Iteration = iteration
        self.StartPnt:gp_Pnt = start
        self.EndPnt: gp_Pnt = end
        self.Grids:GridsMap = grids

        self.__InitializeSolutionArchive()
        return

    def Run(self):
        for _ in range(self.Iteration):
            self.__SelectTheGFunctionSamplingNumbers()

        return self.SolutionArchive[0]

    def __CalOmega(self, index):
        return ((self.q * self.K * np.sqrt(2 * np.pi)) ** -1) * np.exp(-(np.square(index - 1)/(2 * np.square(self.q) * np.square(self.K))))

    def __InitializeSolutionArchive(self) -> None:
        DEVIATION = 1

        for i in range(self.K):
            sampingNumbers = []

            while len(sampingNumbers) < self.N:
                sampleNumber = np.random.normal(5, DEVIATION)
                sampingNumbers.append(sampleNumber)
            sampingNumbers = np.asarray(sampingNumbers)

            for j in range(len(sampingNumbers)):
                self.SolutionArchive[i, j] = sampingNumbers[j]
            solutions = sampingNumbers
            self.SolutionArchive[i, self.N] = self.__Estimate(solutions)

        for i in range(self.K):
            self.SolutionArchive[i, self.N + 1] = self.__CalOmega(i)
            self.OmegaList[i] = self.SolutionArchive[i, self.N + 1]
        sortedIndices = np.argsort(self.SolutionArchive[:, self.N + 1])
        self.SolutionArchive = np.copy(self.SolutionArchive[sortedIndices])
        return

    def __SelectTheGFunctionSamplingNumbers(self) -> None:
        p = self.OmegaList/sum(self.OmegaList)
        ret = np.zeros((self.N))

        for i in range(self.N):
            l = np.random.choice(len(self.OmegaList), p=p)
            deviation = 0

            for j in range(self.K):
                deviation += np.absolute(
                    self.SolutionArchive[j, i] - self.SolutionArchive[l, i])
            deviation *= ((self.zeta) * ((self.K - 1) ** -1))

            ret[i] = np.random.normal(
                    self.SolutionArchive[l, i], deviation)

        for i in range(self.N):
            self.SolutionArchive[self.K - 1, i] = ret[i]

        self.__AppendSolution(ret)
        return

    def __AppendSolution(self, solutions):
        self.SolutionArchive[self.K - 1,
                             self.N] = self.__Estimate(solutions)
        sortedIndices = np.argsort(self.SolutionArchive[:, self.N])
        self.SolutionArchive = np.copy(self.SolutionArchive[sortedIndices])

        for i in range(self.K):
            self.SolutionArchive[i, self.N + 1] = self.__CalOmega(i)
            self.OmegaList[i] = self.SolutionArchive[i, self.N + 1]

    def __Estimate(self, solutions: np.ndarray):
        sols = []
        for i in range(len(self.GpList)):
            x = self.GpList[i].X() + solutions[i * 3]
            y = self.GpList[i].Y() + solutions[i * 3 + 1]
            z = self.GpList[i].Z() + solutions[i * 3 + 2]
            sols.append(gp_Pnt(x, y, z)) 

        sols.insert(0, self.StartPnt)
        sols.append(self.EndPnt)
        
        sb = SplineBuilder(sols, 5)
        chk = CollisionChecker(self.Grids, sb.SplineShape)
        print(chk.ObjectiveFuntion())
        return chk.ObjectiveFuntion()
    
    def ReturnSbShape(self):
        sols = []
        for i in range(len(self.GpList)):
            x = self.GpList[i].X() + self.SolutionArchive[0, i * 3]
            y = self.GpList[i].Y() + self.SolutionArchive[0 ,i * 3 + 1]
            z = self.GpList[i].Z() + self.SolutionArchive[0, i * 3 + 2]
            sols.append(gp_Pnt(x, y, z)) 

        sols.insert(0, self.StartPnt)
        sols.append(self.EndPnt)
        
        sb = SplineBuilder(sols, 5)
        chk = CollisionChecker(self.Grids, sb.SplineShape)
        print("ret:", chk.ObjectiveFuntion())

        return sb.SplineShape


if (__name__ == "__main__"):
    a, b, c, d = init_display()
    a.View.SetBackgroundColor(Quantity_TOC_RGB, 0, 0, 0)
    a.View.SetBgGradientColors(
        Quantity_Color(Quantity_NOC_WHITE),
        Quantity_Color(Quantity_NOC_WHITE),
        2,
        True,
    )

    random.seed(211)
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
    chk = CollisionChecker(grids, sb.SplineShape)

    if (chk.Run()):
        gpList = ast.GetPathPoints()
        acor = ACOR(50, 1, 0.65, 1000, gpList[0], gpList[len(gpList) - 1], gpList, grids)
        acor.Run()
        sbshape = acor.ReturnSbShape()
        if (sbshape != None):
            a.DisplayShape(sbshape)
        
    b()

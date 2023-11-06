import util

from typing import Optional
import numpy as np
import random

from GridsMap.GridsMap import GridsMap
from GridsMap.CollisionChecker import CollisionChecker
from PathFindingAlgorithm.Astar import Astar
from LineBuilder.SplineBuilder import SplineBuilder
from LineBuilder.SplineBuilderVector import InitalVectorList
from LineBuilder.SplineBuilderCurvature import SplineBuilderCurvature
from StoreShape import SaveShpaeForSplineTest
from ExcelWriter import ACORTestExcelWriter
from OCC.Core.gp import gp_Pnt, gp_Vec
import numpy as np


class ACOR:
    def __init__(self, columnOfArchive, q, zeta, iteration, start: gp_Pnt, end: gp_Pnt, startVec: gp_Vec, endVec: gp_Vec, gpList: list[gp_Pnt], gpVList: list[gp_Vec], initalInnerPoint: int, grids: GridsMap) -> None:
        self.q = q
        self.zeta = zeta
        self.N = len(gpList) * 6
        self.K = columnOfArchive
        self.SolutionArchive = np.zeros((self.K, self.N + 2))
        self.OmegaList = np.zeros((self.K, ))
        self.GpList: list[gp_Pnt] = gpList
        self.GpVList: list[gp_Vec] = gpVList
        self.Iteration = iteration
        self.StartPnt: gp_Pnt = start
        self.StartVec: gp_Vec = startVec
        self.EndVec: gp_Vec = endVec
        self.EndPnt: gp_Pnt = end
        self.Grids: GridsMap = grids
        self.Excelwriter: ACORTestExcelWriter = ACORTestExcelWriter(gpList)
        self.IterationNumber = 0
        self.InitialInnerPoint = initalInnerPoint

        self.__InitializeSolutionArchive()
        return

    def Run(self):
        for _ in range(self.Iteration):
            self.__SelectTheGFunctionSamplingNumbers()
            self.IterationNumber += 1

            print(self.IterationNumber)
            # if (self.Excelwriter.InnerPoint == 0 and 0.4 <= self.Excelwriter.Curvature):
            #     break

        self.Excelwriter.Done()
        self.Excelwriter.Draw()

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

        self.__AppendSolution(ret)
        return

    def __AppendSolution(self, solutions):
        objectiveValue = self.__Estimate(solutions)
        if (objectiveValue > self.SolutionArchive[self.K - 1, self.N]):
            return

        self.SolutionArchive[self.K - 1,
                             self.N] = self.__Estimate(solutions)
        sortedIndices = np.argsort(self.SolutionArchive[:, self.N])
        self.SolutionArchive = np.copy(self.SolutionArchive[sortedIndices])

        for i in range(self.K):
            self.SolutionArchive[i, self.N + 1] = self.__CalOmega(i)
            self.OmegaList[i] = self.SolutionArchive[i, self.N + 1]
        return

    def __Estimate(self, solutions: np.ndarray):
        pntSols = []
        vecSols = []
        for i in range(len(self.GpList)):
            x = self.GpList[i].X() + solutions[i * 3]
            y = self.GpList[i].Y() + solutions[i * 3 + 1]
            z = self.GpList[i].Z() + solutions[i * 3 + 2]

            vx = self.GpVList[i].X() + solutions[i * 3 + 3]
            vy = self.GpVList[i].Y() + solutions[i * 3 + 4]
            vz = self.GpVList[i].Z() + solutions[i * 3 + 5]

            pntSols.append(gp_Pnt(x, y, z))
            vecSols.append(gp_Vec(vx, vy, vz))

        vecSols.insert(0, self.StartVec)
        pntSols.insert(0, self.StartPnt)

        pntSols.append(self.EndPnt)
        vecSols.append(self.EndVec)

        sb = SplineBuilderCurvature(pntSols, vecSols, 5)
        if (sb.SplineShape is None):
            return 1e9
        chk = CollisionChecker(self.Grids, sb.SplineShape)

        objectiveValue = chk.ObjectiveFuntion()

        if (self.IterationNumber % 250 == 0 and self.IterationNumber != 0):
            ssfs = SaveShpaeForSplineTest(
                "out"+str(self.IterationNumber), self.Grids, self.ReturnSbSplineBuilder(), None)
            ssfs.Run()
        innerPoint = chk.ObjectiveFuntion()
        self.Excelwriter.WriteData(
            pntSols, vecSols, innerPoint, 0, objectiveValue + 0)
        return innerPoint + 0

    def ReturnSbShape(self):
        pntSols = []
        vecSols = []
        for i in range(len(self.GpList)):
            x = self.GpList[i].X() + self.SolutionArchive[0, i * 3]
            y = self.GpList[i].Y() + self.SolutionArchive[0, i * 3 + 1]
            z = self.GpList[i].Z() + self.SolutionArchive[0, i * 3 + 2]
            vx = self.GpVList[i].X() + self.SolutionArchive[0, i * 3 + 3]
            vy = self.GpVList[i].Y() + self.SolutionArchive[0, i * 3 + 4]
            vz = self.GpVList[i].Z() + self.SolutionArchive[0, i * 3 + 5]
            pntSols.append(gp_Pnt(x, y, z))
            vecSols.append(gp_Vec(vx, vy, vz))

        pntSols.insert(0, self.StartPnt)
        pntSols.append(self.EndPnt)

        vecSols.insert(0, self.StartVec)
        vecSols.append(self.EndVec)

        sb = SplineBuilderCurvature(pntSols, vecSols, 5)
        ssfs = SaveShpaeForSplineTest("endoutput", self.Grids, sb)
        ssfs.Run()

        return sb.SplineShape

    def ReturnSbSplineBuilder(self):
        pntSols = []
        vecSols = []
        for i in range(len(self.GpList)):
            x = self.GpList[i].X() + self.SolutionArchive[0, i * 3]
            y = self.GpList[i].Y() + self.SolutionArchive[0, i * 3 + 1]
            z = self.GpList[i].Z() + self.SolutionArchive[0, i * 3 + 2]
            vx = self.GpVList[i].X() + self.SolutionArchive[0, i * 3 + 3]
            vy = self.GpVList[i].Y() + self.SolutionArchive[0, i * 3 + 4]
            vz = self.GpVList[i].Z() + self.SolutionArchive[0, i * 3 + 5]
            pntSols.append(gp_Pnt(x, y, z))
            vecSols.append(gp_Vec(vx, vy, vz))

        pntSols.insert(0, self.StartPnt)
        pntSols.append(self.EndPnt)

        vecSols.insert(0, self.StartVec)
        vecSols.append(self.EndVec)

        return SplineBuilderCurvature(pntSols, vecSols, 5)


if (__name__ == "__main__"):
    random.seed(6)
    ast = Astar()
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 20)
    grids.InitRandomBoxObstacle(20)

    N = len(grids.NodeMap) - 1
    startNode, endNode = grids.NodeMap[0][0][0], grids.NodeMap[N][N][N]
    ast.Run(startNode, endNode, grids.NodeMap)
    ast._PostSmoothing(startNode, endNode, grids.NodeMap)
    ivl = InitalVectorList(ast.GetPathPoints())

    sb = SplineBuilderCurvature(ast.GetPathPoints(), ivl.Run(), 5)
    if (sb.SplineShape is None):
        print("VACOR Spline Shape is None")
        exit(0)
    chk = CollisionChecker(grids, sb.SplineShape)

    if (chk.Run()):
        gpList = ast.GetPathPoints()
        ssfs = SaveShpaeForSplineTest("out"+"inital", grids, sb)
        ssfs.Run()
        ivl = InitalVectorList(gpList)
        gpvList = ivl.Run()

        acor = ACOR(50, 1, 0.65, 750, gpList[0], gpList[len(
            gpList) - 1], gpvList[0], gpvList[len(gpList) - 1], gpList, gpvList, chk.ObjectiveFuntion(), grids)
        acor.Run()
        sbshape = acor.ReturnSbShape()

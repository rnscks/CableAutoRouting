import random
import time

from OCC.Core.gp import gp_Pnt
from OCC.Core.TopoDS import TopoDS_Shape, TopoDS_Compound
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from typing import Optional
from BFSforGrids import BFS

from OCC.Display.OCCViewer import Viewer3d
from OCC.Display.SimpleGui import init_display

from RandomBoxGenerator import RandomBoxGenerator


from Node import Node
from Box import Box


class GridsMap(Box):
    def __init__(self, minPoint: gp_Pnt, maxPoint: gp_Pnt, mapSize: int) -> None:
        super().__init__(minPoint, maxPoint)
        self.MapSize: int = mapSize
        nodeMap = self.__InitedNodeMap()
        if (nodeMap is not None):
            self.NodeMap: list[list[list[Node]]] = nodeMap
        pass

    def __FuseForFuseList(self, fusedList: list[Node]):
        fusedShape: Optional[TopoDS_Shape] = None

        for i in range(len(fusedList)):
            if (fusedShape is None):
                fusedShape = fusedList[i].GetBoxShape()
            else:
                fuse = BRepAlgoAPI_Fuse(fusedShape, fusedList[i].GetBoxShape())
                fuse.Build()
                fusedShape = fuse.Shape()

        return fusedShape

    def __BuildFusedShape(self):
        fusedShapeList: list[TopoDS_Shape] = []
        bfs = BFS(self)

        for i in range(self.MapSize):
            for j in range(self.MapSize):
                for k in range(self.MapSize):
                    if (self.NodeMap[i][j][k].Obstacle):
                        fusedList = bfs.Run(i, j, k)
                        if (fusedList is None):
                            continue
                        resultShape = self.__FuseForFuseList(fusedList)
                        if (resultShape is not None):
                            fusedShapeList.append(resultShape)
        return fusedShapeList

    def AddShapeForCompoundBuild(self, compoundBuilder: BRepBuilderAPI_Sewing):
        fusedShapeList:list[TopoDS_Shape] = self.__BuildFusedShape()
        for shape in fusedShapeList:
            compoundBuilder.Add(shape)
        return

    def GetStartNode(self):
        return self.NodeMap[0][0][0]

    def GetEndNode(self):
        return self.NodeMap[len(self.NodeMap) - 1][len(self.NodeMap) - 1][len(self.NodeMap) - 1]

    def CollisionCheck(self, shape: TopoDS_Shape) -> bool:
        from CollisionChecker import CollisionChecker
        if (shape == None):
            return False
        chk = CollisionChecker(self, shape)
        return (chk.Run())

    def VoxelizationForShape(self, shape: TopoDS_Shape) -> None:
        from Voxelizaiton import Voxelization
        vel = Voxelization()
        vel.Run(self, shape)
        return

    def VoxelizationForCompound(self, shape: TopoDS_Compound) -> None:
        from Voxelizaiton import Voxelization
        vel = Voxelization()
        vel.RunForCompound(self, shape)
        pass

    def DisplayGridsMapInObstacle(self, display: Viewer3d, _transparency=0.5, _color="red") -> None:
        for i in range(self.MapSize):
            for j in range(self.MapSize):
                for k in range(self.MapSize):
                    if (self.NodeMap[i][j][k].Obstacle):
                        self.NodeMap[i][j][k].DisplayBoxShape(
                            display, _transparency=_transparency, _color=_color)
        return

    def DisplayGridsMapInAllNode(self, display: Viewer3d, _transparency=0.5, _color="red") -> None:
        for i in range(self.MapSize):
            for j in range(self.MapSize):
                for k in range(self.MapSize):
                    self.NodeMap[i][j][k].DisplayBoxShape(
                        display, _transparency=_transparency, _color=_color)
        return

    def InitRandomBoxObstacleByPercent(self, percent: float):
        for i in range(self.MapSize):
            for j in range(self.MapSize):
                for k in range(self.MapSize):
                    if (i == 0 and j == 0 and k == 0):
                        continue
                    elif (i == self.MapSize - 1 and j == self.MapSize - 1 and k == self.MapSize - 1):
                        continue
                    elif (k == self.MapSize - 1 and j == 0):
                        continue
                    elif (k == self.MapSize - 1 and i == self.MapSize - 1):
                        continue
                    elif (j == 0 and i == 0):
                        continue
                    if (self.__PercentChance(percent)):
                        self.NodeMap[i][j][k].Obstacle = True

    def __PercentChance(self, percent: float) -> bool:
        if random.random() < percent:
            return True
        else:
            return False

    def __InitedNodeMap(self):
        gap: float = (self.MaxPoint.X() - self.MinPoint.X()) / self.MapSize
        nodeMap = [[[None for _ in range(self.MapSize)] for _ in range(
            self.MapSize)] for _ in range(self.MapSize)]

        for i in range(self.MapSize):
            for j in range(self.MapSize):
                for k in range(self.MapSize):
                    minPoint = gp_Pnt((self.MinPoint.X(
                    ) + i*gap), (self.MinPoint.Y() + j*gap), (self.MinPoint.Z() + k*gap))
                    maxPoint = gp_Pnt((self.MinPoint.X(
                    ) + (i + 1)*gap), (self.MinPoint.Y() + (j + 1)*gap), (self.MinPoint.Z() + (k + 1)*gap))
                    nodeMap[i][j][k] = Node(minPoint, maxPoint, i, j, k)
        return nodeMap

    def InitRandomBoxObstacle(self, boxNumber: int = 3) -> None:
        randomBox = RandomBoxGenerator(0, len(self.NodeMap), boxNumber)
        randomBox.Run(self.NodeMap)

    def RecycleNodeMap(self) -> None:
        for i in range(self.MapSize):
            for j in range(self.MapSize):
                for k in range(self.MapSize):
                    self.NodeMap[i][j][k].F = 0.0
                    self.NodeMap[i][j][k].G = 0.0
                    self.NodeMap[i][j][k].Parent = None


if (__name__ == "__main__"):
    a, b, c, d = init_display()
    gridsMap = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 10)
    gridsMap.DisplayGridsMapInAllNode(a, _transparency=1, _color="black")
    a.FitAll()
    b()
    pass

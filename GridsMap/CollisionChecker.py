import util

from OCC.Core.gp import gp_Pnt
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.Geom import Geom_BSplineCurve

from Tessellator.TessellatorShape import TessellatorShape

from Node import Node


class CollisionChecker:
    from GridsMap.GridsMap import GridsMap

    def __init__(self, grids: GridsMap, shape: TopoDS_Shape) -> None:
        from GridsMap.GridsMap import GridsMap

        self.Shape = shape
        self.Grids: GridsMap = grids
        if (shape == None):
            return None
        self.GpPntList: list[gp_Pnt] = TessellatorShape(
            shape).Mesh_.GetMeshPoints()
        pass

    def Run(self) -> bool:
        if (self.Shape == None):
            return True
        gap = (self.Grids.MaxPoint.X() - self.Grids.MinPoint.X()) / \
            self.Grids.MapSize
        for i in range(len(self.GpPntList)):
            g: gp_Pnt = self.GpPntList[i]
            x = int(((g.X() - self.Grids.MinPoint.X()) / gap))
            y = int(((g.Y() - self.Grids.MinPoint.Y()) / gap))
            z = int(((g.Z() - self.Grids.MinPoint.Z()) / gap))
            if (x >= self.Grids.MapSize or y >= self.Grids.MapSize or z >= self.Grids.MapSize or x < 0 or y < 0 or z < 0):
                continue
            if (self.Grids.NodeMap[x][y][z].Obstacle):
                return True
        return False
    
    def ObjectiveFuntion(self) -> int:
        ret: int = 0
        if (self.Shape == None):
            return True
        gap = (self.Grids.MaxPoint.X() - self.Grids.MinPoint.X()) / \
            self.Grids.MapSize
        for i in range(len(self.GpPntList)):
            g: gp_Pnt = self.GpPntList[i]
            x = int(((g.X() - self.Grids.MinPoint.X()) / gap))
            y = int(((g.Y() - self.Grids.MinPoint.Y()) / gap))
            z = int(((g.Z() - self.Grids.MinPoint.Z()) / gap))
            if (x >= self.Grids.MapSize or y >= self.Grids.MapSize or z >= self.Grids.MapSize or x < 0 or y < 0 or z < 0):
                continue
            if (self.Grids.NodeMap[x][y][z].Obstacle):
                # if (e < 0.001):
                #     e = 0.001
                ret += 1

        return ret

    def GetCollisionNode(self) -> list[Node]:
        ret = []
        gap = (self.Grids.MaxPoint.X() - self.Grids.MinPoint.X()) / \
            self.Grids.MapSize
        for i in range(len(self.GpPntList)):
            g: gp_Pnt = self.GpPntList[i]
            x = int(((g.X() - self.Grids.MinPoint.X()) / gap))
            y = int(((g.Y() - self.Grids.MinPoint.Y()) / gap))
            z = int(((g.Z() - self.Grids.MinPoint.Z()) / gap))
            if (x >= self.Grids.MapSize or y >= self.Grids.MapSize or z >= self.Grids.MapSize or x < 0 or y < 0 or z < 0):
                continue

            if (self.Grids.NodeMap[x][y][z].Obstacle and not self.Grids.NodeMap[x][y][z] in ret):
                ret.append(self.Grids.NodeMap[x][y][z])

        return ret

    def GetCollisionNeiNode(self) -> list[Node]:
        ret: list[Node] = []
        colNode = self.GetCollisionNode()

        for index in range(len(colNode)):
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    for dz in range(-2, 3):
                        if (dx == dy == dz):
                            continue
                        x = colNode[index].I + dx
                        y = colNode[index].J + dy
                        z = colNode[index].K + dz

                        appendNode = self.Grids.NodeMap[x][y][z]

                        if (not appendNode in ret):
                            ret.append(appendNode)
        return ret

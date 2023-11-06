import util

from OCC.Core.gp import gp_Pnt
from OCC.Core.TopoDS import TopoDS_Shape, TopoDS_Compound

from Node import Node
from Tessellator.TessellatorShape import TessellatorShape
from Tessellator.TessellatorCompound import TessellatorCompound


class Voxelization:
    from GridsMap.GridsMap import GridsMap

    def __init__(self) -> None:
        pass

    def Run(self, grids: GridsMap, shape: TopoDS_Shape) -> None:
        tlt = TessellatorShape(shape)
        points = tlt.Mesh_.GetMeshPoints()
        gap = (grids.MaxPoint.X() - grids.MinPoint.X()) / grids.MapSize
        for i in range(len(points)):
            g: gp_Pnt = points[i]
            x = int(((g.X() - grids.MinPoint.X()) / gap))
            y = int(((g.Y() - grids.MinPoint.Y()) / gap))
            z = int(((g.Z() - grids.MinPoint.Z()) / gap))
            if (x >= grids.MapSize or y >= grids.MapSize or z >= grids.MapSize or x < 0 or y < 0 or z < 0):
                continue

            grids.NodeMap[x][y][z].Obstacle = True

    def RunForCompound(self, grids: GridsMap, shape: TopoDS_Compound) -> None:
        tlt = TessellatorCompound(shape)
        gap = (grids.MaxPoint.X() - grids.MinPoint.X()) / grids.MapSize
        for mesh in tlt.PartList:
            points = mesh.Mesh_.GetMeshPoints()
            for i in range(len(points)):
                g: gp_Pnt = points[i]
                x = int(((g.X() - grids.MinPoint.X()) / gap))
                y = int(((g.Y() - grids.MinPoint.Y()) / gap))
                z = int(((g.Z() - grids.MinPoint.Z()) / gap))
                if (x >= grids.MapSize or y >= grids.MapSize or z >= grids.MapSize or x < 0 or y < 0 or z < 0):
                    continue

                grids.NodeMap[x][y][z].Obstacle = True
        pass

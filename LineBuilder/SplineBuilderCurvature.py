import util

from typing import Optional

from OCC.Core.gp import gp_Pnt, gp_Vec
from OCC.Core.TopoDS import TopoDS_Edge
import random
from OCC.Core.Geom import Geom_Curve
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.GeomLProp import GeomLProp_CLProps
from GridsMap.GridsMap import GridsMap
from PathFindingAlgorithm.Astar import Astar
from OCC.Display.SimpleGui import init_display
from LineBuilder.PipeBuilder import PipeBuilder
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere
from GridsMap.CollisionChecker import CollisionChecker
from Tessellator.TessellatorShape import TessellatorShape
from SplineBuilderVector import SplineBuilderwithVector, InitalVectorList
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_TOC_RGB,
    Quantity_NOC_WHITE,
    Quantity_NOC_GRAY
)


class SplineBuilderCurvature(SplineBuilderwithVector):
    def __init__(self, gpPntList: list[gp_Pnt], gpVecList: Optional[list[gp_Vec]], diameter: float = 3) -> None:
        super().__init__(gpPntList, gpVecList, diameter)

    def get_curve(self) -> Optional[Geom_Curve]:
        if (self.SplineShape is None):
            return None
        explorer = TopExp_Explorer(self.SplineShape, TopAbs_EDGE)
        while explorer.More():
            edge = explorer.Current()

            if (isinstance(edge, TopoDS_Edge)):
                curve, _, _ = BRep_Tool.Curve(edge)
            else:
                continue

            if curve is not None:
                return curve
            explorer.Next()
        return None

    def compute_curvature_over_spline(self, delta_u: float = 0.001) -> float:
        curve = self.get_curve()
        if not curve:
            raise ValueError("No valid curve found in the spline shape.")

        u_start, u_end = curve.FirstParameter(), curve.LastParameter()

        props = GeomLProp_CLProps(curve, 2, 1e-6)

        u = u_start
        max_curvature = 0
        while u <= u_end:
            props.SetParameter(u)
            max_curvature = max(abs(props.Curvature()), max_curvature)
            u += delta_u

        return max_curvature


if (__name__ == "__main__"):
    a, b, c, d = init_display()
    a.View.SetBackgroundColor(Quantity_TOC_RGB, 0, 0, 0)
    # a.hide_triedron()

    a.View.SetBgGradientColors(
        Quantity_Color(Quantity_NOC_WHITE),
        Quantity_Color(Quantity_NOC_WHITE),
        2,
        True,
    )
    # 6, 10, 12, 14
    random.seed(12)
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(100, 100, 100), 20)
    grids.InitRandomBoxObstacle(20)
    # grids.DisplayGridsMapInObstacle(a, _color="black", _transparency=0)

    ast = Astar()
    ast.Run(grids.GetStartNode(), grids.GetEndNode(), grids.NodeMap)
    ast._PostSmoothing(grids.GetStartNode(), grids.GetEndNode(), grids.NodeMap)

    pl = ast.GetPathPoints()
    ast.DisplayPathNodeListByPipe(
        a, diameter=0.5, _color="black", _transparency=0)
    ivl = InitalVectorList(pl)
    gvl = ivl.Run()
    sbc = SplineBuilderCurvature(pl, gvl, 0.75)
    sbc.DisplaySplineShape(a, _color="red", _transparency=0)
    # if (sbc.SplineShape is not None):
    #     ts = TessellatorShape(sbc.SplineShape)
    #     # ts.Mesh_.DiplayEdgeInMesh(a)
    #     for i in ts.Mesh_.GetMeshPoints():
    #         a.DisplayShape(i)

    print(sbc.compute_curvature_over_spline(1))
    a.FitAll()
    b()

import util

from typing import Optional

from OCC.Core.gp import gp_Pnt, gp_Circ, gp_Dir, gp_Ax2,  gp_Vec
from OCC.Core.TopoDS import TopoDS_Shape, TopoDS_Edge, TopoDS_Wire
from OCC.Core.GeomAPI import GeomAPI_Interpolate
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_Sewing
from OCC.Core.TColStd import TColStd_HArray1OfBoolean
from OCC.Core.TColgp import TColgp_HArray1OfPnt, TColgp_Array1OfVec
from OCC.Display.OCCViewer import Viewer3d
from OCC.Display.SimpleGui import init_display


class SplineBuilder:
    def __init__(self, gpPntList: list[gp_Pnt], diameter: float = 3) -> None:
        self.Diameter = diameter
        self.GpPntList: list[gp_Pnt] = gpPntList
        self.SplineShape: Optional[TopoDS_Shape] = self.__InitedSplineShape(
            diameter)
        pass

    def DisplaySplineShape(self, display: Viewer3d, _transparency=0.5, _color="red") -> None:
        if (self.SplineShape):
            display.DisplayShape(
                self.SplineShape, transparency=_transparency, color=_color)

    def AddShapeForCompoundBuild(self, compoundBuilder: BRepBuilderAPI_Sewing):
        if (self.SplineShape is not None):
            compoundBuilder.Add(self.SplineShape)
        return

    def InitSplineShape(self, gpPntList: list[gp_Pnt], diameter: float) -> None:
        self.GpPntList = gpPntList
        self.SplineShape = self.__InitedSplineShape(diameter)
        return

    def __InitedSplineShape(self, diameter: float) -> Optional[TopoDS_Shape]:
        tColgpPnt = TColgp_HArray1OfPnt(1, len(self.GpPntList))
        tColgpVec = TColgp_Array1OfVec(1, len(self.GpPntList))
        tColstdBool = TColStd_HArray1OfBoolean(1, len(self.GpPntList))
        for i in range(1, len(self.GpPntList) + 1):
            tColgpPnt.SetValue(i, self.GpPntList[i - 1])
            if (len(self.GpPntList) == i):
                continue
            tColgpVec.SetValue(
                i, gp_Vec(self.GpPntList[i - 1], self.GpPntList[i]))
            tColstdBool.SetValue(i, True)

        tColgpVec.SetValue(len(self.GpPntList), gp_Vec(
            self.GpPntList[len(self.GpPntList) - 1], self.GpPntList[len(self.GpPntList) - 2]))

        tColstdBool.SetValue(1, True)
        tColstdBool.SetValue(len(self.GpPntList), False)
        tolenrance = 1e-3
        interpolate = GeomAPI_Interpolate(tColgpPnt, False, tolenrance)

        interpolate.Load(tColgpVec, tColstdBool)
        interpolate.Perform()

        if (interpolate.IsDone()):
            curveEdge = interpolate.Curve()
            edgeShape = BRepBuilderAPI_MakeEdge(curveEdge).Edge()

            wireShape = BRepBuilderAPI_MakeWire(edgeShape).Wire()
            circle: gp_Circ = gp_Circ(gp_Ax2(self.GpPntList[0], gp_Dir(
                self.GpPntList[1].XYZ().Subtracted(self.GpPntList[0].XYZ()))), diameter)
            circleEdge: TopoDS_Edge = BRepBuilderAPI_MakeEdge(circle).Edge()
            circleWire: TopoDS_Wire = BRepBuilderAPI_MakeWire(
                circleEdge).Wire()
            try:
                splineShape = BRepOffsetAPI_MakePipe(
                    wireShape, circleWire).Shape()
                return splineShape

            except RuntimeError:
                return None




if (__name__ == "__main__"):
    pass

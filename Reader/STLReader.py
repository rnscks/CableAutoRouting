from OCC.Core.gp import gp_Pnt
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Display.OCCViewer import Viewer3d
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere

from Movement.ShapeMovement import ShapeMovement

from OCC.Display.SimpleGui import init_display
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_TOC_RGB,
    Quantity_NOC_WHITE
)


class STPReader:
    def __init__(self, fileName: str, move: tuple[float, float, float] = (0, 0, 0), rotate: tuple[float, float, float, float] = (1, 0, 0, 0)) -> None:
        self.FileName: str = fileName
        self.STPShape: TopoDS_Compound = self.__InitStpShape()
        sm = ShapeMovement(self.STPShape, move, rotate)
        pass

    def DisplaySTPShape(self, display: Viewer3d, _transparency: float = 0.5, _color: str = "red") -> None:
        display.DisplayShape(
            self.STPShape, transparency=_transparency, color=_color)
        return

    def __InitStpShape(self) -> TopoDS_Compound:
        stpReader = STEPControl_Reader()
        stpReader.ReadFile(self.FileName)
        stpReader.TransferRoots()
        return stpReader.Shape()

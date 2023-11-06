import util

from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Iterator, TopoDS_Shape
from OCC.Display.SimpleGui import init_display


from Tessellator.TessellatorShape import TessellatorShape

from Reader.STLReader import STPReader
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_TOC_RGB,
    Quantity_NOC_WHITE
)

class TessellatorCompound:
    def __init__(self, compound: TopoDS_Compound) -> None:
        self.Compound: TopoDS_Compound = compound
        self.PartList: list[TessellatorShape] = []
        self.__InitCompound()
        pass
    
    def __InitCompound(self):
        shapeIter: TopoDS_Iterator = TopoDS_Iterator(self.Compound)
        while (shapeIter.More()):
            shape: TopoDS_Shape = shapeIter.Value()
            self.PartList.append(TessellatorShape(shape))
            shapeIter.Next()


if (__name__ == "__main__"):
    a,b,c,d = init_display()
    a.View.SetBackgroundColor(Quantity_TOC_RGB, 0, 0, 0)
    a.hide_triedron()

    a.View.SetBgGradientColors(
        Quantity_Color(Quantity_NOC_WHITE),
        Quantity_Color(Quantity_NOC_WHITE),
        2,
        True,
    )

    stpReader = STPReader("AB6M-M1P-G.stp")
    tlt = TessellatorCompound(stpReader.STPShape)
    # a.DisplayShape(stpReader.STPShape)
    ret = []
    for i in tlt.PartList:
        ret.extend(i.Mesh_.GetMeshPoints())
    for i in ret:
        a.DisplayShape(i)
    # for i in tlt.PartList:
    #     i.Mesh_.DiplayEdgeInMesh(a)
    a.FitAll()
    b()

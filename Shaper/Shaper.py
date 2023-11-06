import util
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeShape

class Shaper:
    def __init__(self, bRepModel : BRepBuilderAPI_MakeShape) -> None:
        self.BRepModel: BRepBuilderAPI_MakeShape = bRepModel
        self.Shape: TopoDS_Shape = self.__InitedShape()
        pass
    def __InitedShape(self) -> TopoDS_Shape:
        n = 0
        while (True):
            n += 1
            if (self.BRepModel.IsDone()):
                shape = self.BRepModel.Shape()
                if (shape != None):
                    return shape
            if (n == 10):
                return None

            
        
    

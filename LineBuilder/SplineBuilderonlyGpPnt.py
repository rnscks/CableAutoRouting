from __future__ import print_function
from OCC.Core.BOPAlgo import BOPAlgo_MakerVolume
from OCC.Core.BRep import BRep_Builder
from OCC.Core.TopTools import TopTools_ListOfShape
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Core.gp import gp_Pln
from OCC.Core.gp import gp_Pnt, gp_Vec
from OCC.Display.SimpleGui import init_display
from OCCUtils import Topo
from OCCUtils.Construct import make_box, make_face
from OCCUtils.Construct import vec_to_dir


from OCC.Core.gp import gp_Pnt, gp_Vec
from OCC.Core.GeomFill import (
    GeomFill_BSplineCurves,
    GeomFill_StretchStyle,
    GeomFill_CoonsStyle,
    GeomFill_CurvedStyle,
)
from OCC.Core.GeomAPI import GeomAPI_PointsToBSpline
from OCC.Core.Geom import Geom_BSplineCurve
from OCC.Extend.ShapeFactory import point_list_to_TColgp_Array1OfPnt
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut



def make_compound(topology_to_compound):
    compound = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound)
    for shp in topology_to_compound:
        builder.Add(compound, shp)

    return compound


def common(shapes):
    mv = BOPAlgo_MakerVolume()
    ls = TopTools_ListOfShape()
    for i in shapes:
        ls.Append(i)
    mv.SetArguments(ls)
    mv.Perform()


    return mv.Shape()


def main():
    display, start_display, _, _ = init_display(size=(600,400))


    a1 = []
    a1.append(gp_Pnt(0, 0, 130))  
    a1.append(gp_Pnt(450,0,250))
    a1.append(gp_Pnt(825,0,300))
    a1.append(gp_Pnt(1250, 0, 90))

    pt_list1 = point_list_to_TColgp_Array1OfPnt(a1)
    SPL1 = GeomAPI_PointsToBSpline(pt_list1).Curve()  

    a2 = []
    a2.append(gp_Pnt(0, 0, 130))
    a2.append(gp_Pnt(0, 250, 130))
    a2.append(gp_Pnt(0, 500, 130))
    pt_list2 = point_list_to_TColgp_Array1OfPnt(a2)
    SPL2 = GeomAPI_PointsToBSpline(pt_list2).Curve()

    aGeomFill1 = GeomFill_BSplineCurves(SPL1, SPL2, GeomFill_CurvedStyle)

    aBSplineSurface1 = aGeomFill1.Surface()

    b1 = []
    b1.append(gp_Pnt(10, 10, 120))  
    b1.append(gp_Pnt(450,0,240))
    b1.append(gp_Pnt(825,0,290))
    b1.append(gp_Pnt(1240, 0, 80))

    pt_listb1 = point_list_to_TColgp_Array1OfPnt(b1)
    SPLb1 = GeomAPI_PointsToBSpline(pt_listb1).Curve()  

    b2 = []
    b2.append(gp_Pnt(10,10, 120))
    b2.append(gp_Pnt(10, 250, 120))
    b2.append(gp_Pnt(10, 500, 120))
    pt_listb2 = point_list_to_TColgp_Array1OfPnt(b2)
    SPLb2 = GeomAPI_PointsToBSpline(pt_listb2).Curve()

    aGeomFill2 = GeomFill_BSplineCurves(SPLb1, SPLb2, GeomFill_CurvedStyle)

    aBSplineSurface2 = aGeomFill2.Surface()


    
    box = make_box(gp_Pnt(0,0,0),1250,500,500)
    fc = make_face(aBSplineSurface1, 1e-6)


    boxb2 = make_box(gp_Pnt(10,15,10),1220,470,500)
    fc2 = make_face(aBSplineSurface2, 1e-6)

    wheelbox = make_box(gp_Pnt(255,200,100),200,100,200)

    #display.DisplayShape(fc, transparency=0.5)
    common_shape = common([box, fc])
    common_shape2 = common([boxb2, fc2])

    solids = Topo(common_shape).solids()
    solids2 = Topo(common_shape2).solids()
    #display.DisplayShape(next(solids), transparency=0.5)
    #display.DisplayShape(next(solids2), transparency=0.5)
    #display.DisplayColoredShape(next(solids), "BLACK")
    shell = BRepAlgoAPI_Cut(next(solids),next(solids2)).Shape()
    shell_wheel = BRepAlgoAPI_Cut(shell,wheelbox).Shape()

    #display.DisplayShape(shell,update=True,transparency=0.5)
    display.DisplayShape(shell_wheel,update=True,transparency=0.5)

    display.FitAll()
    #display.DisplayShape(fc)
    start_display()


if __name__ == "__main__":
    main()

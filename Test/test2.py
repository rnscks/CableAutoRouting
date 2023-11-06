import util
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.IFSelect import IFSelect_RetDone

from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Iterator, TopoDS_Shape
from OCC.Display.SimpleGui import init_display
from OCC.Core.gp import gp_Pnt

from Tessellator.TessellatorShape import TessellatorShape
from GridsMap.GridsMap import GridsMap
from PathFindingAlgorithm.JPSTheta2 import JpsTheta

from LineBuilder.SplineBuilder import SplineBuilder
from Reader.STLReader import STPReader
from CableRouting.Routing import NodeSeacher
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_TOC_RGB,
    Quantity_NOC_WHITE,
    Quantity_NOC_GRAY
)
from Tessellator.TessellatorCompound import TessellatorCompound
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing

num = int(input())
ret_n1 = []
ret_r1 = []
ret_n2 = []
ret_r2 = []
for _ in range(num):
    n1 = list(map(float, input().split()))
    ret_n1.append(n1)
    r1 = list(map(int, input().split()))
    ret_r1.append(r1)
    n2 = list(map(float, input().split()))
    ret_n2.append(n2)
    r2 = list(map(int, input().split()))
    ret_r2.append(r2)

a, b, c, d = init_display()
a.View.SetBackgroundColor(Quantity_TOC_RGB, 0, 0, 0)
# a.hide_triedron()

a.View.SetBgGradientColors(
    Quantity_Color(Quantity_NOC_WHITE),
    Quantity_Color(Quantity_NOC_WHITE),
    2,
    True,
)


gray = Quantity_Color(Quantity_NOC_GRAY)
stp = STPReader("Assem cabinets_simple v9.step")
grids = GridsMap(gp_Pnt(-500, -500, -500), gp_Pnt(500, 500, 500), 60)
grids.VoxelizationForCompound(stp.STPShape)
print((grids.MaxPoint.X() - grids.MinPoint.X()) / grids.MapSize)
# grids.DisplayGridsMapInObstacle(a, _transparency = 1)
a.DisplayShape(stp.STPShape)
print("Input Value")

compound_builder = BRepBuilderAPI_Sewing()
compound_builder.Add(stp.STPShape)
for i in range(len(ret_n2)):
    ns = NodeSeacher()
    for l in range(grids.MapSize):
        for j in range(grids.MapSize):
            for k in range(grids.MapSize):
                grids.NodeMap[l][j][k].Parent = None
                grids.NodeMap[l][j][k].F = 0.0
                grids.NodeMap[l][j][k].H = 0.0
                grids.NodeMap[l][j][k].G = 0.0

    node1 = ns.GetNode(
        gp_Pnt(ret_n1[i][0], ret_n1[i][1], ret_n1[i][2]), ret_r1[i], grids)
    node2 = ns.GetNode(
        gp_Pnt(ret_n2[i][0], ret_n2[i][1], ret_n2[i][2]), ret_r2[i], grids)
    ast = JpsTheta()
    ast.Run(node1, node2, grids.NodeMap)
    ast._PostSmoothing(node1, node2, grids.NodeMap)
    pnts = ast.GetPathPoints()
    print(node1.I, node1.J, node1.K)
    if (pnts):
        pnts.insert(0, gp_Pnt(ret_n2[i][0], ret_n2[i][1], ret_n2[i][2]))
        pnts.append(gp_Pnt(ret_n1[i][0], ret_n1[i][1], ret_n1[i][2]))
        sb = SplineBuilder(pnts, 1)
        compound_builder.Add(sb.SplineShape)
        # grids.VoxelizationForShape(sb.SplineShape)
        sb.DisplaySplineShape(a, _color="red", _transparency=0)
print("Done")
compound_builder.Perform()
compound = compound_builder.SewedShape()

# STP 파일 작성
stp_writer = STEPControl_Writer()
stp_writer.Transfer(compound, STEPControl_AsIs)

stp_file = "output.stp"
status = stp_writer.Write(stp_file)

if status == IFSelect_RetDone:
    print(f"STP 파일이 성공적으로 저장되었습니다: {stp_file}")
else:
    print("STP 파일 저장 중 오류가 발생하였습니다.")
a.FitAll()
b()

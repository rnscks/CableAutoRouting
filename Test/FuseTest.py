from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.gp import gp_Pnt
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs

# 두 개의 형상을 생성 (예: 상자)
box1 = BRepPrimAPI_MakeBox(gp_Pnt(0,0,0), gp_Pnt(1,1,1)).Shape()
box3 = BRepPrimAPI_MakeBox(gp_Pnt(0,0,0), gp_Pnt(1,1,1)).Shape()
box2 = BRepPrimAPI_MakeBox(gp_Pnt(2, 2, 2), gp_Pnt(3, 3, 3)).Shape()

# BRepAlgoAPI_Fuse 인스턴스 생성
fuse = BRepAlgoAPI_Fuse(box1, box2)

# 형상 합치기 수행
fuse.Build()
# 결과 형상 가져오기
result_shape = fuse.Shape()
compound_builder = BRepBuilderAPI_Sewing()
compound_builder.Add(result_shape)
compound_builder.Perform()
compound = compound_builder.SewedShape()


stp_writer = STEPControl_Writer()
stp_writer.Transfer(compound, STEPControl_AsIs)

stp_file = "output.stp"
status = stp_writer.Write(stp_file)

if status:
    print("형상이 성공적으로 저장되었습니다.")
else:
    print("형상 저장에 실패했습니다.")

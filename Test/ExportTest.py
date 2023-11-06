import util

from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing

from Reader.STLReader import STPReader

stp = STPReader("06120269R0.stp")

# Compound 생성
compound_builder = BRepBuilderAPI_Sewing()
compound_builder.Add(stp.STPShape)
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

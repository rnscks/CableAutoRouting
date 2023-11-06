import util
import random

from OCC.Core.gp import gp_Pnt

from GridsMap.GridsMap import GridsMap
from PathFindingAlgorithm.Astar import Astar
from LineBuilder.SplineBuilderVector import SplineBuilderwithVector
from OCC.Core.STEPControl import STEPControl_Writer
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing


class SaveShpaeForSplineTest:
    def __init__(self, fileName: str, girds: GridsMap, sb: SplineBuilderwithVector, compund=None) -> None:
        self.Grids: GridsMap = girds
        self.Sb: SplineBuilderwithVector = sb
        self.Compund = compund
        self.FileName = fileName + ".stp"
        pass

    def Run(self):
        # Compound 생성
        compound_builder = BRepBuilderAPI_Sewing()
        self.Sb.AddShapeForCompoundBuild(compound_builder)
        # self.Grids.AddShapeForCompoundBuild(compound_builder)
        if (self.Compund is not None):
            compound_builder.Add(self.Compund)
        compound_builder.Perform()
        compound = compound_builder.SewedShape()

        # STP 파일 작성
        stp_writer = STEPControl_Writer()
        stp_writer.Transfer(compound, STEPControl_AsIs)

        status = stp_writer.Write(self.FileName)

        if status == IFSelect_RetDone:
            print(f"STP 파일이 성공적으로 저장되었습니다: {self.FileName}")
        else:
            print("STP 파일 저장 중 오류가 발생하였습니다.")


if (__name__ == "__main__"):
    random.seed(45)

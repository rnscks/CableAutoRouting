import util
from Reader.RoutingSTPReader import RoutingSTPReader
from CableRouting.Routing import NodeSeacher
from OCC.Core.gp import gp_Pnt
from GridsMap.GridsMap import GridsMap
from PathFindingAlgorithm.Astar import Astar
from LineBuilder.SplineBuilderCurvature import SplineBuilderCurvature
from LineBuilder.SplineBuilderVector import InitalVectorList
from Expriment.VACOREx import ACOR
from GridsMap.CollisionChecker import CollisionChecker
from Expriment.StoreShape import SaveShpaeForSplineTest


def SplineOp(p1, p2, pd1, pd2, compound, grids):
    ns = NodeSeacher()
    startNode1 = ns.GetNode(
        p1, pd1, grids)
    endNode1 = ns.GetNode(
        p2, pd2, grids)
    ast = Astar()
    ast.Run(startNode1, endNode1, grids.NodeMap)
    ast._PostSmoothing(startNode1, endNode1, grids.NodeMap)
    ivl = InitalVectorList(ast.GetPathPoints())

    sb = SplineBuilderCurvature(ast.GetPathPoints(), ivl.Run(), 5)
    if (sb.SplineShape is None):
        print("VACOR Spline Shape is None")
        exit(0)
    chk = CollisionChecker(grids, sb.SplineShape)

    if (chk.Run()):
        gpList = ast.GetPathPoints()

        gpList.insert(0, p2)
        gpList.append(p1)
        ivl = InitalVectorList(gpList)
        gpvList = ivl.Run()
        sb = SplineBuilderCurvature(gpList, ivl.Run(), 5)
        ssfs = SaveShpaeForSplineTest("out"+"inital", grids, sb, compound)
        ssfs.Run()

        acor = ACOR(50, 1, 0.65, 750, gpList[0], gpList[len(
            gpList) - 1], gpvList[0], gpvList[len(gpList) - 1], gpList, gpvList, chk.ObjectiveFuntion(), grids)
        acor.Run()
        return acor.ReturnSbShape()
    print("?")

    return None


grids = GridsMap(gp_Pnt(-400, -400, -400), gp_Pnt(400, 400, 400), 70)
stpreader = RoutingSTPReader("TARGET2.step", [gp_Pnt(-14.882, 191.673, -110.065), gp_Pnt(
    15.169, 190.926, -110.065), gp_Pnt(36.089, -50.857, 74.852), gp_Pnt(26.639, -50.382, 77.302)], [[0, -1, 0], [0, -1, 0], [0, 0, -1], [0, 0, -1]])
grids.VoxelizationForCompound(stpreader.STPShape)

SplineOp(stpreader.Ports[0], stpreader.Ports[2],
         stpreader.PortDirection[0], stpreader.PortDirection[2], stpreader.STPShape, grids)

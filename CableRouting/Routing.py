import util

from OCC.Core.gp import gp_Pnt
from OCC.Display.SimpleGui import init_display


from Reader.RoutingSTPReader import RoutingSTPReader
from GridsMap.GridsMap import GridsMap
from GridsMap.Node import Node

class NodeSeacher:
    def __init__(self) -> None:
        pass    
    
    def GetNode(self, port: gp_Pnt, direction: list[int], grids: GridsMap) -> Node:
        curPos = self.SearchNode(port ,grids)
        while (grids.NodeMap[curPos[0]][curPos[1]][curPos[2]].Obstacle):
            curPos[0] = direction[0] +curPos[0] 
            curPos[1] = direction[1] +curPos[1] 
            curPos[2] = direction[2] +curPos[2] 
            
        return grids.NodeMap[curPos[0]][curPos[1]][curPos[2]]
    
    def SearchNode(self, port:gp_Pnt, grids: GridsMap) -> list[int]:
        gap = (grids.MaxPoint.X() - grids.MinPoint.X()) / grids.MapSize
        x = int((port.X() - grids.MinPoint.X()) / gap)
        y = int((port.Y() - grids.MinPoint.Y()) / gap)
        z = int((port.Z() - grids.MinPoint.Z()) / gap)
        
        return [x, y, z]
    
    
    
if (__name__ == "__main__"):
    a,b,c,d = init_display()
    grids = GridsMap(gp_Pnt(-100, -100, -100), gp_Pnt(100, 100, 100), 40)
    stp = RoutingSTPReader("AB6M-M1P-G.stp", [gp_Pnt(-4.901, 0, -1.4), gp_Pnt(4.599, 0, -1.4)], [(0,0, -1), (0,0, -1)])
    for i in range(len(stp.Ports)):
        ns = NodeSeacher()
        node = ns.GetNode(stp.Ports[i], stp.PortDirection[i], grids)
        node.DisplayBoxShape(a, _color = "black")
        
    grids.VoxelizationForCompound(stp.STPShape)
    grids.DisplayGridsMapInObstacle(a, _transparency = 1)
    
    a.FitAll()
    b()
    pass
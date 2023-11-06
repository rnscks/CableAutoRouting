import util

import pandas as pd

from OCC.Core.gp import gp_Pnt

from GridsMap.GridsMap import GridsMap
from PathFindingAlgorithm.Astar import Astar
from PathFinding2DAlgorithm.jps import JPSNew
from PathFindingAlgorithm.Theta import Theta
from PathFindingAlgorithm.JPS import Jps

ret = {
    'Resolution': [],
    'Algorithm': [],
    'Time': [],
    'Distance': [],
    'Degree': []
}
n = 10
for j in range(3):
    for i in range(100):
        jps = Jps("Jps")
        ast = Theta("theta")

        grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), n)
        grids.InitRandomBoxObstacle(n)

        grids.RecycleNodeMap()
        jps.Run(grids.NodeMap[0][0][0], grids.NodeMap[grids.MapSize - 1]
                [grids.MapSize - 1][grids.MapSize - 1], grids.NodeMap)
        grids.RecycleNodeMap()
        ast.Run(grids.NodeMap[0][0][0], grids.NodeMap[grids.MapSize - 1]
                [grids.MapSize - 1][grids.MapSize - 1], grids.NodeMap)

        ret['Algorithm'].append(jps.Name)
        ret['Resolution'].append(grids.MapSize)
        ret['Time'].append(jps.CalTime)
        ret['Distance'].append(jps.Distance)
        ret['Degree'].append(jps.Degree)

        ret['Algorithm'].append(ast.Name)
        ret['Resolution'].append(grids.MapSize)
        ret['Time'].append(ast.CalTime)
        ret['Distance'].append(ast.Distance)
        ret['Degree'].append(ast.Degree)

        print(n)
    n += 10
df = pd.DataFrame(ret)


excel_file = 'Ex.xlsx'
df.to_excel(excel_file, index=False)
print(f"{excel_file} 파일이 생성되었습니다.")

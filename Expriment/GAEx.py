import util

import numpy as np
import random
import numpy as np
from geneticalgorithm import geneticalgorithm as ga

from GridsMap.GridsMap import GridsMap
from GridsMap.CollisionChecker import CollisionChecker
from PathFindingAlgorithm.Astar import Astar
from LineBuilder.SplineBuilder import SplineBuilder
from StoreShape import SaveShpaeForSplineTest
import pandas as pd
import matplotlib.pyplot as plt

from OCC.Core.gp import gp_Pnt
import numpy as np


    
class GAEx:
    def __init__(self, start: gp_Pnt, end: gp_Pnt,gpList: list[gp_Pnt], grids:GridsMap) -> None:
        self.GpList:list[gp_Pnt] = gpList
        self.StartPnt:gp_Pnt = start
        self.EndPnt: gp_Pnt = end
        self.Grids:GridsMap = grids
        self.Excelwriter: ACORTestExcelWriter = ACORTestExcelWriter(gpList)
        self.IterationNumber = 0
        self.EndSolutions = None
        return
    
    def __Estimate(self, solutions: np.ndarray):
        self.IterationNumber += 1
        sols = []
        for i in range(len(self.GpList)):
            x = self.GpList[i].X() + solutions[i * 3]
            y = self.GpList[i].Y() + solutions[i * 3 + 1]
            z = self.GpList[i].Z() + solutions[i * 3 + 2]
            sols.append(gp_Pnt(x, y, z)) 
        
        sols.insert(0, self.StartPnt)
        sols.append(self.EndPnt)
        
        sb = SplineBuilder(sols, 5)
        chk = CollisionChecker(self.Grids, sb.SplineShape)
        objectiveValue = chk.ObjectiveFuntion()
        self.Excelwriter.WriteData(sols, objectiveValue)
        
        if (self.IterationNumber % 250 == 0 and self.IterationNumber != 0):
            ssfs = SaveShpaeForSplineTest("out"+str(self.IterationNumber), self.Grids, sb)
            ssfs.Run()

        return chk.ObjectiveFuntion()
        
    def Run(self):
        algorithm_param = {'max_num_iteration': 100,\
                        'population_size':5,\
                        'mutation_probability':0.1,\
                        'elit_ratio': 0.01,\
                        'crossover_probability': 0.5,\
                        'parents_portion': 0.3,\
                        'crossover_type':'uniform',\
                        'max_iteration_without_improv':True}

        varbound=np.array([[-50,50]]*(len(self.GpList) * 3))

        model=ga(function=self.__Estimate, dimension=len(self.GpList) * 3,variable_type='real',variable_boundaries=varbound, algorithm_parameters= algorithm_param, progress_bar=False)

        model.run()
        self.EndSolutions = model.best_variable
        self.Excelwriter.Done()
        self.Excelwriter.Draw()
        
        return model.best_variable
    
    def ReturnSbShape(self):
        sols = []
        for i in range(len(self.GpList)):
            x = self.GpList[i].X() + self.EndSolutions[i * 3]
            y = self.GpList[i].Y() + self.EndSolutions[i * 3 + 1]
            z = self.GpList[i].Z() + self.EndSolutions[i * 3 + 2]
            sols.append(gp_Pnt(x, y, z)) 

        sols.insert(0, self.StartPnt)
        sols.append(self.EndPnt)
        
        sb = SplineBuilder(sols, 5)
        ssfs = SaveShpaeForSplineTest("endoutput",self.Grids,sb)
        ssfs.Run()

        return sb.SplineShape
    
class ACORTestExcelWriter:
    def __init__(self, wayPnt: list[gp_Pnt]) -> None:
        self.data:dict = {
            'globalValue': [],
            'localValue': [],
        }
        self.SizeofWapPnt = len(wayPnt)
        self.globalValue = 1e9

        for i in range(len(wayPnt)):
            self.data[str(i) + "waypnt" + "_x"] = []
            self.data[str(i) + "waypnt" + "_y"] = []
            self.data[str(i) + "waypnt"+ "_z"] = []
        return
    
    def WriteData(self, solution: list[gp_Pnt], objectiveValue: int):
        for i in range(1, len(solution) - 1):
            self.data[str(i - 1) + "waypnt" + "_x"].append(solution[i].X())
            self.data[str(i - 1) + "waypnt" + "_y"].append(solution[i].Y())
            self.data[str(i - 1) + "waypnt" + "_z"].append(solution[i].Z())

        self.data['localValue'].append(objectiveValue)
        if (self.globalValue > objectiveValue):
            self.globalValue = objectiveValue
            
        self.data['globalValue'].append(self.globalValue)
        return
    
    def Done(self):
        self.df = pd.DataFrame(self.data)
        self.df.to_excel("result.xlsx")
        self.__InitTitleById()
        return
    
    def __InitTitleById(self):
        self.IDtoTitle = dict()
        for i in range(self.SizeofWapPnt):
            self.IDtoTitle[str(i) + "waypnt" + "_x"] = "Movent of Way Point " +str(i) + ", X"
            self.IDtoTitle[str(i) + "waypnt" + "_y"] = "Movent of Way Point " +str(i) + ", Y"
            self.IDtoTitle[str(i) + "waypnt" + "_z"] = "Movent of Way Point " +str(i) + ", Z"
        self.IDtoTitle["localValue"] = "Local Optimization Value"
        self.IDtoTitle["globalValue"] = "Global Optimization Value"
            
    def __IsOptimizationValue(self, name):
        return (not(name[-1] == 'x' or name[-1] == 'y' or name[-1] == 'z'))
        
    
    def __WhatKindofCoordnate(self, name):
        if (name[-1] == 'x'):
            return 'X'
        elif (name[-1] == 'y'):
            return "Y"
        elif (name[-1] == 'z'):
            return "Z"
        return " " 
        

    def __DrawFigure(self, name):
        x = []
        for i in range(len(self.df[name])):
            x.append(i + 1)
        plt.figure(figsize=(12, 9))
        
        plt.rcParams['font.size'] = 25 
        plt.title(self.IDtoTitle[name], fontname ="serif").set_weight('bold')
        plt.xlabel("Iteration", fontname ="serif").set_weight('bold')
        if (self.__IsOptimizationValue(name)):
            plt.ylabel("Optimization Value", fontname ="serif").set_weight('bold')
            plt.plot(x, self.df[name], color ='k')  # 선 그래프를 그립니다.
        else:
            plt.ylabel(self.__WhatKindofCoordnate(name), fontname ="serif").set_weight('bold')
            if(self.__WhatKindofCoordnate(name) == 'X'):
                plt.plot(x, self.df[name], color ='red')  # 선 그래프를 그립니다.
            elif (self.__WhatKindofCoordnate(name) == 'Y'):
                plt.plot(x, self.df[name], color ='green')  # 선 그래프를 그립니다.
            elif (self.__WhatKindofCoordnate(name) == 'Z'):
                plt.plot(x, self.df[name], color ='blue')  # 선 그래프를 그립니다.
            
        plt.savefig(name + ".png")
        
    def Draw(self):
        for key in self.data.keys():
            self.__DrawFigure(key)

            


if (__name__ == "__main__"):
    random.seed(211)
    ast = Astar()
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 20)
    grids.InitRandomBoxObstacle(20)

    N = len(grids.NodeMap) - 1
    startNode, endNode = grids.NodeMap[0][0][0], grids.NodeMap[N][N][N]
    ast.Run(startNode, endNode, grids.NodeMap)
    ast._PostSmoothing(startNode, endNode, grids.NodeMap)

    sb = SplineBuilder(ast.GetPathPoints(), 5)
    chk = CollisionChecker(grids, sb.SplineShape)

    if (chk.Run()):
        gpList = ast.GetPathPoints()
        ssfs = SaveShpaeForSplineTest("out"+"inital", grids, sb)
        ssfs.Run()
        acor = GAEx( gpList[0], gpList[len(gpList) - 1], gpList, grids)
        acor.Run()
        sbshape = acor.ReturnSbShape()
        

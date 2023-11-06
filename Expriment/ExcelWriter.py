from OCC.Core.gp import gp_Pnt, gp_Vec

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


class ACORTestExcelWriter:
    def __init__(self, wayPnt: list[gp_Pnt]) -> None:
        self.data: dict = {
            'globalValue': [],
            'localValue': [],
            'iteration': [],
            'curvature': [],
            'innerPoint': []
        }
        self.SizeofWapPnt = len(wayPnt)
        self.globalValue = 1e9
        self.InnerPoint = 1e9
        self.Curvature = 1e9

        for i in range(len(wayPnt)):
            self.data[str(i) + "waypnt" + "_x"] = []
            self.data[str(i) + "waypnt" + "_y"] = []
            self.data[str(i) + "waypnt" + "_z"] = []
            self.data[str(i) + "waypnt" + "_vx"] = []
            self.data[str(i) + "waypnt" + "_vy"] = []
            self.data[str(i) + "waypnt" + "_vz"] = []
        return

    def WriteData(self, pntSolution: list[gp_Pnt], vecSolution: list[gp_Vec], innerPoint: int, curvature: float, objectiveValue: float):
        for i in range(1, len(pntSolution) - 1):
            self.data[str(i - 1) + "waypnt" + "_x"].append(pntSolution[i].X())
            self.data[str(i - 1) + "waypnt" + "_y"].append(pntSolution[i].Y())
            self.data[str(i - 1) + "waypnt" + "_z"].append(pntSolution[i].Z())

        for i in range(1, len(vecSolution) - 1):
            self.data[str(i - 1) + "waypnt" + "_vx"].append(vecSolution[i].X())
            self.data[str(i - 1) + "waypnt" + "_vy"].append(vecSolution[i].Y())
            self.data[str(i - 1) + "waypnt" + "_vz"].append(vecSolution[i].Z())

        self.data['localValue'].append(objectiveValue)
        if (self.globalValue > objectiveValue):
            self.globalValue = objectiveValue

        self.InnerPoint = innerPoint
        self.Curvature = curvature
        self.data['globalValue'].append(self.globalValue)
        self.data['innerPoint'].append(innerPoint)
        self.data['curvature'].append(curvature)
        self.data['iteration'].append(len(self.data['globalValue']))
        return

    def Done(self):
        self.df = pd.DataFrame(self.data)
        self.df.to_excel("result.xlsx")
        self.__InitTitleById()
        return

    def __InitTitleById(self):
        self.IDtoTitle = dict()
        for i in range(self.SizeofWapPnt):
            self.IDtoTitle[str(i) + "waypnt" +
                           "_x"] = "Interpolation Point " + str(i) + ", X"
            self.IDtoTitle[str(i) + "waypnt" +
                           "_y"] = "Interpolation Point " + str(i) + ", Y"
            self.IDtoTitle[str(i) + "waypnt" +
                           "_z"] = "Interpolation Point " + str(i) + ", Z"
            self.IDtoTitle[str(i) + "waypnt" +
                           "_vx"] = "Gradient Vector " + str(i) + ", X"
            self.IDtoTitle[str(i) + "waypnt" +
                           "_vy"] = "Gradient Vector " + str(i) + ", Y"
            self.IDtoTitle[str(i) + "waypnt" +
                           "_vz"] = "Gradient Vector " + str(i) + ", Z"
        self.IDtoTitle["localValue"] = "Local Optimization Value"
        self.IDtoTitle["globalValue"] = "Global Optimization Value"
        self.IDtoTitle["curvature"] = "Curvature"
        self.IDtoTitle["innerPoint"] = "Interior Point"

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
        plt.figure(figsize=(16, 11))

        plt.title(self.IDtoTitle[name], fontname="serif").set_weight(  # type: ignore
            'bold')  # type: ignore

        if (name == "globalValue" or name == "localValue"):

            sns.lineplot(x="iteration", y=name,
                         data=self.df, color="red")
            plt.xlabel("Trial", fontname="serif").set_weight('bold')
            plt.ylabel("Optimization Value",
                       fontname="serif").set_weight('bold')
        elif (name == 'curvature'):
            # sns.lineplot(x="iteration", y=name,
            #              data=self.df, color="black")

            sns.regplot(x="iteration", y=name,
                        data=self.df, ci=90, color="black", scatter=True)
            plt.xlabel("Trial", fontname="serif").set_weight('bold')
            plt.ylabel("Max Curvature",
                       fontname="serif").set_weight('bold')
        elif (name == 'innerPoint'):
            sns.lineplot(x="iteration", y=name,
                         data=self.df, color="black")
            plt.xlabel("Trial", fontname="serif").set_weight('bold')
            plt.ylabel("Interior point",
                       fontname="serif").set_weight('bold')
        else:
            if (self.__WhatKindofCoordnate(name) == 'X'):
                sns.regplot(x="iteration", y=name,
                            data=self.df, ci=90, color="red", scatter=False)
                plt.xlabel("Trial", fontname="serif").set_weight('bold')
                plt.ylabel("X",
                           fontname="serif").set_weight('bold')

            elif (self.__WhatKindofCoordnate(name) == 'Y'):
                sns.regplot(x="iteration", y=name,
                            data=self.df, ci=90, color="blue", scatter=False)
                plt.xlabel("Trial", fontname="serif").set_weight('bold')
                plt.ylabel("Y",
                           fontname="serif").set_weight('bold')

            elif (self.__WhatKindofCoordnate(name) == 'Z'):
                sns.regplot(x="iteration", y=name,
                            data=self.df, ci=90, color="green", scatter=False)
                plt.xlabel("Trial", fontname="serif").set_weight('bold')
                plt.ylabel("Z",
                           fontname="serif").set_weight('bold')

        plt.savefig(name + ".png")
        plt.close()

    def Draw(self):
        sns.set(style="whitegrid", font_scale=3.5)

        # sns.set_palette("husl")
        for key in self.data.keys():
            if (key == "iteration"):
                continue
            self.__DrawFigure(key)

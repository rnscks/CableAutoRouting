import util

from OCC.Core.gp import gp_Pnt, gp_Vec
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Display.OCCViewer import Viewer3d

import math as m
import numpy as np
import random

from Shaper.Shaper import Shaper


class Mesh:
    def __init__(self) -> None:
        self.Vertices: list[list[gp_Pnt]] = []
        self.FaceIndexs = []
        self.TriArea: list[float] = []
        self.SamplingPoints = []
        pass
    
    def DiplayEdgeInMesh(self, display:Viewer3d) -> None:
        for i in range(len(self.FaceIndexs)):
            for face in self.FaceIndexs[i]:
                vertices = self.Vertices[i]
                edge1 = BRepBuilderAPI_MakeEdge(vertices[face[0] - 1], vertices[face[1] - 1])
                edge2 = BRepBuilderAPI_MakeEdge(vertices[face[0] - 1], vertices[face[2] - 1])
                edge3 = BRepBuilderAPI_MakeEdge(vertices[face[1] - 1], vertices[face[2] - 1])
                
                edgeShaper1 = Shaper(edge1)
                edgeShaper2 = Shaper(edge2)
                edgeShaper3 = Shaper(edge3)
                display.DisplayShape(edgeShaper1.Shape)
                display.DisplayShape(edgeShaper2.Shape)
                display.DisplayShape(edgeShaper3.Shape)
        return
                
    def GetMeshPoints(self):
        points = []
        for vertex in self.Vertices:
            if (vertex == None):
                continue
            for i in vertex:
                if (i == None):
                    continue
                points.append(i)
                
        for i in self.SamplingPoints:
            points.append(i)

        return points

    def AddVertexPnt(self, vertices: list[gp_Pnt]):
        self.Vertices.append(vertices)

    def AddFaceIndex(self, index):
        self.FaceIndexs.append(index)

    def __InitTriangleArea(self):
        tmp = []
        for i in range(len(self.FaceIndexs)):
            for index in self.FaceIndexs[i]:
                v1 = gp_Vec(self.Vertices[i][index[0] - 1],
                            self.Vertices[i][index[1] - 1])
                v2 = gp_Vec(self.Vertices[i][index[0] - 1],
                            self.Vertices[i][index[2] - 1])
                area = (0.5) * (v1.CrossMagnitude(v2))
                self.TriArea.append(area)

    def RandomSampling(self, num):
        self.__InitTriangleArea()
        self.TriArea = np.asarray(self.TriArea)

        cumSumArea = np.cumsum(self.TriArea)
        totalArea = cumSumArea[-1]

        triList = []
        verList = []

        for i in range(len(self.FaceIndexs)):
            b = []
            for index in self.FaceIndexs[i]:
                triList.append(list(index))
                b.append(self.Vertices[i][index[0] - 1])
                b.append(self.Vertices[i][index[1] - 1])
                b.append(self.Vertices[i][index[2] - 1])
                verList.append(b)
                b = []

        cnt = [0 for _ in range(len(triList))]

        for i in range(num):
            randomArea = random.uniform(0, totalArea)

            triIndex = np.searchsorted(cumSumArea, randomArea)

            cnt[triIndex] += 1
            A = gp_Vec(verList[triIndex][0].XYZ())
            B = gp_Vec(verList[triIndex][1].XYZ())
            C = gp_Vec(verList[triIndex][2].XYZ())

            r1 = random.random()
            r2 = random.random()

            A.Multiply(1 - m.sqrt(r1))
            B.Multiply(m.sqrt(r1) * (1 - r2))
            C.Multiply((m.sqrt(r1)) * r2)

            pnt = A + B + C
            pnt = gp_Pnt(pnt.XYZ())
            self.SamplingPoints.append(pnt)

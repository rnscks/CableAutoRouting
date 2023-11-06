

import numpy as np
from typing import Optional
from GridsMap.Node import Node


class GraphNode:
    def __init__(self, node: Node) -> None:
        self.GridsNode = node
        self.Neihgbors: list['GraphNode'] = []
        self.Pheromone: Optional[np.ndarray] = None
        self.Distance: dict['GraphNode', float] = {}
        self.Degree: dict['GraphNode', float] = {}
        self.Index: dict['GraphNode', int] = {}
        self.DistanceTable: Optional[np.ndarray] = None
        self.DegreeTable: Optional[np.ndarray] = None
        pass

    def AddNeihgborNode(self, nei: 'GraphNode') -> None:
        self.Index[nei] = len(self.Neihgbors)
        self.Neihgbors.append(nei)

    def AddNeihgborDistance(self, nei: 'GraphNode', distance: float) -> None:
        self.Distance[nei] = distance

    def AddNeihgborDegree(self, nei: 'GraphNode', degree: float) -> None:
        self.Degree[nei] = degree

    def __eq__(self, other: 'GraphNode') -> bool:
        if (other is not None):
            return self.GridsNode == other.GridsNode
        return False

    def __hash__(self) -> int:
        return super().__hash__()

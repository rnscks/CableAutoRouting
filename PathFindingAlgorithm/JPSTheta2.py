import heapq
from time import time

from Agent import Agent
from GridsMap.Node import Node

import heapq
import random
from time import time
from OCC.Core.gp import gp_Pnt

from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import Viewer3d
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_TOC_RGB,
    Quantity_NOC_WHITE
)

from Agent import Agent
from GridsMap.Node import Node
from GridsMap.GridsMap import GridsMap
from LineBuilder.SplineBuilder import SplineBuilder

IsEnd: bool = False
openList: list[Node] = []
closedList: set[Node] = set()
nodeMap = None


def _LineOfSight3D(src: Node, dst: Node, nodeMap: list[list[list[Node]]]) -> bool:
    if (src is None or src.Parent is None or src == src.Parent.Parent):
        return False
    x1, y1, z1 = src.I, src.J, src.K

    x2, y2, z2 = dst.I, dst.J, dst.K

    if (x2 > x1):
        xs = 1
        dx = x2 - x1
    else:
        xs = -1
        dx = x1 - x2

    if (y2 > y1):
        ys = 1
        dy = y2 - y1
    else:
        ys = -1
        dy = y1 - y2

    if (z2 > z1):
        zs = 1
        dz = z2 - z1
    else:
        zs = -1
        dz = z1 - z2

    if (dx >= dy and dx >= dz):
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        while (x1 != x2):
            x1 += xs
            if (p1 >= 0):
                y1 += ys
                p1 -= 2 * dx
            if (p2 >= 0):
                z1 += zs
                p2 -= 2 * dx
            p1 += 2 * dy
            p2 += 2 * dz
            if (nodeMap[x1][y1][z1].Obstacle):
                return False

    elif (dy >= dx and dy >= dz):
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        while (y1 != y2):
            y1 += ys
            if (p1 >= 0):
                x1 += xs
                p1 -= 2 * dy
            if (p2 >= 0):
                z1 += zs
                p2 -= 2 * dy
            p1 += 2 * dx
            p2 += 2 * dz
            if (nodeMap[x1][y1][z1].Obstacle):
                return False
    else:
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        while (z1 != z2):
            z1 += zs
            if (p1 >= 0):
                y1 += ys
                p1 -= 2 * dz
            if (p2 >= 0):
                x1 += xs
                p2 -= 2 * dz
            p1 += 2 * dy
            p2 += 2 * dx
            if (nodeMap[x1][y1][z1].Obstacle):
                return False
    return True


def SearchJumpPointDiagonal3D(src: Node, dx: int, dy: int, dz: int, dst: Node, nodeMap: list[list[list[Node]]]):
    global IsEnd
    if (src in closedList or IsEnd is True):
        return True
    x, y, z = src.I, src.J, src.K
    n = len(nodeMap)
    if (dz == 1):
        for i in range(2):
            if ((not _IsOutOfRange(x - dx, y + dy, z + i + 1, n)) and
                    nodeMap[x - dx][y + dy][z + i].Obstacle and
                    (not nodeMap[x - dx][y + dy][z + i + 1].Obstacle)):
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                src.G = _CalEuclidDistance(
                    src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True

            if ((not _IsOutOfRange(x + dx, y - dy, z + i + 1, n)) and
                    nodeMap[x + dx][y - dy][z + i + 1].Obstacle and
                    (not nodeMap[x + dx][y - dy][z + i + 1].Obstacle)):
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                    src.G = _CalEuclidDistance(src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True

        ExploreOrthogonal(src, dx, 0, 0, dst,  nodeMap)
        ExploreOrthogonal(src, 0, dy, 0, dst,  nodeMap)
        ExploreOrthogonal(src, 0, 0, dz, dst,  nodeMap)
        ExploreDiagonal2D(src, dx, dy, 0, dst, nodeMap)
        ExploreDiagonal2D(src, 0, dy, dz, dst,  nodeMap)
        ExploreDiagonal2D(src, dx, 0, dz, dst,  nodeMap)

        return False

    if (dz == -1):
        for i in range(0, -2, -1):
            if ((not _IsOutOfRange(x - dx, y + dy, z + i - 1, n)) and
                    nodeMap[x - dx][y + dy][z + i].Obstacle and
                    (not nodeMap[x - dx][y + dy][z + i - 1].Obstacle)):
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                    src.G = _CalEuclidDistance(
                        src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True

            if ((not _IsOutOfRange(x + dx, y - dy, z + i - 1, n)) and
                    nodeMap[x + dx][y - dy][z + i].Obstacle and
                    (not nodeMap[x + dx][y - dy][z + i - 1].Obstacle)):
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                    src.G = _CalEuclidDistance(
                        src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True

        ExploreOrthogonal(src, dx, 0, 0, dst,  nodeMap)
        ExploreOrthogonal(src, 0, dy, 0, dst,  nodeMap)
        ExploreOrthogonal(src, 0, 0, dz, dst,  nodeMap)
        ExploreDiagonal2D(src, dx, dy, 0, dst, nodeMap)
        ExploreDiagonal2D(src, 0, dy, dz, dst, nodeMap)
        ExploreDiagonal2D(src, dx, 0, dz, dst, nodeMap)

        return False


def SearchJumpPointDiagonal2D(src: Node, dx: int, dy: int, dz: int, dst: Node, nodeMap: list[list[list[Node]]]):
    global IsEnd
    if (src in closedList or IsEnd is True):
        return True
    x, y, z = src.I, src.J, src.K
    n = len(nodeMap)

    if (dz == 0):
        if (not _IsOutOfRange(x + dx, y, z, n) and nodeMap[x + dx][y][z].Obstacle):
            for i in range(-1, 2):
                if (i == 0):
                    continue
                if (not _IsOutOfRange(x + dx, y, z + i, n) and not nodeMap[x + dx][y][z + i].Obstacle):
                    while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                        if (src.Parent == src.Parent.Parent or src.Parent == src):
                            break
                        src.Parent = src.Parent.Parent
                        src.G = _CalEuclidDistance(
                            src.Parent, src) + src.Parent.G
                    src.F = _CalEuclidDistance(src, dst) + src.G
                    if (src in openList):
                        openList.remove(src)
                    heapq.heappush(openList, src)
                    return True

        if (not _IsOutOfRange(x, y + dy, z, n) and nodeMap[x][y + dy][z].Obstacle):
            for i in range(-1, 2):
                if (i == 0):
                    continue
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                    src.G = _CalEuclidDistance(
                        src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True
        ExploreOrthogonal(src, dx, 0, 0, dst, nodeMap)
        ExploreOrthogonal(src, 0, dy, 0, dst, nodeMap)

        return False

    if (dx == 0):
        if (not _IsOutOfRange(x, y, z + dz, n) and nodeMap[x][y][z + dz].Obstacle):
            for i in range(-1, 2):
                if (i == 0):
                    continue
                if (not _IsOutOfRange(x + i, y, z + dz, n) and not nodeMap[x + i][y][z + dz].Obstacle):
                    while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                        if (src.Parent == src.Parent.Parent or src.Parent == src):
                            break
                        src.Parent = src.Parent.Parent
                        src.G = _CalEuclidDistance(
                            src.Parent, src) + src.Parent.G
                    src.F = _CalEuclidDistance(src, dst) + src.G
                    if (src in openList):
                        openList.remove(src)
                    heapq.heappush(openList, src)
                    return True

        if (not _IsOutOfRange(x, y + dy, z, n) and nodeMap[x][y + dy][z].Obstacle):
            for i in range(-1, 2):
                if (i == 0):
                    continue
                if (not _IsOutOfRange(x + i, y + dy, z, n) and not nodeMap[x + i][y + dy][z].Obstacle):
                    while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                        if (src.Parent == src.Parent.Parent or src.Parent == src):
                            break
                        src.Parent = src.Parent.Parent
                        src.G = _CalEuclidDistance(
                            src.Parent, src) + src.Parent.G
                    src.F = _CalEuclidDistance(src, dst) + src.G
                    if (src in openList):
                        openList.remove(src)
                    heapq.heappush(openList, src)
                    return True
        ExploreOrthogonal(src, 0, 0, dz, dst, nodeMap)
        ExploreOrthogonal(src, 0, dy, 0, dst, nodeMap)
        return False

    if (dy == 0):
        if (not _IsOutOfRange(x, y, z + dz, n) and nodeMap[x][y][z + dz].Obstacle):
            for i in range(-1, 2):
                if (i == 0):
                    continue
                if (not _IsOutOfRange(x, y + i, z + dz, n) and not nodeMap[x][y + i][z + dz].Obstacle):
                    while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                        if (src.Parent == src.Parent.Parent or src.Parent == src):
                            break
                        src.Parent = src.Parent.Parent
                        src.G = _CalEuclidDistance(
                            src.Parent, src) + src.Parent.G
                    src.F = _CalEuclidDistance(src, dst) + src.G
                    if (src in openList):
                        openList.remove(src)
                    heapq.heappush(openList, src)
                    return True
        if (not _IsOutOfRange(x + dx, y, z, n) and nodeMap[x + dx][y][z].Obstacle):
            for i in range(-1, 2):
                if (i == 0):
                    continue
                if (not _IsOutOfRange(x + dx, y + i, z + dz, n) and not nodeMap[x + dx][y + i][z].Obstacle):
                    while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                        if (src.Parent == src.Parent.Parent or src.Parent == src):
                            break
                        src.Parent = src.Parent.Parent
                        src.G = _CalEuclidDistance(
                            src.Parent, src) + src.Parent.G
                    src.F = _CalEuclidDistance(src, dst) + src.G
                    if (src in openList):
                        openList.remove(src)
                    heapq.heappush(openList, src)
                    return True
        ExploreOrthogonal(src, 0, 0, dz, dst, nodeMap)
        ExploreOrthogonal(src, dx, 0, 0, dst, nodeMap)

        return False


def SearchJumpPointOrthogonal(src: Node, dx: int, dy: int, dz: int, dst: Node, nodeMap: list[list[list[Node]]]):
    global IsEnd
    if (src in closedList or IsEnd is True):
        return True
    x, y, z = src.I, src.J, src.K
    n = len(nodeMap)

    if (dx):
        for i in range(-1, 2):
            if ((not _IsOutOfRange(x + dx, y + 1, z + i, n)) and
                    nodeMap[x][y + 1][z + i].Obstacle and
                    (not nodeMap[x + dx][y + 1][z + i].Obstacle)):
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                    src.G = _CalEuclidDistance(
                        src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True
            if ((not _IsOutOfRange(x + dx, y - 1, z + i, n)) and
                    nodeMap[x][y - 1][z + i].Obstacle and
                    (not nodeMap[x + dx][y - 1][z + i].Obstacle)):
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                    src.G = _CalEuclidDistance(
                        src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True
        return False

    if (dy):
        for i in range(-1, 2):
            if ((not _IsOutOfRange(x + 1, y + dy, z + i, n)) and
                    nodeMap[x + 1][y][z + i].Obstacle and
                    (not nodeMap[x + 1][y + dy][z + i].Obstacle)):
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                    src.G = _CalEuclidDistance(
                        src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True
            if ((not _IsOutOfRange(x - 1, y + dy, z + i, n)) and
                    nodeMap[x - 1][y][z + i].Obstacle and
                    (not nodeMap[x - 1][y + dy][z + i].Obstacle)):
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                    src.G = _CalEuclidDistance(
                        src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True
        return False

    if (dz):
        for i in range(-1, 2):
            if ((not _IsOutOfRange(x + 1, y + i, z + dz, n)) and
                    nodeMap[x + 1][y + i][z].Obstacle and
                    (not nodeMap[x + 1][y + i][z + dz].Obstacle)):
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                    src.G = _CalEuclidDistance(
                        src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True
            if ((not _IsOutOfRange(x - 1, y + i, z + dz, n)) and
                    nodeMap[x - 1][y + i][z].Obstacle and
                    (not nodeMap[x - 1][y + i][z + dz].Obstacle)):
                while (_LineOfSight3D(src.Parent.Parent, src, nodeMap)):
                    if (src.Parent == src.Parent.Parent or src.Parent.Parent == src):
                        break
                    src.Parent = src.Parent.Parent
                    src.G = _CalEuclidDistance(
                        src.Parent, src) + src.Parent.G
                src.F = _CalEuclidDistance(src, dst) + src.G
                if (src in openList):
                    openList.remove(src)
                heapq.heappush(openList, src)
                return True
        return False


def _IsOutOfRange(i: int, j: int, k: int, mapSize: int) -> bool:
    return i >= mapSize or j >= mapSize or k >= mapSize or i < 0 or j < 0 or k < 0


def _CalEuclidDistance(src: Node, dst: Node) -> float:
    return src.CenterPoint.Distance(dst.CenterPoint)


def ExploreDiagonal3D(src: Node, dx: int, dy: int, dz: int, dst: Node, nodeMap: list[list[list[Node]]]):
    global IsEnd
    if (IsEnd is True or src.Obstacle):
        return

    parNode: Node = src
    n = len(nodeMap)
    if (src == dst):
        IsEnd = True
        return
    g = src.G
    x, y, z = src.I, src.J, src.K
    while (IsEnd is False):
        nx: int = dx + x
        ny: int = dy + y
        nz: int = dz + z
        x, y, z = nx, ny, nz
        ng: float = g + (3) ** 0.5
        g = ng

        if (_IsOutOfRange(nx, ny, nz, n) or nodeMap[nx][ny][nz].Obstacle or nodeMap[nx][ny][nz].G != 0.0):
            return
        nodeMap[nx][ny][nz].Parent = parNode
        nodeMap[nx][ny][nz].G = ng

        if (nodeMap[nx][ny][nz] == dst):
            IsEnd = True
            return

        if (SearchJumpPointDiagonal3D(nodeMap[nx][ny][nz], dx, dy, dz, dst, nodeMap)):
            return

    return


def ExploreDiagonal2D(src: Node, dx: int, dy: int, dz: int, dst: Node,  nodeMap: list[list[list[Node]]]):
    global IsEnd
    if (IsEnd is True or src.Obstacle):
        return

    parNode: Node = src
    if (src == dst):
        IsEnd = True
        return
    n = len(nodeMap)
    g = src.G
    x, y, z = src.I, src.J, src.K
    while (IsEnd is False):
        nx: int = dx + x
        ny: int = dy + y
        nz: int = dz + z
        x, y, z = nx, ny, nz
        ng: float = g + (2) ** 0.5
        g = ng

        if (_IsOutOfRange(nx, ny, nz, n) or nodeMap[nx][ny][nz].Obstacle or nodeMap[nx][ny][nz].G != 0.0):
            return

        nodeMap[nx][ny][nz].Parent = parNode
        nodeMap[nx][ny][nz].G = ng

        if (nodeMap[nx][ny][nz] == dst):
            IsEnd = True
            return

        if (SearchJumpPointDiagonal2D(nodeMap[nx][ny][nz], dx, dy, dz, dst, nodeMap)):
            return

    return


def ExploreOrthogonal(src: Node, dx: int, dy: int, dz: int, dst: Node, nodeMap: list[list[list[Node]]]):
    global IsEnd
    if (IsEnd == True or src.Obstacle):
        return None
    parNode: Node = src
    x, y, z = src.I, src.J, src.K

    if (src == dst):
        IsEnd = True
        return

    n = len(nodeMap)
    g = src.G
    while (IsEnd is False):
        nx: int = dx + x
        ny: int = dy + y
        nz: int = dz + z
        x, y, z = nx, ny, nz

        ng: float = g + 1
        g = ng

        if (_IsOutOfRange(nx, ny, nz, n) or nodeMap[nx][ny][nz].Obstacle or nodeMap[nx][ny][nz].G != 0.0):
            return
        nodeMap[nx][ny][nz].Parent = parNode
        nodeMap[nx][ny][nz].G = ng
        if (nodeMap[nx][ny][nz] == dst):
            IsEnd = True
            return
        if (SearchJumpPointOrthogonal(nodeMap[nx][ny][nz], dx, dy, dz, dst,  nodeMap)):
            return
    return


class JpsTheta(Agent):
    def __init__(self, agentName: str = 'Jps') -> None:
        super().__init__(agentName)

    def Run(self, src: Node, dst: Node, nodeMap: list[list[list[Node]]]):
        global IsEnd

        closedList.clear()
        openList.clear()
        IsEnd = False
        startTime: float = time()
        src.Parent = src
        src.F = _CalEuclidDistance(src, dst)
        heapq.heappush(openList, src)

        while (openList and not IsEnd):
            curNode: Node = heapq.heappop(openList)
            closedList.add(curNode)

            ExploreOrthogonal(curNode, 1, 0, 0, dst, nodeMap)
            ExploreOrthogonal(curNode, -1, 0, 0, dst, nodeMap)
            ExploreOrthogonal(curNode, 0, 1, 0, dst, nodeMap)
            ExploreOrthogonal(curNode, 0, -1, 0, dst,  nodeMap)
            ExploreOrthogonal(curNode, 0, 0, 1, dst, nodeMap)
            ExploreOrthogonal(curNode, 0, 0, -1, dst,  nodeMap)

            ExploreDiagonal2D(curNode, 1, -1, 0, dst,  nodeMap)
            ExploreDiagonal2D(curNode, -1, -1, 0, dst, nodeMap)
            ExploreDiagonal2D(curNode, 1, 1, 0, dst, nodeMap)
            ExploreDiagonal2D(curNode, -1, 1, 0, dst,  nodeMap)

            ExploreDiagonal2D(curNode, 1, 0, -1, dst,  nodeMap)
            ExploreDiagonal2D(curNode, -1, 0, -1, dst,  nodeMap)
            ExploreDiagonal2D(curNode, 1, 0, 1, dst, nodeMap)
            ExploreDiagonal2D(curNode, -1, 0, 1, dst,  nodeMap)

            ExploreDiagonal2D(curNode, 0, -1, -1, dst, nodeMap)
            ExploreDiagonal2D(curNode, 0, 1, -1, dst,  nodeMap)
            ExploreDiagonal2D(curNode, 0, -1, 1, dst,  nodeMap)
            ExploreDiagonal2D(curNode, 0, 1, 1, dst, nodeMap)

            # 3d diagonal
            ExploreDiagonal3D(curNode, 1, 1, 1, dst, nodeMap)

            ExploreDiagonal3D(curNode, 1, 1, -1, dst,  nodeMap)
            ExploreDiagonal3D(curNode, 1, -1, 1, dst,  nodeMap)
            ExploreDiagonal3D(curNode, -1, 1, 1, dst,  nodeMap)

            ExploreDiagonal3D(curNode, 1, -1, -1, dst, nodeMap)
            ExploreDiagonal3D(curNode, -1, 1, -1, dst, nodeMap)
            ExploreDiagonal3D(curNode, -1, -1, 1, dst, nodeMap)

            ExploreDiagonal3D(curNode, -1, -1, -1, dst, nodeMap)

        # if (IsEnd):
        self.CalTime = time() - startTime
        self._InitPathNodeList(src, dst)
        self._InitDistance()
        self._InitDegree()


if (__name__ == "__main__"):
    a, b, c, d = init_display()
    a.View.SetBackgroundColor(Quantity_TOC_RGB, 0, 0, 0)
    a.hide_triedron()

    a.View.SetBgGradientColors(
        Quantity_Color(Quantity_NOC_WHITE),
        Quantity_Color(Quantity_NOC_WHITE),
        2,
        True,
    )

    startTime = time()
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 10)
    # 1081, 9, 1002, 9
    # random.seed(1213)
    # random.seed(332345)
    grids.InitRandomBoxObstacle(3)
    grids.DisplayGridsMapInObstacle(a, _color="black", _transparency=0)

    start = grids.NodeMap[0][0][0]

    end = grids.NodeMap[9][9][9]

    astar = JpsTheta("astar")
    astar.Run(start, end, grids.NodeMap)
    astar._PostSmoothing(start, end, grids.NodeMap)
    # astar._PostSmoothing(start, end, grids.NodeMap)
    pnts = astar.GetPathPoints()
    for i in pnts:
        sh = BRepPrimAPI_MakeSphere(i, 10).Shape()
        a.DisplayShape(sh, color="blue")
    sb = SplineBuilder(pnts, 3)
    sb.DisplaySplineShape(a, _color="black")

    b()

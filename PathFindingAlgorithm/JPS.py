import numba
import heapq
import cProfile
import time


from Agent import Agent
from GridsMap.Node import Node
from GridsMap.GridsMap import GridsMap
from OCC.Core.gp import gp_Pnt

IsEnd: bool = False
openList: list[Node] = []
closedList: set[Node] = set()
nodeMap = None


def SearchJumpPointDiagonal3D(src: Node, dx: int, dy: int, dz: int, dst: Node, nodeMap: list[list[list[Node]]]):
    global IsEnd
    if (src in closedList or IsEnd is True):
        return True
    x, y, z = src.I, src.J, src.K
    n = len(nodeMap)
    if (dz == 1):
        for i in range(3):
            if ((not _IsOutOfRange(x - dx, y + dy, z + i, n)) and
                nodeMap[x - dx][y][z + i].Obstacle) and \
                    (not nodeMap[x - dx][y + dy][z + i].Obstacle):
                return True

            if ((not _IsOutOfRange(x + dx, y - dy, z + i, n)) and
                    nodeMap[x][y - dy][z + i].Obstacle and
                    (not nodeMap[x + dx][y - dy][z + i].Obstacle)):
                return True

            if (i == 0):
                if (not _IsOutOfRange(x + dx, y + dy, z + i, n)) and \
                    nodeMap[x][y][z + i].Obstacle and \
                        (not nodeMap[x + dx][y + dy][z + i].Obstacle):
                    return True
                if (not _IsOutOfRange(x + dx, y, z + i, n)) and \
                    nodeMap[x][y][z + i].Obstacle and \
                        (not nodeMap[x + dx][y][z + i].Obstacle):
                    return True
                if (not _IsOutOfRange(x, y + dy, z + i, n)) and \
                    nodeMap[x][y][z + i].Obstacle and \
                        (not nodeMap[x][y + dy][z + i].Obstacle):
                    return True

        return False

    if (dz == -1):
        for i in range(0, -3, -1):
            if ((not _IsOutOfRange(x - dx, y + dy, z + i, n)) and
                    nodeMap[x - dx][y][z + i].Obstacle and
                    (not nodeMap[x - dx][y + dy][z + i].Obstacle)):
                return True

            if ((not _IsOutOfRange(x + dx, y - dy, z + i, n)) and
                    nodeMap[x][y - dy][z + i].Obstacle and
                    (not nodeMap[x + dx][y - dy][z + i].Obstacle)):
                return True

            if (i == 0):
                if ((not _IsOutOfRange(x + dx, y + dy, z + i, n)) and
                        nodeMap[x][y][z + i].Obstacle and
                        (not nodeMap[x + dx][y + dy][z + i].Obstacle)):
                    return True
                if (not _IsOutOfRange(x + dx, y, z + i, n)) and \
                    nodeMap[x][y][z + i].Obstacle and \
                        (not nodeMap[x + dx][y][z + i].Obstacle):
                    return True
                if (not _IsOutOfRange(x, y + dy, z + i, n)) and \
                    nodeMap[x][y][z + i].Obstacle and \
                        (not nodeMap[x][y + dy][z + i].Obstacle):
                    return True

        return False


def SearchJumpPointDiagonal2D(src: Node, dx: int, dy: int, dz: int, dst: Node, nodeMap: list[list[list[Node]]]):
    global IsEnd
    if (src in closedList or IsEnd is True):
        return True
    x, y, z = src.I, src.J, src.K
    n = len(nodeMap)

    if (dz == 0):
        for i in range(-1, 2):
            if (not _IsOutOfRange(x + 2 * dx, y, z + i, n) and nodeMap[x + dx][y][z + i].Obstacle and not nodeMap[x + 2 * dx][y][z + i].Obstacle):

                return True

            if (not _IsOutOfRange(x, y + 2 * dy, z + i, n) and nodeMap[x][y + dy][z + i].Obstacle and not nodeMap[x][y + 2 * dy][z + i].Obstacle):

                return True

            if (i != 0):
                if (not _IsOutOfRange(x + dx, y + dy, z + i, n) and nodeMap[x][y][z + i].Obstacle and not nodeMap[x + dx][y + dy][z + i]):

                    return True
                if (not _IsOutOfRange(x + dx, y, z + i, n) and nodeMap[x][y][z + i].Obstacle and not nodeMap[x + dx][y][z + i]):
                    return True
                if (not _IsOutOfRange(x, y + dy, z + i, n) and nodeMap[x][y][z + i].Obstacle and not nodeMap[x][y + dy][z + i]):
                    return True

        return False

    if (dx == 0):
        for i in range(-1, 2):
            if (not _IsOutOfRange(x + i, y, z + 2 * dz, n) and nodeMap[x + i][y][z + dz].Obstacle and not nodeMap[x + i][y][z + 2 * dz].Obstacle):
                return True

            if (not _IsOutOfRange(x + i, y + 2 * dy, z, n) and nodeMap[x + i][y + dy][z].Obstacle and not nodeMap[x + i][y + 2 * dy][z].Obstacle):
                return True

            if (i != 0):
                if (not _IsOutOfRange(x + i, y + dy, z + dz, n) and nodeMap[x + i][y][z].Obstacle and not nodeMap[x + i][y + dy][z + dz]):
                    return True
                if (not _IsOutOfRange(x + i, y + dy, z, n) and nodeMap[x + i][y][z].Obstacle and not nodeMap[x + i][y + dy][z]):
                    return True
                if (not _IsOutOfRange(x + i, y, z + dz, n) and nodeMap[x + i][y][z].Obstacle and not nodeMap[x + i][y][z + dz]):
                    return True

        return False

    if (dy == 0):
        for i in range(-1, 2):
            if (not _IsOutOfRange(x, y, z + 2 * dz, n) and nodeMap[x][y][z + dz].Obstacle and not nodeMap[x][y][z + 2 * dz].Obstacle):
                return True

            if (not _IsOutOfRange(x + 2 * dx, y, z, n) and nodeMap[x + dx][y][z].Obstacle and not nodeMap[x + 2 * dx][y][z].Obstacle):
                return True

            if (i != 0):
                if (not _IsOutOfRange(x + dx, y + i, z + dz, n) and nodeMap[x][y + i][z].Obstacle and not nodeMap[x + dx][y + i][z + dz]):
                    return True
                if (not _IsOutOfRange(x + dx, y + i, z, n) and nodeMap[x][y + i][z].Obstacle and not nodeMap[x + dx][y + i][z]):
                    return True
                if (not _IsOutOfRange(x, y + i, z + dz, n) and nodeMap[x][y + i][z].Obstacle and not nodeMap[x][y + i][z + dz]):
                    return True

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
                return True
            if ((not _IsOutOfRange(x + dx, y - 1, z + i, n)) and
                    nodeMap[x][y - 1][z + i].Obstacle and
                    (not nodeMap[x + dx][y - 1][z + i].Obstacle)):
                return True
            if (i != 0):
                if ((not _IsOutOfRange(x + i, y, z + dz, n)) and nodeMap[x + i][y][z].Obstacle and not nodeMap[x + i][y][z + dz].Obstacle):
                    return True
                if ((not _IsOutOfRange(x + i, y + dy, z, n)) and nodeMap[x + i][y][z].Obstacle and not nodeMap[x + i][y + dy][z].Obstacle):
                    return True
        return False

    if (dy):
        for i in range(-1, 2):
            if ((not _IsOutOfRange(x + 1, y + dy, z + i, n)) and
                    nodeMap[x + 1][y][z + i].Obstacle and
                    (not nodeMap[x + 1][y + dy][z + i].Obstacle)):
                return True
            if ((not _IsOutOfRange(x - 1, y + dy, z + i, n)) and
                    nodeMap[x - 1][y][z + i].Obstacle and
                    (not nodeMap[x - 1][y + dy][z + i].Obstacle)):
                return True

            if (i != 0):
                if ((not _IsOutOfRange(x + dx, y + i, z, n)) and nodeMap[x][y + i][z].Obstacle and not nodeMap[x + dx][y + i][z].Obstacle):
                    return True
                if ((not _IsOutOfRange(x, y + i, z + dz, n)) and nodeMap[x][y + i][z].Obstacle and not nodeMap[x][y + i][z + dz].Obstacle):
                    return True

        return False

    if (dz):
        for i in range(-1, 2):
            if ((not _IsOutOfRange(x + 1, y + i, z + dz, n)) and
                    nodeMap[x + 1][y + i][z].Obstacle and
                    (not nodeMap[x + 1][y + i][z + dz].Obstacle)):
                return True
            if ((not _IsOutOfRange(x - 1, y + i, z + dz, n)) and
                    nodeMap[x - 1][y + i][z].Obstacle and
                    (not nodeMap[x - 1][y + i][z + dz].Obstacle)):
                return True
            if (i != 0):
                if ((not _IsOutOfRange(x + dx, y, z + i, n)) and nodeMap[x][y][z + i].Obstacle and not nodeMap[x + dx][y][z + i].Obstacle):
                    return True
                if ((not _IsOutOfRange(x, y + dy, z + i, n)) and nodeMap[x][y][z + i].Obstacle and not nodeMap[x][y + dy][z + i].Obstacle):
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
            nodeMap[nx][ny][nz].F = nodeMap[nx][ny][nz].G + \
                _CalEuclidDistance(nodeMap[nx][ny][nz], dst)
            if (nodeMap[nx][ny][nz] in openList):
                openList.remove(nodeMap[nx][ny][nz])
            heapq.heappush(openList, nodeMap[nx][ny][nz])
            return

        node = ExploreOrthogonal(nodeMap[nx][ny][nz], dx, 0, 0, dst,  nodeMap)
        if (node):
            heapq.heappush(openList, node)
            return
        node = ExploreOrthogonal(nodeMap[nx][ny][nz], 0, dy, 0, dst,  nodeMap)
        if (node):
            heapq.heappush(openList, node)
            return
        node = ExploreOrthogonal(nodeMap[nx][ny][nz], 0, 0, dz, dst,  nodeMap)
        if (node):
            heapq.heappush(openList, node)
            return
        node = ExploreDiagonal2D(nodeMap[nx][ny][nz], dx, dy, 0, dst, nodeMap)
        if (node):
            heapq.heappush(openList, node)
            return
        node = ExploreDiagonal2D(nodeMap[nx][ny][nz], 0, dy, dz, dst, nodeMap)
        if (node):
            heapq.heappush(openList, node)
            return
        node = ExploreDiagonal2D(nodeMap[nx][ny][nz], dx, 0, dz, dst, nodeMap)
        if (node):
            heapq.heappush(openList, node)
            return
    return


def ExploreDiagonal2D(src: Node, dx: int, dy: int, dz: int, dst: Node,  nodeMap: list[list[list[Node]]]):
    global IsEnd
    if (IsEnd is True or src.Obstacle):
        return

    parNode: Node = src

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
            nodeMap[nx][ny][nz].F = nodeMap[nx][ny][nz].G + \
                _CalEuclidDistance(nodeMap[nx][ny][nz], dst)
            if (nodeMap[nx][ny][nz] in openList):
                openList.remove(nodeMap[nx][ny][nz])
            heapq.heappush(openList, nodeMap[nx][ny][nz])
            return nodeMap[nx][ny][nz]

        if (dx):
            node = ExploreOrthogonal(
                nodeMap[nx][ny][nz], dx, 0, 0, dst, nodeMap)
            if (node):
                heapq.heappush(openList, node)
                return

        if (dy):
            node = ExploreOrthogonal(
                nodeMap[nx][ny][nz], 0, dy, 0, dst, nodeMap)
            if (node):
                heapq.heappush(openList, node)
                return
        if (dz):
            node = ExploreOrthogonal(
                nodeMap[nx][ny][nz], 0, 0, dz, dst, nodeMap)
            if (node):
                heapq.heappush(openList, node)
                return
    return


def ExploreOrthogonal(src: Node, dx: int, dy: int, dz: int, dst: Node, nodeMap: list[list[list[Node]]]):
    global IsEnd
    if (IsEnd == True or src.Obstacle):
        return None
    parNode: Node = src
    x, y, z = src.I, src.J, src.K

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
            nodeMap[nx][ny][nz].F = nodeMap[nx][ny][nz].G + \
                _CalEuclidDistance(nodeMap[nx][ny][nz], dst)
            if (nodeMap[nx][ny][nz] in openList):
                openList.remove(nodeMap[nx][ny][nz])
            heapq.heappush(openList, nodeMap[nx][ny][nz])
            return
    return


class Jps(Agent):
    def __init__(self, agentName: str = 'Jps') -> None:
        super().__init__(agentName)

    def Run(self, src: Node, dst: Node, nodeMap: list[list[list[Node]]]):
        global IsEnd

        closedList.clear()
        openList.clear()
        IsEnd = False
        startTime: float = time.time()
        src.Parent = src
        src.F = _CalEuclidDistance(src, dst)
        heapq.heappush(openList, src)

        while (openList and not IsEnd):
            curNode: Node = heapq.heappop(openList)

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

            ExploreDiagonal3D(curNode, 1, 1, 1, dst, nodeMap)

            ExploreDiagonal3D(curNode, 1, 1, -1, dst,  nodeMap)
            ExploreDiagonal3D(curNode, 1, -1, 1, dst,  nodeMap)
            ExploreDiagonal3D(curNode, -1, 1, 1, dst,  nodeMap)

            ExploreDiagonal3D(curNode, 1, -1, -1, dst, nodeMap)
            ExploreDiagonal3D(curNode, -1, 1, -1, dst, nodeMap)
            ExploreDiagonal3D(curNode, -1, -1, 1, dst, nodeMap)

            ExploreDiagonal3D(curNode, -1, -1, -1, dst, nodeMap)

        self.CalTime = time.time() - startTime
        self._InitPathNodeList(src, dst)
        self._InitDistance()
        self._InitDegree()


if (__name__ == "__main__"):
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 50)
    grids.InitRandomBoxObstacle(50)
    startNode = grids.NodeMap[0][0][0]
    endNode = grids.NodeMap[grids.MapSize -
                            1][grids.MapSize - 1][grids.MapSize - 1]

    jps = Jps("jps")
    jps.Run(startNode, endNode, grids.NodeMap)
    print(jps.CalTime)

    pass

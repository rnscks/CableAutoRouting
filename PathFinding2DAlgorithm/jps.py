import util
import heapq
import math
from time import time
from PathFindingAlgorithm.Agent import Agent
from GridsMap.Node import Node

import numba
import heapq
import cProfile
import time


from GridsMap.Node import Node
from GridsMap.GridsMap import GridsMap
from OCC.Core.gp import gp_Pnt

retTime: float = 0.0
r = 0.0
obstacleList = [[[False for _ in range(100)]for _ in range(100)]for _ in range(100)]
closed_set = set()

def HasForcedNeighbor(src: Node, dx: int, dy: int, dz: int, nodeMap: list[list[list[Node]]]):
    x, y, z = src.I, src.J, src.K
    n = len(nodeMap)
    
    if (dx and dy == 0 and dz == 0):
        if ((not (x + dx >= n or y + 1 >= n or z >= n or x + dx < 0 or y + 1 < 0 or z< 0)) and
                    obstacleList[x][y + 1][z] and
                    (not obstacleList[x + dx][y + 1][z])):
            return True
        if ((not (x + dx>= n or y - 1 >= n or z >= n or x + dx < 0 or y - 1 < 0 or z < 0)) and
                    obstacleList[x][y - 1][z] and
                    (not obstacleList[x + dx][y - 1][z])):
            return True
        return False

    if (dy and dx == 0 and dz == 0):
        if ((not (x + 1 >= n or y + dy >= n or z >= n or x + 1 < 0 or y + dy < 0 or z < 0)) and
                    obstacleList[x + 1][y][z] and
                    (not obstacleList[x + 1][y + dy][z])):
            return True
        if ((not (x - 1 >= n or y + dy >= n or z >= n or x - 1 < 0 or y + dy < 0 or z < 0)) and
                    obstacleList[x - 1][y][z] and
                    (not obstacleList[x - 1][y + dy][z])):
            return True
        return False

    if (dz and dx == 0 and dy == 0):
        if ((not (x + 1 >= n or y >= n or z + dz >= n or x + 1 < 0 or y < 0 or z + dz < 0)) and
                    obstacleList[x + 1][y][z] and
                    (not obstacleList[x + 1][y][z + dz])):
            return True
        if ((not (x - 1 >= n or y >= n or z + dz >= n or x - 1 < 0 or y < 0 or z + dz < 0)) and
                    obstacleList[x - 1][y][z] and
                    (not obstacleList[x - 1][y][z + dz])):
            return True
        return False
    
    if (dz == 0 and dx and dy):
        if (not (x + 2*dx >= n or y - dy >= n or z >= n or x + 2*dx < 0 or y - dy < 0 or z < 0) and obstacleList[x + dx][y - dy][z] and not obstacleList[x + 2 * dx][y - dy][z]):
            return True

        if (not (x - dx >= n or y +2*dy >= n or z >= n or x - dx < 0 or y + 2 * dy < 0 or z < 0) and obstacleList[x - dx][y + dy][z] and not obstacleList[x - dx][y + 2 * dy][z]):
            return True
        return False

    if (dx == 0 and dz and dy):
        if (not (x >= n or y - dy >= n or z + 2 * dz >= n or x < 0 or y -dy < 0 or z + 2*dz < 0) and obstacleList[x][y - dy][z + dz] and not obstacleList[x][y - dy][z + 2 * dz]):
            return True

        if (not (x >= n or y + 2* dy >= n or z - dz >= n or x < 0 or y + 2* dy < 0 or z - dz < 0) and obstacleList[x][y + dy][z - dz] and not obstacleList[x][y + 2 * dy][z - dz]):
            return True
        return False

    if (dy == 0 and dx and dz):
        if (not (x - dx >= n or y >= n or z + 2 * dz >= n or x - dx < 0 or y < 0 or z + 2*dz< 0) and obstacleList[x - dx][y][z + dz] and not obstacleList[x - dx][y][z + 2 * dz]):
            return True

        if (not (x + 2 * dx >= n or y >= n or z - dz >= n or x + 2 * dx < 0 or y < 0 or z -dz< 0) and obstacleList[x + dx][y][z - dz] and not obstacleList[x + 2 * dx][y][z - dz]):
            return True
        return False
    
    if (dx and dy and dz):
        if ((not (x - dx >= n or y + 2 * dy >= n or z + dz>= n or x - dx < 0 or y + 2 * dy < 0 or z + dz< 0)) and
                    obstacleList[x - dx][y + dy][z] and
                    (not obstacleList[x - dx][y + 2 * dy][z + dz])):
            return True
        if ((not (x + 2 * dx >= n or y - dy >= n or z + dz >= n or x + 2* dx < 0 or y - dy < 0 or z + dz< 0)) and
                    obstacleList[x + dx][y - dy][z] and
                    (not obstacleList[x + 2 * dx][y - dy][z + dz])):
            return True
        return False







def get_neighbors(node: Node, grid: list[list[list[Node]]]):
    neighbors = []
    directions = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (-1, 0, 0), (0, -1, 0), (0, 0, -1), (1, 1, 0), (-1, 1, 0), (1, -1, 0), (-1, -1, 0), (1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1), (0, 1, 1), (0, -1, 1), (0, 1, -1), (0, -1, -1), (1, 1, 1),(-1, 1, 1), (1, -1, 1), (1, 1, -1), (-1, -1, 1), (-1, 1 ,-1), (1, -1, -1), (-1, -1, -1)]

    n = len(grid)
    for dx, dy, dz in directions:
        nx, ny, nz = node.I + dx, node.J + dy, node.K + dz
        if ((nx >= n or ny >= n or nz >= n or nx < 0 or ny < 0 or nz < 0) or obstacleList[nx][ny][nz]):
            continue
        neighbors.append(grid[nx][ny][nz])
            
    return neighbors

def jump(current: Node, direction: tuple[int], goal:Node, grid: list[list[Node]]):
    global r
    nx, ny, nz = current.I + direction[0], current.J + direction[1], current.K + direction[2]
    n = len(grid)
    
    if (nx >= n or ny >= n or nz >= n or nx < 0 or ny < 0 or nz < 0) or obstacleList[nx][ny][nz] or current.Parent != None:
        return None
    
    nextNode: Node = grid[nx][ny][nz]
    
    if nextNode == goal:  # 목표 도달
        return nextNode

    # 다음 점프 방향 계산
    dx, dy, dz = direction
    s = time.time()
    
    if (HasForcedNeighbor(nextNode, dx, dy, dz, grid)):
        return nextNode
    s = time.time() - s
    r += s

    # 2D
    if (dx and dy and dz == 0):
        if (jump(nextNode, (dx, 0, 0), goal,grid) or jump(nextNode, (0, dy, 0), goal,grid)):
            return nextNode
    elif (dx and dz and dy == 0):
        if (jump(nextNode, (dx, 0, 0), goal,grid) or jump(nextNode, (0, 0, dz), goal,grid)):
            return nextNode
    elif (dy and dz and dx == 0):
        if (jump(nextNode, (0, 0, dz), goal,grid) or jump(nextNode, (0, dy, 0), goal,grid)):
            return nextNode
        
    # 3D
    if (dx and dy and dz):
        if (jump(nextNode, (dx, 0, 0), goal,grid) or jump(nextNode, (0, dy, 0), goal,grid) or jump(nextNode, (0, 0, dz), goal,grid)):
            return nextNode
        if (jump(nextNode, (dx, dy, 0), goal,grid) or jump(nextNode, (dx, 0, dz), goal,grid) or jump(nextNode, (0, dy, dz), goal,grid)):
            return nextNode
    
    return jump(nextNode, (dx, dy, dz), goal, grid)


def jps(start: Node, goal:Node, grid: list[list[list[Node]]]):
    global retTime
    retTime = time.time()
    open_set: list[Node] = []  # 우선순위 큐
    
    heapq.heappush(open_set, start)
    
    start.G = 0

    while open_set:
        current = heapq.heappop(open_set)
        if current == goal:
            retTime = time.time() - retTime
            return reconstruct_path(current)

        neighbors = get_neighbors(current, grid)
        
        for neighbor in neighbors:
            jump_point = jump(neighbor, (neighbor.I - current.I, neighbor.J - current.J,neighbor.K - current.K), goal,grid )
            
            if jump_point:
                tentative_g = current.G + current.CenterPoint.Distance(jump_point.CenterPoint)
                if jump_point.Parent == None or tentative_g < jump_point.G:
                    jump_point.G = tentative_g
                    jump_point.F = tentative_g + jump_point.CenterPoint.Distance(goal.CenterPoint)
                    jump_point.Parent = current
                    
                    heapq.heappush(open_set, jump_point)

def reconstruct_path(node):
    path = []
    while node:
        path.append((node.I, node.J, node.K))
        node = node.Parent
    return path[::-1]

    

    
class JPSNew(Agent):
    def __init__(self, name: str = "agent") -> None:
        super().__init__(name)
    def Run(self, startNode: Node, endNode: Node, nodeMap: list[list[list[Node]]]):
        global retTime
        for i in range(len(nodeMap)):
            for j in range(len(nodeMap)):
                for k in range(len(nodeMap)):
                    if (nodeMap[i][j][k].Obstacle):
                        obstacleList[i][j][k] = True
                        
        ret = jps(startNode, endNode, nodeMap)
        self.CalTime = retTime
        for i in range(len(ret) - 1):
            self.Distance += nodeMap[ret[i][0]][ret[i][1]][ret[i][2]].CenterPoint.Distance(nodeMap[ret[i + 1][0]][ret[i + 1][1]][ret[i + 1][2]].CenterPoint)
        

if (__name__ == "__main__"):
    grids = GridsMap(gp_Pnt(0, 0, 0), gp_Pnt(1000, 1000, 1000), 30)
    grids.InitRandomBoxObstacle(30)
    start = grids.NodeMap[0][0][0]
    end = grids.NodeMap[grids.MapSize - 1][grids.MapSize - 1][grids.MapSize - 1]

    j = JPSNew()
    j.Run(start, end, grids.NodeMap)
    print(j.CalTime)
    print(r)
    pass
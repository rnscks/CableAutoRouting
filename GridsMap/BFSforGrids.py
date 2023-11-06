import util
from Node import Node


class BFS:        
    def __init__(self, grids) -> None:
        self.NodeMap:list[list[list[Node]]] = grids.NodeMap
        self.ClosedSet: set[Node] = set()
        pass
    
    def Run(self, i:int, j: int, k: int):
        fuseList: list[Node] = []
        queue: list[Node] = []

        if (self.NodeMap[i][j][k] in self.ClosedSet):
            return
        fuseList.append(self.NodeMap[i][j][k])
        self.ClosedSet.add(self.NodeMap[i][j][k])
        queue.append(self.NodeMap[i][j][k])

        while queue:
            curNode: Node = queue.pop(0)
            dx = [-1, 1 ,0 ,0, 0, 0]
            dy = [0, 0, -1, 1, 0, 0]
            dz = [0, 0, 0, 0, -1, 1]

            x = curNode.I
            y = curNode.J
            z = curNode.K

            for i in range(6):
                nx: int = x + dx[i]    
                ny: int = y + dy[i]    
                nz: int = z + dz[i]    

                if (self.__OutOfRange(nx ,ny , nz)):
                    continue

                nxtNode:Node =  self.NodeMap[nx][ny][nz]
                if (not nxtNode.Obstacle or nxtNode in self.ClosedSet):
                    continue

                queue.append(nxtNode)
                self.ClosedSet.add(nxtNode)
                fuseList.append(nxtNode)
        return fuseList
                
    def __OutOfRange(self, nx, ny, nz):
        N = len(self.NodeMap)
        if (0 > nx or 0 > ny or 0 > nz or nx <= N or ny <= N or nz <= N):
            return True
        return False
    


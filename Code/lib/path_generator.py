from sys import path
from pathfinding.core.grid import Grid
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.finder.a_star import AStarFinder

class PathGenerator:
    def __init__(self, width=0, height=0, matrix=None, inverse=False):
        self.grid = Grid(width, height, matrix, inverse)

    def makeNeighborsObstacles(self, *position):
        node = self.grid.node(position[0], position[1])
        neighbors = self.grid.neighbors(node, DiagonalMovement.always)
        for node in neighbors:
            node.walkable = False
        return neighbors

    def makeNeighborsWalkable(self, neighbors):
        for node in neighbors:
            node.walkable = True

    def setToObstacle(self, *position):
        node = self.grid.node(position[0], position[1])
        node.walkable = False
        print(self.grid.grid_str())

    def setToWalkable(self, *position):
        node = self.grid.node(position[0], position[1])
        node.walkable = True
        print(self.grid.grid_str())

    def generateAStarPath(self, start, end):
        # print(start, end)
        start = self.grid.node(start[0], start[1])
        end = self.grid.node(end[0], end[1])
        pathfinder = AStarFinder(diagonal_movement=DiagonalMovement.never)
        path, _ = pathfinder.find_path(start, end, self.grid)
        if path:
            path.pop(0)
        self.grid.cleanup()
        return start, end, path[0]

    def printPath(self, path=None, start=None, end=None, start_chr='S', end_chr='X', path_chr='x', empty_chr=' ', show_weight=False):
        print(self.grid.grid_str(path=path, start=start, end=end, start_chr='S', end_chr='X', path_chr='x', empty_chr=' ', show_weight=False))



if __name__ == '__main__':
    path_generator = PathGenerator(10, 10)

    start = (2, 1)
    end = (9, 9)
    # print(path_generator.setToObstacle(2,1))
    _, __, path = path_generator.generateAStarPath(start, end)
    print(path)

    # path_generator.printPath(_, __, path)
    # path_generator.grid.
    # neighbors = path_generator.makeNeighborsObstacles(1, 1)
    # path_generator.printPath()

    # path_generator.makeNeighborsWalkable(neighbors)
    # path_generator.printPath()
# Niko's Seeker

import heapq
import math
from collections import deque
import random
# from threading import Lock, Thread
from Mesh.nav_mesh import NavMesh, NavMeshCell
from agent_base import Agent
from world_state import WorldState
import shapely


class wiseAgent(Agent):

    def __init__(
        self,
        world_map: NavMesh,
        max_speed: float,
    ):
        Agent.__init__(self, world_map, max_speed)
        self.world_map: NavMesh = world_map
        self.current_target: shapely.Point | None = None
        self.current_path_coords: deque[tuple] = deque()
        self.world_corners = list()
        self.name = "Wise Agent"
        self.corners_visited = []
        self.chase = False
        self.randoms = 0

    def getCorners(self) -> list[shapely.Point]:
        # adapted from NavMesh random_position
        # 0 is top left 
        # 1 is bottom left
        # 2 is bottom right
        # 3 is top right
        corners = []
        min_x, min_y, max_x, max_y = self.world_map.polygon.bounds
        x_width = max_x - min_x
        y_height = max_y - min_y 
        bounds = [([min_x, min_y], [min_x + x_width / 6, min_y + y_height / 6]),
                  ([min_x, min_y + 5 * y_height / 6], [min_x + x_width / 6, max_y]),
                  ([min_x + 5 * x_width / 6, min_y + 5 * y_height / 6], [max_x, max_y]),
                  ([min_x + 5 * x_width / 6, min_y], [max_x, min_y + y_height / 6])]
        i = 0
        while i < 4:
            point = shapely.Point(NavMesh.rng.uniform(bounds[i][0], bounds[i][1]))
            if self.world_map.polygon.contains(point):
                corners.insert(i, point)
                i += 1
        return corners

    def cellFromPoint(self, point: shapely.Point):
            return self.world_map.find_cell(point)

    def getClosestCorner(self, state: WorldState) -> shapely.Point | None:
        pos = self.cellFromPoint(state.seeker_position)

        def cornerCell(corner: shapely.Point) -> NavMeshCell | None:
            return self.cellFromPoint(corner)

        dists = list()
        i = 0
        if pos == None:
            return None
        while i < 4:
            corner = cornerCell(self.world_corners[i])
            if corner != None:
                heapq.heappush(dists, (pos.distance(corner), i))
            i += 1
        index = heapq.heappop(dists)
        index = index[1]
        return self.world_corners[index]

    def updatePath(self, start_point: shapely.Point, end_point: shapely.Point):
        current_path_corners = deque()
        
        def sharedCorners(cell1: NavMeshCell, cell2: NavMeshCell) -> set[tuple]:
            set1 = set(cell1.polygon.exterior.coords)
            set2 = set(cell2.polygon.exterior.coords)
            return set1 & set2
        
        if (start_result:= self.cellFromPoint(start_point)) != None and (end_result := self.cellFromPoint(end_point)) != None:
            start_cell: NavMeshCell = start_result
            end_cell: NavMeshCell = end_result

            # navigation within same cell
            if start_cell == end_cell:
                self.current_path_coords.append((end_point.x, end_point.y))
                return

            current_path_cells = self.astar(start_cell, end_cell)
            if current_path_cells != None:
                for cell, next_cell in zip(current_path_cells, current_path_cells[1:]):
                    for corner in sharedCorners(cell, next_cell):
                        if (len(current_path_corners) < 2) or corner != current_path_corners[-1]:
                            current_path_corners.appendleft(corner)
                current_path_corners.pop()
                current_path_corners.append((end_point.x, end_point.y))
            self.current_path_coords = current_path_corners

            # corner 1 is shared corner of cell 1 and 2
            # corner 2 is shared corner of cell 1 and 2
            # corner 3 is shared unvisited corener of cell 2 and 3
            # corner 4 is shared unvisited corener of cell 3 and 4

            return

    def lookahead(self, state: WorldState):
        # check how many windows ahead are visible
        foundLimit = False
        while not foundLimit:
            if (len(self.current_path_coords) > 1 and
                self.world_map.has_line_of_sight(state.seeker_position, shapely.Point(self.current_path_coords[1]))):
                self.current_path_coords.popleft()
            else:
                foundLimit = True

    # def continuityCheck(self, start_point: shapely.Point, end_point: tuple) -> bool:
    #     # this was intended to be used to discover if the centroid to centroid path crosses through a corner, but then i  funnel aimplementednd forgot to remove this testing code from the navigation, so it is now here
    #     line = shapely.LineString([start_point, end_point])
    #     if self.world_map.polygon.contains(line):
    #         if self.world_map.polygon.contains_properly(line):
    #             return True
    #         else:
    #             return True
    #     return False
    
    def hiderCheck(self, state: WorldState):
        if state.hider_position != None:
            self.chase = True
            self.randoms = 0


    def act(self, state: WorldState) -> tuple[float, float] | None:
        # this if statement runs only on the first act
        if self.world_corners == []:
            self.world_corners = self.getCorners()
            closest_corner = self.getClosestCorner(state)
            if closest_corner != None:
                self.current_target = closest_corner
                self.updatePath(state.seeker_position, self.current_target)

        self.hiderCheck(state)

        # chase mode -> chase down hider if seen at some point
        if self.chase:
            if state.hider_position != None:
                # if LOS -> go to hider
                self.current_target = state.hider_position
                self.updatePath(state.seeker_position, self.current_target)
            elif state.seeker_position.distance(self.current_target) <= 3:
                # if previously seen but LOS lost + at previous known location -> pick up to 25 random neighboring cells
                if self.randoms <= 25:
                    target_neighbors = self.world_map.find_cell(state.seeker_position).neighbors # type: ignore
                    new_target = random.choice(list(target_neighbors.keys()))
                    self.current_target = new_target.polygon.centroid
                    self.updatePath(state.seeker_position, self.current_target)
                    self.randoms += 1
                else:
                    # if unable to regain LOS after visiting 25 random nearby cells, discontinue chase
                    self.chase = False
            # else (occurs if still traveling to next target while LOS lost) -> do nothing

        else:
            # not in chase mode

            # visit all corners
            if len(self.corners_visited) < 4:
                # print("visiting corners")
                if state.seeker_position in self.world_corners and state.seeker_position not in self.corners_visited:
                    index = list.index(self.world_corners, state.seeker_position)
                    self.corners_visited.insert(index, self.world_corners[index])
                    if index < 3:
                        index += 1
                    else:
                        index = 0
                    self.current_target = self.world_corners[index]
                    self.updatePath(state.seeker_position, self.current_target)
            else:
                # if all corners are already visited -> visit random locations (bogosort)
                if state.seeker_position.distance(self.current_target) <= 3:
                    self.current_target = self.world_map.random_position()
                    self.updatePath(state.seeker_position, self.current_target)

        # target determination above, movement below

        if state.seeker_position == self.current_target:
            return
        
        if not self.map.in_bounds(self.current_target): # type: ignore
            # if target not in bounds pick a nearby point as target
            while True:
                new_target = shapely.Point(NavMesh.rng.uniform([self.current_target.x - 0.25, self.current_target.y - 0.25], [self.current_target.x + 0.25, self.current_target.y + 0.25])) #type: ignore
                if self.world_map.polygon.contains_properly(new_target):
                    break
            self.current_target = new_target
            self.updatePath(state.seeker_position, self.current_target)

        self.lookahead(state)

        if len(self.current_path_coords) != 0:
            if tuple(state.seeker_position.coords)[0] == self.current_path_coords[0] and len(self.current_path_coords) > 1:
                self.current_path_coords.popleft()

            # self.continuityCheck(state.seeker_position, self.current_path_coords[0])

            dx, dy = (
                self.current_path_coords[0][0] - state.seeker_position.x,
                self.current_path_coords[0][1] - state.seeker_position.y,
            )
        else:
            # just in case
            dx, dy = (
                self.current_target.x - state.seeker_position.x, # type: ignore
                self.current_target.y - state.seeker_position.y # type: ignore
            )

        if dx == 0 and dy == 0:
            return (dx, dy)
        else:
            distance = math.dist((dx, dy), (0, 0))
            speed = min(distance, self.max_speed * 0.99)
            dx, dy = speed * dx / distance, speed * dy / distance
            return (dx, dy)
    
    def astar(self, start_cell: NavMeshCell, end_cell: NavMeshCell) -> list[NavMeshCell] | None:
        def distTogoal(cell: NavMeshCell) -> float:
            return cell.distance(end_cell)

        frontier = list()
        camefrom = dict()
        path = list()
        tiebreaker = 0
        found = False

        # tuple is (fscore, gscore, cell)
        heapq.heappush(frontier, (distTogoal(start_cell), 0, tiebreaker, start_cell))
        camefrom[start_cell] = None

        # finding goal
        while len(frontier) > 0:
            current_fscore = frontier[0][0]
            current_gscore = frontier[0][1]
            current_cell = frontier[0][3]

            if current_cell == end_cell:
                found = True
                break
            
            heapq.heappop(frontier)

            for cell in current_cell.neighbors:
                tiebreaker += 1
                if cell not in camefrom:
                    camefrom[cell] = current_cell
                    heapq.heappush(frontier, (distTogoal(cell) + current_gscore + 1, current_gscore + 1, tiebreaker, cell))
            
        # if all possible nodes are explored and frontier goes to zero without finding goal, return none    
        if found == False:
            return None

        # creating path
        current_cell = end_cell
        while current_cell != start_cell:
            path.append(current_cell)
            current_cell = camefrom[current_cell]
        path.append(start_cell)
        path.reverse

        return path

    @property
    def is_seeker(self) -> bool:
        return True
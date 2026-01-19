
#Ved
import shapely
from agent_base import Agent
from world_state import WorldState
from Mesh.nav_mesh import NavMesh, NavMeshCell
from collections import deque
import heapq
import math
import numpy as np


class DumbHider(Agent):

    def __init__(self, world_map: NavMesh, max_speed: float):
        Agent.__init__(self, world_map, max_speed)
        self.name = "Dumb Hider"
        self.world_map = world_map
        self.current_target: shapely.Point | None = None
        self.current_path_coords: deque[tuple] = deque()

    def updatePath(self, start_point: shapely.Point, end_point: shapely.Point):
        current_path_corners = deque()

        def cellFromPoint(point: shapely.Point):
            return self.world_map.find_cell(point)
        
        def sharedCorners(cell1: NavMeshCell, cell2: NavMeshCell) -> set[tuple]:
            set1 = set(cell1.polygon.exterior.coords)
            set2 = set(cell2.polygon.exterior.coords)
            return set1 & set2
        
        if (start_result:= cellFromPoint(start_point)) != None and (end_result := cellFromPoint(end_point)) != None:
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


    def lookahead(self, state: WorldState):
        # check how many windows ahead are visible
        foundLimit = False
        while not foundLimit:
            if (len(self.current_path_coords) > 1 and
                self.world_map.has_line_of_sight(state.hider_position, shapely.Point(self.current_path_coords[1]))):
                self.current_path_coords.popleft()
            else:
                foundLimit = True

    def continuityCheck(self, start_point: shapely.Point, end_point: tuple) -> bool:
        line = shapely.LineString([start_point, end_point])
        if self.world_map.polygon.contains(line):
            if self.world_map.polygon.contains_properly(line):
                return True
            else:
                return True
        return False


    def move(self, state: WorldState, target) -> tuple[float, float] | None:
        # if target has changed run astar and update path
        target_point = target[3].polygon.centroid
        if target[3] != self.current_target:
            self.current_target = target[3]
            
            self.updatePath(state.hider_position, target[3].polygon.centroid)

        if len(self.current_path_coords) == 0:
            return

        if state.hider_position == target_point or not self.map.in_bounds(target_point):
            return

        self.lookahead(state)

        if tuple(state.hider_position.coords)[0] == self.current_path_coords[0]:
            self.current_path_coords.popleft()

        self.continuityCheck(state.hider_position, self.current_path_coords[0])

        dx, dy = (
            self.current_path_coords[0][0] - state.hider_position.x,
            self.current_path_coords[0][1] - state.hider_position.y,
        )
        if dx == 0 and dy == 0 and len(self.current_path_coords) > 1:
            dx, dy = (
            self.current_path_coords[1][0] - state.hider_position.x,
            self.current_path_coords[1][1] - state.hider_position.y,
        )
        distance = math.dist((dx, dy), (0, 0))
        speed = min(distance, self.max_speed*0.99999999)
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

    def act(self, state: WorldState) -> tuple[float, float] | None:
        #print(state.frame)
        hider_pos = state.hider_position
        seeker_pos = state.seeker_position
        seeker_dis = []
        hider_cell = self.map.find_cell(hider_pos)
        seeker_cell = self.map.find_cell(seeker_pos)
        for cell in self.world_map.cells:
            heapq.heappush_max(seeker_dis,(cell.distance(seeker_cell), len(cell.neighbors), cell.id, cell))
        return self.move(state, seeker_dis[0])
        ...


    @property
    def is_seeker(self) -> bool:
        return False
    

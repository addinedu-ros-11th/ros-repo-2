import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

from .config import AutoParkConfig

GridCell = Tuple[int, int]


class Planner:
    def __init__(self, config: AutoParkConfig) -> None:
        self.cfg = config
        self.blocks: Set[GridCell] = set(config.block_cells)

    def in_bounds(self, cell: GridCell) -> bool:
        x, y = cell
        return 0 <= x < self.cfg.grid_size and 0 <= y < self.cfg.grid_size

    def is_free(self, cell: GridCell) -> bool:
        return self.in_bounds(cell) and cell not in self.blocks

    def neighbors(self, cell: GridCell) -> List[GridCell]:
        x, y = cell
        candidates = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [c for c in candidates if self.is_free(c)]

    def heuristic(self, a: GridCell, b: GridCell) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan_path(self, start: GridCell, goal: GridCell) -> List[GridCell]:
        if not self.is_free(start) or not self.is_free(goal):
            return []

        frontier: List[Tuple[float, GridCell]] = []
        heapq.heappush(frontier, (0.0, start))
        came_from: Dict[GridCell, Optional[GridCell]] = {start: None}
        cost_so_far: Dict[GridCell, float] = {start: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                break
            for nxt in self.neighbors(current):
                new_cost = cost_so_far[current] + 1.0
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self.heuristic(nxt, goal)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current

        if goal not in came_from:
            return []

        path: List[GridCell] = []
        cur: Optional[GridCell] = goal
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()
        return path

    def to_world(self, cell: GridCell) -> Tuple[float, float]:
        x, y = cell
        return (
            (x + 0.5) * self.cfg.cell_size_m,
            (y + 0.5) * self.cfg.cell_size_m,
        )

    def world_to_cell(self, x_m: float, y_m: float) -> GridCell:
        x = int(x_m // self.cfg.cell_size_m)
        y = int(y_m // self.cfg.cell_size_m)
        x = max(0, min(self.cfg.grid_size - 1, x))
        y = max(0, min(self.cfg.grid_size - 1, y))
        return (x, y)

    def heading_for_segment(self, a: GridCell, b: GridCell) -> float:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        return math.atan2(dy, dx)

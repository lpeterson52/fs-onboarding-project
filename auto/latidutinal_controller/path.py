import dataclasses
import math

class PathPoint:
    def __init__(self, x, y, heading=0, curvature=0, distance=0):
        self.x = x
        self.y = y
        self.heading = heading
        self.curvature = curvature
        self.distance = distance
        
class Path:
    def __init__(self, coordinates: list[tuple[float, float]]):
        self.points = [PathPoint(x, y) for (x, y) in coordinates]
        self._fill_distances()
        self._fill_headings()
        self._fill_curvatures()

    def _fill_distances(self):
        self.points[0].distance = 0.0
        for i in range(1, len(self.points)):
            prev, curr = self.points[i - 1], self.points[i]
            curr.distance = prev.distance + math.hypot(curr.x - prev.x, curr.y - prev.y)

    def _fill_headings(self):
        for i in range(1, len(self.points)):
            self.points[i - 1].heading = math.atan2(self.points[i].y - self.points[i - 1].y,
                                                    self.points[i].x - self.points[i - 1].x)
        self.points[-1].heading = self.points[-2].heading
    
    def _fill_curvatures(self):
        for i in range(1, len(self.points) - 1):
            x0, x1, x2 = self.points[i - 1].x, self.points[i].x, self.points[i + 1].x
            y0, y1, y2 = self.points[i - 1].y, self.points[i].y, self.points[i + 1].y
            cross = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)
            a = math.hypot(x1 - x0, y1 - y0)
            b = math.hypot(x2 - x1, y2 - y1)
            c = math.hypot(x2 - x0, y2 - y0)
            kappa_i = 2 * cross / (a * b * c)
        self.points[0].curvature = self.points[1].curvature
        self.points[-1].curvature = self.points[-2].curvature
    
    def get_num_points(self):
        return len(self.points)
    
    
            
        
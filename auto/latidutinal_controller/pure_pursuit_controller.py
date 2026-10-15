from path import Path
from lateral_car import LateralCar
import math

class PurePursuitController:
    def __init__(self, path: Path, lookahead):
        self.path: Path = path
        self.lookahead: float = lookahead
        self.closest_index = 0
    
    def calculate_desired_steering_angle(self, car: LateralCar):
        closest_point = self.path.points[self.closest_index]
        if (self.closest_index < self.path.get_num_points() - 1
            and math.hypot(self.path.points[self.closest_index + 1].x - car.x,
                           self.path.points[self.closest_index + 1].y - car.y)
                < math.hypot(closest_point.x - car.x, closest_point.y - car.y)):
            self.closest_index += 1
            closest_point = self.path.points[self.closest_index]
        target_s = closest_point.distance + self.lookahead
        j = self.closest_index
        while j < len(self.path.points) - 1 and self.path.points[j].distance < target_s:
            j += 1
        p0, p1 = self.path.points[j - 1], self.path.points[j]
        t = (target_s - p0.distance) / (p1.distance - p0.distance)
        gx = p0.x + t * (p1.x - p0.x)
        gy = p0.y + t * (p1.y - p0.y)
        dx = gx - car.x
        dy = gy - car.y
        
        x_v =  math.cos(car.heading) * dx + math.sin(car.heading) * dy
        y_v = -math.sin(car.heading) * dx + math.cos(car.heading) * dy
        curvature = 2 * y_v / (x_v ** 2 + y_v ** 2)
        return math.atan(car.wheelbase * curvature)
        
        
        
    
    
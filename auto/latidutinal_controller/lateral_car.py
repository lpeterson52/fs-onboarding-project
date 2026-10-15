import math
import numpy as np
class LateralCar:
    def __init__(self,
                 velocity,
                 wheelbase,
                 max_steering_angle,
                 heading: float = 0,
                 dt: float = 0.1,
                 start_x: float = 0,
                 start_y: float = 0):
        self.x = start_x
        self.y = start_y
        self.velocity = velocity
        self.heading = heading
        self.dt = dt
        self.wheelbase = wheelbase
        self.max_steering_angle = max_steering_angle
    
    def update(self, desired_turn_angle):
        clipped_turn_angle = np.clip(desired_turn_angle,
                                     -self.max_steering_angle,
                                     self.max_steering_angle)
        self.x += self.velocity * math.cos(self.heading) * self.dt
        self.y += self.velocity * math.sin(self.heading) * self.dt
        self.heading += self.velocity / self.wheelbase * math.tan(clipped_turn_angle) * self.dt
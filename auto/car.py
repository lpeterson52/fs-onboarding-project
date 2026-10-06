import numpy as np

class Car:
    def __init__(self, desired_v: float=20.0, dt: float=0.1, mass: float=1000, max_throttle_force: float=5000, friction: float=2.0):
        """A car containing simulation info.

        A car object to be used with the PIDController class. Stores kinematics info, current time, and step count.

        Args:
            desired_v: Velocity set point [m/s]. Defaults to 20.0.
            dt: Timestep length [s]. Defaults to 0.1.
            mass: Mass of car [kg]. Defaults to 1000.
            max_throttle_force: Max force output of car [N]. Defaults to 5000.
            friction: Level of negative acceleration on the car [m/s^2]. Defaults to 2.
        """
        self.velocity: float = 0
        self.acceleration: float = 0
        self.time: float = 0 
        self.x: float = 0
        self.dt: float = dt 
        self.desired_v: float = desired_v 
        self.step: int = 0
        self.mass: float = mass
        self.max_throttle_force: float = max_throttle_force
        self.friction: float = friction
    
    def update(self, throttle_perc: float) -> None:
        """Updates the car's state variables based on the throttle percentage.

        Args:
            throttle_perc: throttle percentage (-1 to 1)
        """
        force = throttle_perc * self.max_throttle_force
        self.acceleration = (force / self.mass) - self.friction
        self.velocity += self.acceleration * self.dt
        self.x += self.velocity * self.dt
        self.time += self.dt
        self.step += 1
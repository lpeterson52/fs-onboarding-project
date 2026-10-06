import numpy as np

class Car:
    def __init__(self, desired_v: float=20.0, dt: float=0.1):
        """A car containing simulation info.

        A car object to be used with the PIDController class. Stores kinematics info, current time, and step count.

        Args:
            desired_v: Velocity set point [m/s]. Defaults to 20.0.
            dt: Timestep length [s]. Defaults to 0.1.
        """
        self.velocity: float = 0
        self.acceleration: float = 0
        self.time: float = 0 
        self.x: float = 0
        self.dt: float = dt 
        self.desired_v: float = desired_v 
        self.step: int = 0
    
    def update(self, throttle_perc: float, mass: float = 1000, max_throttle_force: float = 5000, friction: float = 2.0) -> None:
        
        """
        Updates the car's state variables based on the throttle percentage.
        Use this function after finding throttle percentage to update the car's state variables.

        Inputs:
        car: dictionary containing the car's state variables
        throttle_perc: float, throttle percentage (-1 to 1)

        Outputs:
        None, but updates the car's state variables
        """
        force = throttle_perc * max_throttle_force
        self.acceleration = (force / mass) - friction
        self.velocity += self.acceleration * self.dt
        self.x += self.velocity * self.dt
        self.time += self.dt
        self.step += 1
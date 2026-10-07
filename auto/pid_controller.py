import numpy as np
from car import Car

class PIDController:
    def __init__(self, K_P: float, K_I: float = 0.0, K_D: float = 0.0, K_B: float = 0.0):
        """A simple PID Controller.

        A PID Controller to be used with the Car class. With the default gains,
        it acts as a P-only controller.

        Args:
            K_P: Proportional gain.
            K_I: Integral gain. Defaults to 0.0 (disabled).
            K_D: Derivative gain. Defaults to 0.0 (disabled).
            K_B: Back-calculation gain. Defaults to 0.0 (disabled).
        """
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.K_B = K_B
        
        self.error_prev: float = None
        self.net_integral: float = 0.0

    def calculate_desired_acceleration(self, car: Car) -> tuple[float, float]:
        """Calculate the desired acceleration from the car's velocity error.

        Args:
            car: A car object whose desired_v and velocity are used to compute the error.

        Returns:
            A tuple (desired_acceleration, error) where the error is
                car.desired_v - car.velocity.
        """
        error: float = car.desired_v - car.velocity
        P: float = self.K_P * error
        self.net_integral += error * car.dt
        I: float = self.K_I * self.net_integral
        
        D: float = 0
        if self.error_prev is not None:
            D = self.K_D * (error - self.error_prev) / car.dt
        
        # back-calculation
        raw_output: float = P + I + D
        if self.K_I != 0:
            saturated_output: float = _calculate_saturated_output(car, desired_acceleration=raw_output)
            self.net_integral += (self.K_B / self.K_I) * (saturated_output - raw_output) * car.dt
        self.error_prev = error
        return (raw_output, error)

def acceleration_to_throttle_percentage(car: Car, desired_acceleration: float) -> float:
    """Calculates throttle based on desired acceleration.

    Args:
        car: The car which contains the kinematics state.
        desired_acceleration: The commanded acceleration [m/s^2].

    Returns:
        The desired throttle clipped to be between -1 and 1.
    """
    max_acceleration: float = car.max_throttle_force / car.mass
    desired_throttle: float = desired_acceleration / max_acceleration
    clipped: float = np.clip(desired_throttle, -1.0, 1.0)
    return clipped

def _calculate_saturated_output(car: Car, desired_acceleration: float) -> float:
    max_acceleration: float = car.max_throttle_force / car.mass
    return np.clip(desired_acceleration, -max_acceleration, max_acceleration)
    

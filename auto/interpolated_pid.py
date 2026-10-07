import numpy as np
from car import Car
from dataclasses import dataclass
from pid_controller import acceleration_to_throttle_percentage, _calculate_saturated_output

@dataclass(frozen=True)
class GainPoint:
    velocity: float
    K_P: float
    K_I: float = 0.0
    K_D: float = 0.0
    K_B: float = 0.0
    
    def __post_init__(self):
        if min(self.K_P, self.K_I, self.K_D, self.K_B) < 0:
            raise ValueError(f"Gains must be non-negative: {self}")
    
def _is_strictly_increasing(arr: np.ndarray):
    return all(a < b for a, b in zip(arr, arr[1:]))
    

class InterpolatedPID():
    
    def __init__(self, gain_table: list[GainPoint]):
        self.velocities = np.array([p.velocity for p in gain_table])
        
        if not gain_table:
            raise ValueError("Gain table must contain at least one GainPoint")
        if not _is_strictly_increasing(self.velocities):
            raise ValueError(f"Gain table velocities must be strictly increasing: {self.velocities.tolist()}")
        
        self.K_Ps = np.array([p.K_P for p in gain_table])
        self.K_Is = np.array([p.K_I for p in gain_table])
        self.K_Ds = np.array([p.K_D for p in gain_table])
        self.K_Bs = np.array([p.K_B for p in gain_table])
        
        self.error_prev: float = None
        self.net_integral: float = 0.0
        
        
        
    def get_interpolated_gains(self, velocity: float) -> GainPoint:
        return GainPoint(
            velocity=velocity,
            K_P=float(np.interp(velocity, self.velocities, self.K_Ps)),
            K_I=float(np.interp(velocity, self.velocities, self.K_Is)),
            K_D=float(np.interp(velocity, self.velocities, self.K_Ds)),
            K_B=float(np.interp(velocity, self.velocities, self.K_Bs))
        )
    
    def calculate_desired_acceleration(self, car: Car) -> tuple[float, float]:
        gains: GainPoint = self.get_interpolated_gains(car.velocity)
        error: float = car.desired_v - car.velocity
        P: float = gains.K_P * error
        self.net_integral += error * car.dt
        I: float = gains.K_I * self.net_integral
        
        D: float = 0
        if self.error_prev is not None:
            D = gains.K_D * (error - self.error_prev) / car.dt
        
        # back-calculation
        raw_output: float = P + I + D
        if gains.K_I != 0:
            saturated_output: float = _calculate_saturated_output(car, desired_acceleration=raw_output)
            self.net_integral += (gains.K_B / gains.K_I) * (saturated_output - raw_output) * car.dt
        self.error_prev = error
        return (raw_output, error)
    
    
    
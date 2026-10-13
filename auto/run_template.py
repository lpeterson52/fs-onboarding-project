import matplotlib.pyplot as plt
from car import Car
from pid_controller import PIDController, acceleration_to_throttle_percentage
from interpolated_pid import InterpolatedPID, GainPoint
from drag_function import create_drag_function

K_P = 2
K_I = 0.22
K_D = 0.5
K_B = 0.12
DRAG_FUNCTION = create_drag_function(drag_coefficient=0.23,
                                     reference_area=2)

car: Car = Car(desired_v=80, dt=0.1, drag_function=DRAG_FUNCTION)

gain_table: list[GainPoint] = [
    GainPoint(velocity=0.0,  K_P=1.5, K_I=0.03, K_D=0.2, K_B=0.03),
    GainPoint(velocity=30.0, K_P=1.0, K_I=0.03, K_D=0.3, K_B=0.03),
    GainPoint(velocity=60.0, K_P=2.0, K_I=0.15, K_D=0.4, K_B=0.10),
    GainPoint(velocity=80.0, K_P=2.0, K_I=0.3, K_D=0.4, K_B=0.1)
]
pid_controller = InterpolatedPID(gain_table)
# pid_controller = PIDController(K_P=K_P, K_I=K_I, K_D=K_D, K_B=K_B)

SECONDS:    int = 55
STEPS:      int = int(SECONDS / car.dt)

velocities: list[float] = []
errors:     list[float] = []
times:      list[float] = []
integral:   list[float] = []
throttles:  list[float] = []

# Run PID Simulation
for _ in range(STEPS):
    (desired_acceleration, error) = pid_controller.calculate_desired_acceleration(car=car)
    velocities.append(car.velocity)
    errors.append(error)
    times.append(car.time)
    integral.append(pid_controller.net_integral)
    
    throttle_percentage: float = acceleration_to_throttle_percentage(car=car, 
                                                                     desired_acceleration=desired_acceleration)
    throttles.append(throttle_percentage)
    car.update(throttle_perc=throttle_percentage)
    

plt.plot(times, velocities)
plt.ylabel("Velocity")
plt.xlabel("Time")
plt.hlines(y=car.desired_v, xmin=0, xmax=SECONDS, colors='g', linestyles='solid')
plt.show()

plt.plot(times, errors)
plt.ylabel("Error")
plt.xlabel("Time")
plt.hlines(y=0, xmin=0, xmax=SECONDS, colors='g', linestyles='solid')
plt.show()

plt.plot(times, integral)
plt.ylabel("Integral")
plt.xlabel("Time")
plt.hlines(y=0, xmin=0, xmax=SECONDS, colors='g', linestyles='solid')
plt.show()

plt.plot(times, throttles)
plt.ylabel("Throttle")
plt.xlabel("Time")
plt.hlines(y=0, xmin=0, xmax=SECONDS, colors='g', linestyles='solid')
plt.show()
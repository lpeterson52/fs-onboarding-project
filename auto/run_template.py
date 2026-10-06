import matplotlib.pyplot as plt
from car import Car
from pid_controller import PIDController, acceleration_to_throttle_percentage

K_P = 2
K_I = 0.1
K_D = 0.5

car: Car = Car(desired_v=20.0, dt=0.1)
pid_controller: PIDController = PIDController(K_P=K_P, K_I=K_I, K_D=K_D)

SECONDS = 55
STEPS = int(SECONDS / car.dt)

velocities: list[float] = []
errors: list[float] = []
times: list[float] = []

# Run PID Simulation
for _ in range(STEPS):
    (desired_acceleration, error) = pid_controller.calculate_desired_acceleration(car=car)
    throttle_percentage: float = acceleration_to_throttle_percentage(car=car, desired_acceleration=desired_acceleration)
    car.update(throttle_perc=throttle_percentage)
    velocities.append(car.velocity)
    errors.append(error)
    times.append(car.time)

plt.plot(times, velocities)
plt.ylabel("Velocity")
plt.xlabel("Time")
plt.show()

plt.plot(times, errors)
plt.ylabel("Error")
plt.xlabel("Time")
plt.show()

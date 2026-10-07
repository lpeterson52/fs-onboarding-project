import matplotlib.pyplot as plt
from car import Car
from pid_controller import PIDController, acceleration_to_throttle_percentage

K_P = 1
K_I = 0.1
K_D = 0.5
K_B = 0.1

car: Car = Car(desired_v=60.0, dt=0.1)
pid_controller: PIDController = PIDController(K_P=K_P, K_I=K_I, K_D=K_D, K_B=K_B)

SECONDS = 55
STEPS = int(SECONDS / car.dt)

velocities: list[float] = []
errors: list[float] = []
times: list[float] = []
integral: list[float] = []

# Run PID Simulation
for _ in range(STEPS):
    (desired_acceleration, error) = pid_controller.calculate_desired_acceleration(car=car)
    velocities.append(car.velocity)
    errors.append(error)
    times.append(car.time)
    integral.append(pid_controller.net_integral)
    
    throttle_percentage: float = acceleration_to_throttle_percentage(car=car, desired_acceleration=desired_acceleration)
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
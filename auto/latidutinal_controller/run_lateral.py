import math
import matplotlib.pyplot as plt
import numpy as np
from path import Path
from lateral_car import LateralCar
from pure_pursuit_controller import PurePursuitController

VELOCITY = 6.0                          # [m/s]
WHEELBASE = 1.6                         # [m]
MAX_STEERING_ANGLE = math.radians(30)   # [rad]
LOOKAHEAD = 50.0                         # [m]
DT = 0.1                                # [s]
MAX_SECONDS = 60

# Waypoints: a sine wave, 1 m spacing
xs = np.arange(0, 100, 1.0)
ys = 8 * np.sin(xs / 12)
path = Path(list(zip(xs, ys)))

car = LateralCar(velocity=VELOCITY,
                 wheelbase=WHEELBASE,
                 max_steering_angle=MAX_STEERING_ANGLE,
                 heading=0.0,
                 dt=DT,
                 start_x=0.0,
                 start_y=-3.0)
controller = PurePursuitController(path=path, lookahead=LOOKAHEAD)

STEPS: int = int(MAX_SECONDS / DT)

car_x:          list[float] = []
car_y:          list[float] = []
times:          list[float] = []
steering_deg:   list[float] = []

for step in range(STEPS):
    if path.points[-1].distance - path.points[controller.closest_index].distance < LOOKAHEAD:
        break

    desired_steering_angle = controller.calculate_desired_steering_angle(car=car)
    car_x.append(car.x)
    car_y.append(car.y)
    times.append(step * DT)
    steering_deg.append(math.degrees(np.clip(desired_steering_angle,
                                             -MAX_STEERING_ANGLE,
                                             MAX_STEERING_ANGLE)))
    car.update(desired_turn_angle=desired_steering_angle)


plt.plot([p.x for p in path.points], [p.y for p in path.points], 'g.', label="Path")
plt.plot(car_x, car_y, 'b-', label="Car")
plt.plot(car_x[0], car_y[0], 'ko', label="Start")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis("equal")
plt.legend()
plt.show()

plt.plot(times, steering_deg)
plt.ylabel("Steering angle [deg]")
plt.xlabel("Time")
plt.hlines(y=0, xmin=0, xmax=times[-1], colors='g', linestyles='solid')
plt.show()

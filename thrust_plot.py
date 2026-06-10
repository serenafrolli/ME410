import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("0010_thrust.csv", delimiter=",")
thrust_joystick = data[:, 0]
auto_thrust = data[:, 1]
camera_z = data[:, 2]
camera_x = data[:, 3]
camera_y = data[:, 4]
t = np.arange(len(data))

fig, ax1 = plt.subplots(figsize=(11, 5))
ax1.plot(t, thrust_joystick, label="joystick desired thrust")
ax1.plot(t, auto_thrust, label="camera auto thrust")
ax1.set_xlabel("loop iteration")
ax1.set_ylabel("thrust")
ax1.legend(loc="upper left")
ax1.grid(True)

ax2 = ax1.twinx()
ax2.plot(t, camera_z, color="green", label="camera z")
ax2.plot(t, camera_x, color="purple", label="camera x")
ax2.plot(t, camera_y, color="brown", label="camera y")
ax2.axhline(1.0, color="green", linestyle="--", alpha=0.5, label="desired z")
ax2.set_ylabel("camera position (m)")
ax2.legend(loc="upper right")

plt.title("Thrust inputs and camera x, y, z")
plt.tight_layout()
plt.show()

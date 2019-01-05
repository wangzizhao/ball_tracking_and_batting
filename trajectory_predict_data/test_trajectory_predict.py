import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt

filename1 = "trajectory.npy"
filename2 = "velocities.npy"
data = np.load(filename1)
velocity = np.load(filename2)
# print data

mpl.rcParams['legend.fontsize'] = 10

true_x = []
true_y = []
true_z = []

fig = plt.figure()
ax = fig.gca(projection='3d')
for trajectory in data:
	x = [Y[0] for Y in trajectory]
	y = [Y[1] for Y in trajectory]
	z = [Y[2] for Y in trajectory]
	if len(trajectory):
		true_x.append(trajectory[0][0])
		true_y.append(trajectory[0][1])
		true_z.append(trajectory[0][2])

	ax.plot(x, y, z)
	# plt.plot(y, z)

ax.plot(true_x, true_y, true_z, linewidth = 3, color="r")

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.legend()


# plt.plot(true_y, true_z)
# plt.xlabel('y')
# plt.ylabel('z')

fig = plt.figure()
plt.plot(range(len(velocity[0])), velocity[0])
plt.plot(range(len(velocity[2])), velocity[2])
plt.title('vy')

fig = plt.figure()
plt.plot(range(len(velocity[1])), velocity[1])
plt.plot(range(len(velocity[3])), velocity[3])
plt.title('vz')
plt.show()
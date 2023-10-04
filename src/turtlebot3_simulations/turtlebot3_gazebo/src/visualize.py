import numpy as np 
import matplotlib.pyplot as plt 

file_path = "run2.npy"

data = np.load(file_path)



x_robot= data[:, 0]
y_robot = data[:, 1]

x_traj = data[:, 2]
y_traj = data[:, 3]

plt.plot(x_robot, y_robot, 'x', 'blue')
plt.plot(x_traj, y_traj, 'red')

plt.title("robot trajectory and desired trajectory")
plt.legend(['robot trajector', 'desired trajectory'])

plt.xlabel("x")
plt.ylabel("y")
plt.show()
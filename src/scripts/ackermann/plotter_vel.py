#!/usr/bin/env python3
import matplotlib.pyplot as plt 
import numpy as np 


# x,y,h,time, active, detection(bool)
no = 3


# Loading robot logged parameters 
robotX, robotY, H, Time, Act, Det = [],[],[],[],[],[]
with open(f"./log/log{no}.txt", "r") as f:
    lines = f.readlines()

    for line in lines:
        x,y,h,t,a,d = list(map(float, line.split()))
        robotX.append(x)
        robotY.append(y)
        H.append(h)
        Time.append(t)
        Act.append(a)
        Det.append(d)

robotX, robotY, H, Time, Act, Det = np.array(robotX), np.array(robotY), np.array(H),np.array(Time),np.array(Act), np.array(Det)



T_obs, X_obs, Y_obs = [], [], []

with open(f"./log/obstaclepos{no}.txt", "r") as f:
    lines = f.readlines()
    for line in lines:
        if line.find("None") != -1:
            l = line.split()

            T_obs.append(float(l[0]))
            X_obs.append(-1)
            Y_obs.append(-1)
            continue
        t, x, y = list(map(float, line.split()))
        if not x == None and not y == None:
            T_obs.append(t)
            X_obs.append(x)
            Y_obs.append(y)


Tvel, Xvel, Yvel = [], [], []
Xvel_gt, Yvel_gt, X_gt, Y_gt = [],[],[],[]
with open(f"./log/obstaclevel{no}.txt", "r") as f:
    lines = f.readlines()
    for line in lines:
        if line.find("None") != -1:
            l = line.split()

            # Tvel.append(float(l[0]))
            # Xvel.append(0)
            # Yvel.append(0)
            continue
        t, x, y = list(map(float, line.split()))
        if not x == None and not y == None:
            
            Tvel.append(t)
            Xvel.append(x)
            Yvel.append(y)

# robotx, roboty, active = [], [], []

# with open(f"./log/robotpos{no}.txt", "r") as f:
#     lines = f.readlines()
#     for line in lines:
#         if line.find("None") != -1:
#             l = line.split()

#             # Tvel.append(float(l[0]))
#             # Xvel.append(0)
#             # Yvel.append(0)
#             continue
#         _, x, y, a = list(map(float, line.split()))
#         if not x == None and not y == None:
        
#             robotx.append(x)
#             roboty.append(y)
#             active.append(a)

# robotx = np.array(robotx)
# roboty = np.array(roboty)
# active = np.array(active)




# Plotting the Barrier function value vs time 
col = np.where(Act==1, 'r', 'b')
plt.scatter(Time[0],-10, c='y')
plt.scatter(Time[0],10,c='r')
plt.scatter(Time, H, c=col )
plt.ylim(-2.5, 2.5)
plt.legend(['cbf active', 'cbf inactive'])
plt.show()




# Plotting the Robot trajectory
plt.scatter(2,2,c='r')
col = np.where(Act==1, 'r', 'b')
plt.scatter(robotX, robotY, c=col)
plt.scatter(2,0,c='g', s=150)
plt.title("robotx vs roboty")
plt.legend(['cbf active', 'cbf inactive', 'obstacle'])
plt.ylim(-0.5,0.5)
plt.show()


#Pltting Obstacle Pos and Vel vs Time 
plt.subplot(2,2,1)
plt.plot(T_obs, X_obs, c='b')
# plt.plot(T, X_gt, c='b')

plt.title("X vs Time")
plt.xlim(1, 1.5)
plt.ylim(-1.2, 6)

plt.subplot(2,2,2)
plt.plot(T, Y, c='b')
# plt.plot(T, Y_gt, c='b')

# plt.xlim(1, 1.5)
plt.ylim(-1, 1  )
plt.title("Y vs Time")

T = [i for i in range(len(Xvel))]
plt.subplot(2,2,3)
plt.plot(Tvel, Xvel, c='b')

# plt.xlim(1, 1.5)
plt.ylim(-0.9, 0.1)
plt.title("Xvel vs Time")

plt.subplot(2,2,4)
plt.plot(Tvel, Yvel, c='b')
# plt.xlim(1, 1.5)
plt.ylim(-0.5, 0.5)
plt.title("Yvel vs Time")

plt.show()

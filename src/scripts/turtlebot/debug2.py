#!/usr/bin/env python3
import matplotlib.pyplot as plt 
import numpy as np 
import time 
import sys
sys.path.append("./Turtle-C3BF")
sys.path.append("./Turtle-C3BF/bots")
sys.path.append("./Turtle-C3BF/controllers")


from bots.AC_unicycle import Unicycle
from controllers.QP_controller_unicycle import QP_Controller_Unicycle
from tf2_msgs.msg import TFMessage
# x,y,h,time, active, detection(bool)
T, X, Y = [], [], []

# To check consistency of trjacectory static obs at 0.2 y 
no = 104


robotX, robotY, H, Time, Act, Det = [],[],[],[],[],[]
Acc, Alpha = [], []
with open(f"./runs/log{no}.txt", "r") as f:
    lines = f.readlines()

    for line in lines:
        x,y,h,t,a,d, acc,alpha = list(map(float, line.split()))
        robotX.append(x)
        robotY.append(y)
        H.append(h)
        Time.append(t)
        Act.append(a)
        Det.append(d)
        Acc.append(acc)
        Alpha.append(alpha)


botx, boty, bottheta, botv, botw, _,_, _, refa,refalpha, stara, staralpha, vtarget, w_target = [[] for i in range(14)]
with open(f"./runs/cbf{no}.txt", "r") as f:
    lines = f.readlines()

    for line in lines:
        x,y,theta, v,w, t, _,_, a, alpha, astar, alphastar, vt, wt = list(map(float, line.split()))
        botx.append(x)
        boty.append(y)
        bottheta.append(theta)
        botv.append(v)
        botw.append(w)
        refa.append(a)
        refalpha.append(alpha)
        stara.append(astar)
        staralpha.append(alphastar)



# Creating mirror about y 

bottheta_mir, boty_mir, botw_mir, refalpha_mir = -np.array(bottheta), -np.array(boty), -np.array(botw), -np.array(refalpha)

# plt.subplot(1,4,1)
# plt.plot(Time, boty)
# plt.plot(Time, boty_mir)
# plt.title("y vs time")


# plt.subplot(1,4,2)
# plt.plot(Time, botw)
# plt.plot(Time, botw_mir)
# plt.title("w vs time")

# plt.subplot(1,4,3)
# plt.plot(Time, bottheta)
# plt.plot(Time, bottheta_mir)
# plt.title("theta vs time")

# plt.subplot(1,4,4)
# plt.plot(Time, refalpha)
# plt.plot(Time, refalpha_mir)
# plt.title("refalpha vs time")


# plt.show()

# Getting cbf values 
qp = QP_Controller_Unicycle(1, obs_radius= 0.3)

bot1_config_file_path = './Turtle-C3BF/bots/bot_config/bot1.json'
bot = Unicycle.from_JSON(bot1_config_file_path)
abs_x, abs_y, obs_v_x, obs_v_y = [2, 0.0, -0,0]



stara_mir, staralpha_mir = [],[]
for id, (x,y,theta, v,w, a ,alpha) in enumerate(zip(botx, boty_mir, bottheta_mir, botv, botw_mir, refa, refalpha_mir)):
    bot.x, bot.y, bot.theta, bot.v, bot.w = x, y, theta, v, w
    u_ref = np.array([a, alpha])
    qp.set_reference_control(u_ref)
    qp.setup_QP(bot, [abs_x, abs_y], [obs_v_x, obs_v_y]) #  
    h  = qp.solve_QP(bot)
    u_star = qp.get_optimal_control() 
    stara_mir.append(u_star[0])
    staralpha_mir.append(u_star[1])

    if abs(u_star[0]) != abs(stara[id]) or abs(u_star[1]) != abs(staralpha[id]):
        print("- -------------")
        print("ogl bot", botx[id], boty[id], bottheta[id], botv[id], botw[id])
        print("mirror bot", x, y, theta, v , w)
        print()
        print("ogl ref", refa[id], refalpha[id])
        print("mirror ref", a, alpha)
        print("ogl star", stara[id], staralpha[id]) 
        print("mirror star", u_star[0], u_star[1])
        print("----------------")
plt.subplot(2,2,1)
plt.plot(Time, stara)
plt.plot(Time, stara_mir)
plt.legend(["reference", "mirror"])
plt.title("a vs time")

plt.subplot(2,2,2)
plt.plot(Time, staralpha)
plt.plot(Time, stara_mir)
plt.legend(["Ref", "mirror"])
plt.title("alpah vs time")

plt.subplot(2,2,3)
plt.plot(botx, boty)
plt.plot(botx, boty_mir)
plt.legend(["Ref", "mirror"])
plt.title("trajectory")


plt.show()













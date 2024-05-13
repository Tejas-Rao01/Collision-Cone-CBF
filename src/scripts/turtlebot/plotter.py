#!/usr/bin/env python3
import matplotlib.pyplot as plt 
import numpy as np 


# x,y,h,time, active, detection(bool)
T, X, Y = [], [], []

# To check consistency of trjacectory static obs at 0.2 y 
# run no 75,76,77



no = 9
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



robotX, robotY, H, Time, Act, Det = np.array(robotX), np.array(robotY), np.array(H),np.array(Time),np.array(Act), np.array(Det)
col = np.where(Det==0, 'y', np.where(Act == 1, 'r', 'b'))
Time = Time - Time[0]
plt.scatter(Time[0],-10, c='y')
plt.scatter(Time[0],10,c='r')
plt.scatter(Time[0],10,c='b')

# lab = np.where(Det==0, 'outside perception boundary', np.where(Act == 1, 'cbf active', 'cbf inactive'))
plt.scatter(Time, H, c=col )

# c2 = np.where(Act == 1, 'r', 'b')

# plt.scatter(Time[(Det == 0) & (Time <25)], H[(Det == 0) & (Time < 25)], c='y' )
# plt.scatter(Time[(Det == 1) & (Act  == 0)], H[(Det == 1) & (Act == 0)], c='b' )
# plt.scatter(Time[(Det == 1) & (Act  == 1)], H[(Det == 1) & (Act == 1)], c='r' )

# perception_boundary = H[(Det == 0) & (Time > 20) ]
# coefficients = np.polyfit(Time[(Det == 0) & (Time > 20)], perception_boundary, 2)

# # Create a polynomial function from the coefficients
# poly_function = np.poly1d(coefficients)
# points = poly_function(Time[(Det == 0) & (Time > 20)]) 
# points += + np.random.normal(0, 0.003, points.shape)
# plt.scatter(Time[(Det == 0) & (Time > 20)], points, c='y' )
# print(perception_boundary)

plt.ylim(-1, 1)
plt.legend(['obstacle outside perception boundary', 'cbf active', 'cbf inactive'])
# plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("H")
plt.title("Value of H vs Time")
plt.show()

plt.subplot(2,1,1)
plt.plot(Time, Acc)
plt.ylim(-0.5,0.5)

plt.title("v vs Time")
plt.subplot(2,1,2)
plt.plot(Time, Alpha)
plt.ylim(-0.5,0.5)

plt.title("w vs Time")
plt.show()

with open(f"./runs/obstaclepos{no}.txt", "r") as f:
    lines = f.readlines()
    for line in lines:
        if line.find("None") != -1:
            l = line.split()

            T.append(float(l[0]))
            X.append(-1)
            Y.append(-1)
            continue
        t, x, y = list(map(float, line.split()))
        if not x == None and not y == None:
            T.append(t)
            X.append(x)
            Y.append(y)

Tvel, Xvel, Yvel = [], [], []
Xvel_gt, Yvel_gt, X_gt, Y_gt = [],[],[],[]
with open(f"./runs/obstaclevel{no}.txt", "r") as f:
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
# T = [i for i in range(len(X))]
# y = [0.2872831636230383, 0.3672973956303658, 0.44772461751970755, 0.5201783565533453, -0.07737868812433255, -0.07748498148638243, 0.4751443518460361, -0.06199216514116165, -0.08790363887245506, -0.07661984378371868, -0.07657299702000046, -0.06511100330478098, -0.07504310544500754, -0.06364152212185845, 0.46274019327388094, 0.5284205128728479, -0.06246217293675726, -0.06362678075261458, 0.5216069624040227, 0.6378489723268498, 0.6462152530135259, 0.49751303944086755, -0.07663673841498839, -0.07122110013961995, -0.0661116801797581, -0.05965089781512709, -0.049924267422626, -0.05296315953098148, -0.06944746110853613, -0.07957088570940454, -0.08478863958181108, 0.4623366732424875, -0.06628743712263688, -0.07869857382118337, -0.06084394327478969, -0.06108689222403505, -0.06818987260105831, 0.3646005595789247, 0.3679841099179169, -0.04792004529299001, 0.4418630695941949, 0.46188932541344213, 0.44598804094757893, -0.03626700243841687, -0.04687995203554208, -0.05419841304833093, -0.06698929915503328, -0.0733408704849754, -0.05238682837792197, -0.05689561886372633, 0.45542983498964934, -0.031257489046229374, -0.02503979217668097, -0.04288115499628862, -0.03392274501099578, -0.038979753917751776, -0.017578357518623373, -0.026331899081234345, -0.02767962089692819, -0.030930891815239094, 0.4797507014924432, -0.031052509162353803, -0.04074331410104581, -0.03211071518486288, -0.030935596927889098, -0.03299970555005759, -0.025449795723795545, -0.03752056827302883, 0.5721293933269165, -0.029497598646820083, -0.035540109435238235, 0.36257917008201485, -0.015812094842934082, -0.01367318623882676, -0.004052307685793377, -0.008660499846551376, -0.011354753705578927, -0.015758188598648925, -0.016826985469829445, -0.01946593855797679, 0.4583913582317628, 0.45612807813958817, -0.03725514600354352, -0.037085007478904745, -0.03590520794425983, -0.03736144540037626, -0.023333004458127728, -0.018963560988471535, -0.015992918441200862, 0.5583511194504334, -0.018826759850971368, -0.020394911882337854, -0.01948979095343146, 0.5892789167393845, -0.020007035481423675, -0.018090341132750282, -0.02234321659978991, -0.023145032468020625, -0.03079552124793327, -0.043549143959459614, -0.04179817528958204, -0.034075442573804716, -0.020928278367541145, -0.01971568154282648, -0.01288671621006667, 0.0027026210535609943, -0.0006737214281828061, 3.514614732772912e-05, 0.00048729539553952523, -0.010334179621306588, -0.014059958075319743, -0.02001005224940194, -0.027558231485611553, -0.02459493356498024, -0.02893016559218043, -0.025770567867758283, -0.035321173896528994, -0.030103130308611795, -0.016842796290057818, -0.015020333338874491, -0.007166874532814367, -0.012549622290154158, -0.013634063476835542, -0.011258582269846721, -0.011231922273300823, -0.011916615047306683, -0.007720457232847821, -0.011048607987855225, -0.008270483678672055, -0.0036540458566034573, -0.006046303080984091, -0.0035130825148750127, -0.00794052075334064, -0.03904690227416199, -0.039073302758917135, -0.03655019943989936, -0.029517873075148346, -0.015978551673518363, -0.01635996604284346, -0.009267932723306458, -0.006869691373493252, -0.003465453003933418, -0.00632952817332154, -0.010823111493252494, -0.008686644828935185, -0.01827503936330171, -0.008592408397835835, -0.0063765266231151545, -0.00907374987615034, -0.009448618272941784, -0.014196918596107434, -0.014458629125620969, -0.02056588469351881, -0.012894773481085132, -0.013167565158414364, -0.008049984226634044, -0.007501562330541939, -0.008937165643937043, -0.012369966370329256, -0.012407406254855787, -0.00813141037813296, -0.0036645573999482635, 0.0010708645076698232, 0.0010850188943855605, -0.007669837399322425, -0.00810324984191645, -0.009860520549720548, -0.01811544104079839, -0.019244217434095115, -0.022644316905402705, -0.028267724407572612, -0.019129194602411928, -0.017879814863432413, -0.012797629797879953, -0.005514104962041783, -0.005538282577325449, 0.001724895331668112, -0.00035998111801364735, -0.005943859372087945, 2.9091573502150442e-05, -0.0015500442502878714, 0.007646897200893735, -0.002737500582229651, -0.00433988230731395, 0.0005313411293142626, -0.00799219870909272, -0.0079211684469629, -0.01667844003560671, -0.013211663957970713, -0.012183642274191832, -0.018988215829933455, -0.010468570151825094, -0.01058972198515318, -0.003278653974519965, -0.002999085482113493, -0.0009585749343611531, 0.00575496254829296, 0.005995679497427437, 0.0029489152376814204, -0.008478003894318334, -0.010753444831965083, -0.014485727300132804, -0.011342455405474683, -0.013949287568298347, -0.016998277177686504, -0.019663892024230523, -0.008672511882117198, -0.009269103499993218, -0.015290209732948042, -0.006759265975138028, 0.0003367348433815723, -0.0073744245033572665, -0.004234813850894676, -0.0026694265353689713, -0.0020131500467822847, 0.0021107231875731754, 0.005744830799308501, -0.001040375906040464, 0.005864742627143805, 0.006965648684449242, 0.002803739584430257, -0.0004322890181018545, 0.0008933418717241542, -0.005740799414459178, -0.011381677498303442, -0.0013526910830244788, -0.0041069542860483095, -0.0008740426658401404, -0.0007371816936018844, 0.0013736142314447093, 0.0023570877600598266, 0.0001460596990723032, -0.0006378192390905484, 0.00021731404401171251, -0.004376328126745425, -0.006214851926057815, -0.002721497285697114, -0.004783229539665279, -0.0023736598874456545, -0.0015041814571792988, 0.0032158339124372655, -0.005915727511636209, 0.004018035266527546, 0.003970896943335477, -0.005179407404878643, 0.0015299920257695438, 0.0017924194214048415, 0.0015885300456632237, 0.0025450716266428534, 0.0028754111463369647, 0.005471357970321557, 0.011883857038537336, 0.012009772510547773, 0.01006633905965254, 0.011031244090186135, 0.01138345801339275, 0.009038839733117175, 0.010800721493768707, 0.0023510372750103585, -0.0016923877141489393, -0.0014549881954190212, 0.0048091992139768485, 0.010671385098080705, 0.006279690575523751, 0.007104191214501761, 0.009733541355202314, 0.010125172735712902, 0.01019230653976044, 0.006634989808111714, 0.008422529554347634, 0.006461182914386853, 0.011629832780294428, 0.0125162326733325, 0.011183915803976367, 0.008653127108563582, 0.007392768955716252, 0.009685792853435028, 0.0064070070886753875, 0.005355848462551847, 0.005876671507976776, 0.006501352013460245, 0.008070687891051032, 0.010630977861434606, 0.006790710028190389]
# t = [i for i in range(len(y))]
# plt.plot(t, y)
# plt.show()

robotx, roboty, active = [], [], []
with open(f"./runs/gt{no}.txt", "r") as f:
    lines = f.readlines()
    for line in lines:
        _, x,y,vx,vy = list(map(float, line.split()))
        X_gt.append(x)
        Y_gt.append(y)
        Xvel_gt.append(vx)
        Yvel_gt.append(vy)

with open(f"./runs/robotpos{no}.txt", "r") as f:
    lines = f.readlines()
    for line in lines:
        if line.find("None") != -1:
            l = line.split()

            # Tvel.append(float(l[0]))
            # Xvel.append(0)
            # Yvel.append(0)
            continue
        _, x, y, a = list(map(float, line.split()))
        if not x == None and not y == None:
        
            robotx.append(x)
            roboty.append(y)
            active.append(a)

robotx = np.array(robotx)
roboty = np.array(roboty)
active = np.array(active)

plt.scatter(2,2,c='r')
plt.scatter(2,2,c='b')
plt.scatter(2,2,c='y')
col = np.where(active==1, 'r', 'b')
col = np.where(Det==0, 'y', np.where(Act == 1, 'r', 'b'))
plt.scatter(robotx, roboty, c=col)
# plt.scatter(2,0,c='g', s=150)
plt.title("robotx vs roboty")
plt.legend(['cbf active', 'cbf inactive','obstacle outside perception boundary' 'obstacle'])
plt.ylim(-1.5,1.5)
plt.show()

plt.subplot(2,2,1)
plt.plot(T, robotx, c ='g')
plt.plot(T, X, c='b')
plt.plot(T, X_gt, c='b')

plt.title("X vs Time")
# plt.xlim(1, 1.5)
plt.ylim(-1.2, 6)

plt.subplot(2,2,2)
plt.plot(T, Y, c='b')
plt.plot(T, Y_gt, c='b')
# plt.xlim(1, 1.5)
plt.ylim(-1, 1  )
plt.title("Y vs Time")

T = [i for i in range(len(Xvel))]
plt.subplot(2,2,3)
plt.plot(Tvel, Xvel, c='b')
plt.plot(Tvel, Xvel_gt, c='k')
# plt.xlim(1, 1.5)
plt.ylim(-0.9, 0.1)
plt.title("Xvel vs Time")

plt.subplot(2,2,4)
plt.plot(Tvel, Yvel, c='b')
plt.plot(Tvel, Yvel_gt, c='b')
# plt.xlim(1, 1.5)
plt.ylim(-0.5, 0.5)
plt.title("Yvel vs Time")

plt.show()

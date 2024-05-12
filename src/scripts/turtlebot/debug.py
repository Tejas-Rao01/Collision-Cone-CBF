#usr/bin/env python3 



file = "/home/rao/cbf_multi/Collision-Cone-CBF_multi/Collision-Cone-CBF/src/scripts/turtlebot/final_runs/multi/cbf237_best.txt"


with open(file) as f:
    lines = f.readlines()

data = [list(map(float, line.split())) for line in lines]


print(len(data[0]))




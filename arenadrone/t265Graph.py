import matplotlib.pyplot as plt
import sys
from matplotlib.ticker import StrMethodFormatter, MaxNLocator

fig = plt.figure()

param = "Z"
if len(sys.argv) > 1:
  param = sys.argv[1]

pfPosFile = open("./data/pfOut.csv", "r")
arduAttFile = open("./data/arduPilotAtt.csv", "r")
arduGpsFile = open("./data/arduPilotGps.csv", "r")
t265File = open("./data/t265Pos.csv", "r")

index = "XYZRPW".index(param)

pfPos = [[], [], [], []]
arduAtt = [[], [], [], []]
arduGps = [[], [], [], []]
for line in pfPosFile.readlines():
  s = line.split(",")
  pfPos[0].append(float(s[0]))
  pfPos[1].append(float(s[1])-0.15)
  pfPos[2].append(-float(s[2]))
  pfPos[3].append(-float(s[3]))
for line in arduAttFile.readlines():
  s = line.split(",")
  arduAtt[0].append(float(s[0]))
  arduAtt[1].append(float(s[1]))
  arduAtt[2].append(float(s[2]))
  arduAtt[3].append(float(s[3]))
for line in arduGpsFile.readlines():
  s = line.split(",")
  arduGps[0].append(float(s[0]))
  arduGps[1].append(float(s[1]))
  arduGps[2].append(float(s[2]))
  arduGps[3].append(float(s[3]))

t265 = [[], [], [], [], [], [], []]
for line in t265File:
  s = line.split(",")
  t265[0].append(float(s[0]))
  t265[1].append(float(s[1]))
  t265[2].append(float(s[2]))
  t265[3].append(float(s[3]))
  t265[4].append(float(s[4]))
  t265[5].append(float(s[5]))
  t265[6].append(float(s[6]))

time_init = arduGps[0][0]
pfPos[0] = [i - time_init for i in pfPos[0]]
arduAtt[0] = [i - time_init for i in arduAtt[0]]
arduGps[0] = [i - time_init for i in arduGps[0]]
t265[0] = [i - time_init for i in t265[0]]

if len(sys.argv) > 1 and len(sys.argv[1]) > 1:
  ax = fig.add_subplot(projection='3d')
  ax.plot3D(t265[1], t265[2], t265[3])
  ax.plot3D(pfPos[1], pfPos[2], pfPos[3])
  ax.plot3D(arduGps[1], arduGps[2], arduGps[3])

  ax.legend()
  ax.set_xlabel('X Position')
  ax.set_ylabel('Y Position')
  ax.set_zlabel('Z Position')
  plt.show()

else:
  ax = fig.add_subplot()
  ax.yaxis.set_major_formatter(StrMethodFormatter('{x:,.2f}')) # 2 decimal places
  ax.yaxis.set_major_locator(MaxNLocator(5))
  ax.plot(t265[0], t265[index+1], label='T265')
  if index > 2:
    ax.plot(arduAtt[0], arduAtt[index-3+1], label='arduPilot')
  else:
    ax.plot(pfPos[0], pfPos[index+1], label='Particle Filter')
    ax.plot(arduGps[0], arduGps[index+1], label='arduPilot GPS')

  plt.xlabel("time (seconds)")
  plt.ylabel("distance (meters)")

  plt.xlabel("time (seconds)")
  plt.ylabel("distance (meters)")
  plt.title(param+" Position")

  ax.legend()
  plt.show()

import numpy as np
import serial
import time
from particlefilter import ParticleFilterLoc

pf = ParticleFilterLoc()
uwb = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)

uwb_locations = {
  7: [1.828, 1.749, 2.558],
  8: [2.806, 0.799, -1.901],
  9: [-1.82, 0.168, 1.955],
  10: [-2.94, 2.804, -2.126],
  11: [1.334, 2.052, -3.678]
}

uwb.write(b"AT+RESET 20\n")
time.sleep(1)
uwb.write(b"AT+ID 25\n")
time.sleep(1)
uwb.write(b"AT+RATE 20\n")
time.sleep(1)
uwb.write(b"AT+STARTBLE\n")
time.sleep(1)
uwb.write(b"AT+STARTUWB\n")
time.sleep(1)
uwb.write(b"AT+STREAMMODE 0\n")
time.sleep(1)

uwb.readline()
count = 0
startTime = time.time()
while True:
  line = uwb.readline()
  print(line)
  if len(line) > 0 and (line[0] != ord('#')):
    line = line[0:-2].replace(b' ', b'').decode('utf-8')
    #ID, RANGE, RSSI, TIMESTAMP
    curData = line.split(',')
    if (len(curData) < 4):
      print(curData)
      continue
    if int(curData[0]) not in uwb_locations:
      continue
    x, y, z = uwb_locations[int(curData[0])]
    # print (x, y, z, float(curData[1]))
    curTime = time.time()
    # pf.depositVio(np.float64(curTime), 0, 0, 0, np.float32(0.5*(curTime - startTime)))

    pf.depositRange(np.float32(x), np.float32(y), np.float32(z), np.float32(float(curData[1])), 0.1)

    count += 1

    pf_data = pf.getTagLoc()
    # box.update_attributes(position=Position(x.value, y.value, z.value), rotation=Rotation(0, theta.value*RAD_TO_DEG, 0))
    # scene.update_object(box)
    print(pf_data[2],"\t", pf_data[3],"\t", pf_data[4])

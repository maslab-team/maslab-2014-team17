import time
import sys
sys.path.append("../../staff2013")
import arduino

# Example code to run an IR sensor. Turns on the LED
# at digital pin 13 when the IR sensor detects anything within
# a certain distance.

THRESH = 200.0  # Experimentally chosen

ard = arduino.Arduino()  # Create the Arduino object
a0 = arduino.AnalogInput(ard, 0)  # Create an analog sensor on pin A0
ard.run()  # Start the thread which communicates with the Arduino

# Main loop -- check the sensor and update the digital output

def irToInches(val, longRange=True):
  l = [ # ordered pair distance (inches), value; increasing order by value
    (9000, -1), #fake data to create lower bound
    (14, 352),
    (13, 380),
    (12, 405),
    (11, 432),
    (10, 462),
    ( 9, 502),
    ( 8, 527),
    ( 7, 557),
    ( 6, 581),
    ( 5, 592),
    ( 4, 592+(592-581)) #fake data to create upper bound
  ]
  s = [
    (0, -1), #fake data to create lower bound
    (2, 365),
    (3, 418),
    (4, 540),
    (5, 592),
    (6, 592+(592-540)) #fake data to create upper bound
  ]
  
  data = l if longRange else s

  #interpolate linearly between closest known measurements
  lower = 0
  for i in xrange(len(data)-1):
    if data[i][1] <= val:
      lower = i
  return data[lower][0] + (data[lower+1][0]-data[lower][0])*(val-data[lower][1])/float(data[lower+1][1]-data[lower][1])

while True:
    ir_val = a0.getValue() # Note -- the higher value, the *closer* the dist
    if ir_val!=None:
      print ir_val, irToInches(ir_val, False), irToInches(ir_val, True)
    time.sleep(0.1)

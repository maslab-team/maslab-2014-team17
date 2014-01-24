import sys
import time
import pygame #install pygame with sudo apt-get install python-pygame

pygame.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

if len(joysticks)==0:
  print "No joysticks found; exiting"
  sys.exit(1)

j = joysticks[0]
j.init()

sys.path.append("../../staff2013")
import arduino

ard = arduino.Arduino()
right = arduino.Motor(ard, 0, 0, 1) #unused, direction pin, pwm pin
left = arduino.Motor(ard, 0, 2, 3) #unused, direction pin, pwm pin
ard.run()  # Start the Arduino communication thread

while True:
  pygame.event.pump()
  turn, forward = j.get_axis(0)*.5,-j.get_axis(1)
  r = forward - turn
  l = forward + turn
  scale = max(r,l,1)
  r *= scale
  l *= scale

  right.setSpeed(int(1024*r))
  left.setSpeed(int(1024*l))
  
  time.sleep(.1)


#while True:
#    left.setSpeed(127)
#    time.sleep(1)
#    left.setSpeed(0)
#    time.sleep(1)
#    left.setSpeed(-127)
#    time.sleep(1)
#    left.setSpeed(0)
#    time.sleep(1)

import pygame #install pygame with sudo apt-get install python-pygame
import time
import sys

pygame.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

if len(joysticks)==0:
  print "No joysticks found; exiting"
  sys.exit(1)
  

j = joysticks[0]
j.init()

while True:
  pygame.event.pump()
  x,y = j.get_axis(0),-j.get_axis(1)
  print x,"\t",y
  time.sleep(.1)


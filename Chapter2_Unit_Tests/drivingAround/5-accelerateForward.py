from easygopigo3 import EasyGoPiGo3
from time import time, sleep

gpg = EasyGoPiGo3()

# setting speed to lowest value and
# calculating the step increase in speed
current_speed = 50
end_speed = 400
step = (end_speed - current_speed) / 20
gpg.set_speed(current_speed)

# start moving the robot at an ever increasing speed
gpg.forward()
while current_speed <= end_speed:
  # sleep(0.5) # Slow acceleration
  sleep(0.1) # Fast acceleration
  gpg.set_speed(current_speed)
  current_speed += step

# and then stop it
gpg.stop()
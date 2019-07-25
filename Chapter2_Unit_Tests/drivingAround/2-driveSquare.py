from easygopigo3 import EasyGoPiGo3

gpg = EasyGoPiGo3()
length = 30

for i in range(4):
  gpg.drive_cm(length) # drive forward for length cm
  gpg.turn_degrees(90) # rotate 90 degrees to the right
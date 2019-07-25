from easygopigo3 import EasyGoPiGo3

gpg = EasyGoPiGo3()
radius = 30

gpg.orbit(-270, radius) # to rotate to the left
gpg.drive_cm(radius * 2) # move forward
gpg.orbit(270, radius) # to rotate to the right
gpg.drive_cm(radius * 2) # move forward
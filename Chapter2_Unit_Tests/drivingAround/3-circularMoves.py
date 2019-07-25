from easygopigo3 import EasyGoPiGo3

gpg = EasyGoPiGo3()

gpg.orbit(180, 50) # draw half a circle
gpg.turn_degrees(180) # rotate the GoPiGo3 around
gpg.orbit(-180, 50) # return on the initial path
gpg.turn_degrees(180) # and put it in the initial position
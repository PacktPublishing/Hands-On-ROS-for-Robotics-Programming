# import the time library for the sleep function
import time
# import the GoPiGo3 drivers
from easygopigo3 import EasyGoPiGo3

# Create an instance of the GoPiGo3 class.
# GPG will be the GoPiGo3 object.
gpg = EasyGoPiGo3()

print("Move the motors forward freely for 1 second.")
gpg.forward()
time.sleep(1)
gpg.stop()

print("Stop the motors for 1 second.")
time.sleep(1)

print("Drive the motors 50 cm and then stop.")
gpg.drive_cm(50, True)
time.sleep(1)

print("Turn right 1 second.")
gpg.right()
time.sleep(1)

print("Turn left 1 second.")
gpg.left()
time.sleep(1)

print("Stop!")
gpg.stop()
print("Done!")
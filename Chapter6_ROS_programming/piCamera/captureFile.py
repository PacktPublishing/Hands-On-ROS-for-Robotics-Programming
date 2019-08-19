#!/usr/bin/python
# https://picamera.readthedocs.io/en/release-1.13/recipes1.html#capturing-to-a-file

from time import sleep
from picamera import PiCamera

camera = PiCamera()
camera.resolution = (1024, 768)
camera.start_preview()
# Camera warm-up time
sleep(2)
camera.capture('foo.jpg')

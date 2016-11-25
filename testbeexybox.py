#!/usr/bin/env python

import BeexyBox as bb
import ctypes as ct
from time import sleep

box = bb.BeexyBox()

#box_fout = bb.bxyCreate(1)
#print box_fout

class CustomInputEvent(bb.BeexyInputCallback):
    
    def at_input_event(self, t, state, fall, rise):
        form = "time = {}\t state = {}\t fall = {}\t rise= {}"
        print (form.format( t,state,fall,rise) )


print("bb.bxyHardwareVersion {}".format(box.hardware_version()) )
print("bb.bxyisOpen = {}".format(box.is_open()) )
box.open("/dev/ttyACM0", bb.BAUD_RATE_115200)
print("bb.bxyisOpen = {}".format(box.is_open()) )

box.enter_mode(bb.MODE_STANDBY, True)
sleep(1.0)

box.enter_mode(bb.MODE_SERVE, True)
box.register_input_event_callback(CustomInputEvent(box))

# just wait a little before exiting.
sleep(10)

box.enter_mode(bb.MODE_OFF, False)

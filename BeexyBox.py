#!/usr/bin/env python

#
# Copyright (C) 2016  Maarten Duijndam <m.j.a.duijndam@uu.nl>
#
# For the licence of the BeexyBox SDK see:
# Beexy - Behavioral Experiment Software http://www.beexy.nl
#
# This file is part of the python wrapper for the  BeexyBox SDK.
#
# This library is free software; you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published
# by the Free Software Foundation; either version 2.1 of the License, or
# (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this library; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


'''
This module contains the BeexyBox SDK wrapper for the Python programming
language. It has been written with the conventions for python in mind.
This module tries to comply with PEP8. So to check the style run:
pep8 --ignore=E221 BeexyBox.py
So E221 is ignored (multiple spaces before operator). I Think the constants
defined in this header stand out better when the = operator is in the
same column.
'''

import ctypes as ct
import abc


###############################################################################
#  Import constants from headers, not from the dll/so                         #
###############################################################################

BAUD_RATE_1200          = 1200
BAUD_RATE_2400          = 2400
BAUD_RATE_4800          = 4800
BAUD_RATE_9600          = 9600
BAUD_RATE_19200         = 19200
BAUD_RATE_38400         = 38400
BAUD_RATE_57600         = 57600
BAUD_RATE_115200        = 115200


# BeexyStatus_t
NO_ERROR = 0                     # Success.
INVALID_ARG = 1                  # Invalid argument passed.
OPEN_FAILED = 2                  # Unable to open BeexyBox.
OPEN_FAILED_NONE_FOUND = 3       # No BeexyBox devices detected.
OPEN_FAILED_MULTIPLE_FOUND = 4   # Multiple BeexyBox devices detected.
OPEN_FAILED_NO_MATCH = 5         # No matching BeexyBox device detected.
OPEN_FAILED_MULTIPLE_MATCH = 6   # Multiple matching BeexyBox devices detected.
NOT_OPEN = 7                     # BeexyBox device is not open.
CMD_FAILED = 8                   # BeexyBox failed to execute command.
CMD_FAILED_NOT_VALID = 9         # Command not valid in current mode.
CMD_FAILED_NOT_SUPPORTED = 10    # Command not supported.
CMD_FAILED_UNKNOWN = 11          # Unknown command.
CMD_FAILED_INV_ARG_SIZE = 12     # Wrong argument size in command.
CMD_FAILED_INV_ARG = 13          # Invalid argument in command.
READ_FAILED = 14                 # Read from BeexyBox failed.
READ_FAILED_INV_CHECKSUM = 15    # Read from BeexyBox failed (checksum error).
READ_FAILED_INV_REPLY_SIZE = 16  # Read from BeexyBox failed (invalid reply
#                                  size).
READ_FAILED_INV_DATA = 17        # Read from BeexyBox failed (invalid data
#                                  received).
READ_FAILED_TIMED_OUT = 18       # Read from BeexyBox timed out.
WRITE_FAILED = 19                # Write to BeexyBox failed.
THREAD_FAILED = 12               # Unable to create BeexyBox thread.

# Types for bxyQueryType2
# note order matters
QUERY_DEVICE_TYPE           = 0
QUERY_HARDWARE_VERSION      = QUERY_DEVICE_TYPE + 1
QUERY_FIRMWARE_VERSION      = QUERY_HARDWARE_VERSION + 1
QUERY_NUM_DIGITAL_INPUTS    = QUERY_FIRMWARE_VERSION + 1
QUERY_NUM_DIGITAL_OUTPUTS   = QUERY_NUM_DIGITAL_INPUTS + 1
QUERY_NUM_ANALOG_INPUTS     = QUERY_NUM_DIGITAL_OUTPUTS + 1
QUERY_NUM_ANALOG_OUTPUTS    = QUERY_NUM_ANALOG_INPUTS + 1
QUERY_NUM_GPIO              = QUERY_NUM_ANALOG_OUTPUTS + 1
QUERY_NUM_BUTTONS           = QUERY_NUM_GPIO + 1
QUERY_NUM_LEDS              = QUERY_NUM_BUTTONS + 1
QUERY_NUM_TRIGGERS          = QUERY_NUM_LEDS + 1
QUERY_NUM_VOICES            = QUERY_NUM_TRIGGERS + 1
QUERY_NUM_TIMERS            = QUERY_NUM_VOICES + 1
QUERY_RS232                 = QUERY_NUM_TIMERS + 1
QUERY_RS485                 = QUERY_RS232 + 1
QUERY_LAN                   = QUERY_RS485 + 1
QUERY_WLAN                  = QUERY_LAN + 1
QUERY_SPEAKER               = QUERY_WLAN + 1
QUERY_MICROPHONE            = QUERY_SPEAKER + 1
QUERY_VOICE_KEY             = QUERY_MICROPHONE + 1
QUERY_MAC1                  = QUERY_VOICE_KEY + 1
QUERY_MAC2                  = QUERY_MAC1 + 1
QUERY_CHIP_ID1              = QUERY_MAC2 + 1
QUERY_CHIP_ID2              = QUERY_CHIP_ID1 + 1
QUERY_CHIP_ID3              = QUERY_CHIP_ID2 + 1
QUERY_CHIP_ID4              = QUERY_CHIP_ID3 + 1
QUERY_VREF                  = QUERY_CHIP_ID4 + 1
QUERY_TEMPERATURE           = QUERY_VREF + 1
QUERY_UPTIME                = QUERY_TEMPERATURE + 1
QUERY_CLOCK_DRIFT           = QUERY_UPTIME + 1

# Types for bxyQueryType2
# note order matters.
QUERY_VENDOR_NAME           = 0
QUERY_VENDOR_URL            = QUERY_VENDOR_NAME + 1
QUERY_VENDOR_EMAIL          = QUERY_VENDOR_URL + 1
QUERY_VENDOR_PHONE          = QUERY_VENDOR_EMAIL + 1
QUERY_SERIAL_NUMBE          = QUERY_VENDOR_PHONE + 1

# constants for BxyDeviceType
DEVICE_A = 0
DEVICE_B = 1
DEVICE_X = 2

# Modi operandi
MODE_OFF = 0
MODE_STANDBY        = MODE_OFF + 1
MODE_TEST_INPUTS    = MODE_STANDBY + 1
MODE_TEST_VOICE_KEY = MODE_TEST_INPUTS + 1
MODE_TEST_HID       = MODE_TEST_VOICE_KEY + 1
MODE_SERVE          = MODE_TEST_HID + 1
MODE_SERVE_HID      = MODE_SERVE + 1
MODE_SERVE_XHID     = MODE_SERVE_HID + 1


###############################################################################
# Extraction of all the functions from the shared object.                     #
###############################################################################

# TODO Make this work on at least mac, windows and Linux.
libBB = ct.cdll.LoadLibrary('libbeexybox.so')

# General functions
# functions related to loading functions from the shared object.

# Functions from debug.h
enable_warnings         = libBB.bxyEnableWarnings
enable_warnings.restype = None
enable_warnings.argtypes = ct.c_bool,

error_string            = libBB.bxyErrorString
error_string.restype    = ct.c_char_p
error_string.argtypes   = ct.c_int,

###############################################################################
#                                                                             #
# Definition of the callback functions to retrieve event from a BeexyBox.     #
#                                                                             #
# IMPORTANT Note all callbacks live in another thread.                        #
#                                                                             #
###############################################################################

# input event function.
INPUT_EVENT_FUNC = ct.CFUNCTYPE(None,
                                ct.c_void_p,      # the low level beexybox
                                ct.c_longlong,    # time
                                ct.c_uint,        # state
                                ct.c_uint,        # fall
                                ct.c_uint         # rise
                                )

# voice key event
VOICE_KEY_EVENT_FUNC = ct.CFUNCTYPE(None,
                                    ct.c_void_p,      # the low level beexybox
                                    ct.c_longlong,    # time
                                    ct.c_uint,        # action
                                    ct.c_longlong     # detected time
                                    )


###############################################################################
# Here we extract all functions related to the BeexyBox Type.                 #
###############################################################################

bxyCreate               = libBB.bxyCreate
bxyCreate.restype       = ct.c_void_p
bxyCreate.argtypes      = tuple()

bxyDestroy              = libBB.bxyDestroy
bxyDestroy.restype      = None
bxyDestroy.argtypes     = ct.c_void_p,

bxyOpen                 = libBB.bxyOpen
bxyOpen.restype         = ct.c_bool
bxyOpen.argtypes        = ct.c_void_p, ct.c_char_p, ct.c_int

bxyScanOpen             = libBB.bxyScanOpen
bxyScanOpen.restype     = ct.c_int
bxyScanOpen.argtypes    = ct.c_void_p, ct.c_char_p, ct.c_int

bxyClose                = libBB.bxyClose
bxyClose.restype        = None
bxyClose.argtypes       = ct.c_void_p,

bxyIsOpen               = libBB.bxyIsOpen
bxyIsOpen.restype       = ct.c_bool
bxyIsOpen.argtypes      = ct.c_void_p,

bxyIsTrueSerial         = libBB.bxyIsTrueSerial
bxyIsTrueSerial.restype = ct.c_bool
bxyIsTrueSerial.argtypes = ct.c_void_p,

bxyDeviceType           = libBB.bxyDeviceType
bxyDeviceType.restype   = ct.c_int
bxyDeviceType.argtypes  = ct.c_void_p,

bxyHardwareVersion          = libBB.bxyHardwareVersion
bxyHardwareVersion.restype  = ct.c_int
bxyHardwareVersion.argtypes = ct.c_void_p,

bxyFirmwareVersion          = libBB.bxyFirmwareVersion
bxyFirmwareVersion.restype  = ct.c_int
bxyFirmwareVersion.argtypes = ct.c_void_p,

bxySerialNumber         = libBB.bxySerialNumber
bxySerialNumber.restype = ct.c_char_p
bxySerialNumber.argtypes = ct.c_void_p,

bxyPing                 = libBB.bxyPing
bxyPing.restype         = ct.c_int
bxyPing.argtypes        = ct.c_void_p, ct.c_bool

bxyPlayTune             = libBB.bxyPlayTune
bxyPlayTune.restype     = ct.c_int
bxyPlayTune.argtypes    = ct.c_void_p, ct.c_bool

bxyReset                = libBB.bxyReset
bxyReset.restype        = ct.c_int
bxyReset.argtypes       = ct.c_void_p, ct.c_bool

bxyEnterMode            = libBB.bxyEnterMode
bxyEnterMode.restype    = ct.c_int
bxyEnterMode.argtypes   = ct.c_void_p, ct.c_int, ct.c_bool

bxyQueryInfo            = libBB.bxyQueryInfo
bxyQueryInfo.restype    = ct.c_int
# beeyxbox, enume bxyQueryType, pointer to unsigned int
bxyQueryInfo.argtypes   = ct.c_void_p, ct.c_int, ct.c_void_p

bxyQueryInfo2           = libBB.bxyQueryInfo2
bxyQueryInfo2.restype   = ct.c_int
# beeyxbox, enum bxyQuery2Type, char buffer, buffer length
bxyQueryInfo2.argtypes  = ct.c_void_p, ct.c_int, ct.c_void_p, ct.c_int

###############################################################################
# Setting the callback functions for the beexybox.                            #
###############################################################################

bxyRegisterInputEventCallback   = libBB.bxyRegisterInputEventCallback
bxyRegisterInputEventCallback.restype   = ct.c_int
bxyRegisterInputEventCallback.argtypes  = ct.c_void_p, INPUT_EVENT_FUNC

bxyRegisterVoiceKeyEventCallback = libBB.bxyRegisterVoiceKeyEventCallback
bxyRegisterVoiceKeyEventCallback.restype = ct.c_int
bxyRegisterVoiceKeyEventCallback.argtypes = ct.c_void_p, VOICE_KEY_EVENT_FUNC

###############################################################################
# Definition of the runtime classes to be used by the clients of this wrapper.#
###############################################################################


class BeexyException (Exception):

    ''' This is the exception that is raised when communication with the
    beexybox goes wrong.
    '''

    def __init__(self, BxyStatus):
        self.status = BxyStatus

    def __str__(self):
        super(BeexyException, self).__str__() + \
            ":{}".format(error_string(self.status))


class BeexyInputCallback(object):

    '''Abstract base class for input callbacks.'''

    __metaclass__ = abc.ABCMeta

    def __init__(self, box):
        ''' Initializes a callback functor. @param box A beexybox instance. '''
        self.box = box

    @abc.abstractmethod
    def at_input_event(self, t, state, fall, rise):
        '''This method is called when an input event occurs
        This must be implemented by a derived class. This method is
        called from another thread, not the one where the BeexyBox instance
        lives.
        '''
        return

    def __call__(self, lowlevel, t, state, fall, rise):
        ''' Calls the at_input_event method. '''
        print ("Event")
        self.at_input_event(t, state, fall, rise)


class BeexyBox (object):

    ''' A class that wraps the ctypes in a pythonic class '''

    def __init__(self, devicefn="", serial="", baudrate=BAUD_RATE_115200):
        ''' Initializes a BeexyBox, if device or serial is given. '''
        self.bb = bxyCreate()
        if not self.bb:
            raise MemoryError("Unable to allocate BeexyBox.")

        if devicefn:
            self.open(devicefn, baudrate)
        elif serial:
            self.scan_open(self.bb, -1, serial)

    def __del__(self):
        ''' free the beexy box '''
        if self.bb and self.is_open():
            self.close()
        bxyDestroy(self.bb)
        self.bb = ct.c_void_p(0x0)

    def open(self, devicefn, baudrate=BAUD_RATE_115200):
        ''' Open the beexy box '''
        assert(self.bb)
        status = bxyOpen(self.bb, devicefn, baudrate)
        if status:
            raise BeexyException(status)

    def scan_open(self, bbtype=-1, serial=""):
        ''' scan for open devices '''
        assert(self.bb)
        status = bxyScanOpen(self.bb, bbtype, serial)
        if status:
            raise BeexyException(status)

    def close(self):
        ''' Closes the device '''
        assert(self.bb)
        bxyClose(self.bb)

    def is_open(self):
        ''' Checks whether the device is open '''
        assert(self.bb)
        return bxyIsOpen(self.bb)

    def is_true_serial(self):
        ''' Returns whether the device is a true serial device. '''
        assert(self.bb)
        return bxyIsTrueSerial(self.bb)

    def device_type(self):
        ''' Returns the device type '''
        assert(self.bb)
        return bxyDeviceType(self.bb)

    def hardware_version(self):
        ''' Returns the hardware version. '''
        assert(self.bb)
        return bxyHardwareVersion(self.bb)

    def firmware_version(self):
        ''' Returns the firmware version. '''
        assert(self.bb)
        return bxyFirmwareVersion(self.bb)

    def serial_number(self):
        ''' Returns the firmware version. '''
        assert(self.bb)
        return bxySerialNumber(self.bb)

    def ping(self, ack):
        '''
            Pings the device
            @param ack (Bool) whether you want an acknowledgment
        '''
        assert(self.bb)
        status = bxyPing(self.bb, ack)
        if status:
            raise BeexyException(status)

    def play_tune(self, ack):
        '''
            Plays the tune.
            @param ack (Bool) whether you want an acknowledgment
        '''
        assert(self.bb)
        status = bxyPlayTune(self.bb, ack)
        if status:
            raise BeexyException(status)

    def reset(self, ack):
        '''
            resets the device.
            @param ack (Bool) whether you want an acknowledgment
        '''
        assert(self.bb)
        status = bxyReset(self.bb, ack)
        if status:
            raise BeexyException(status)

    def enter_mode(self, mode, ack):
        '''
            Sets the mode the device is in.
            @param ack (Bool) whether you want an acknowledgment
        '''
        assert(self.bb)
        status = bxyEnterMode(self.bb, mode, ack)
        if status:
            raise BeexyException(status)




    # Registration of callbacks

    def register_input_event_callback(self, functor):
        assert(self.bb)
        callback = INPUT_EVENT_FUNC(functor)
        status = bxyRegisterInputEventCallback(self.bb, callback)
        if status:
            raise BeexyException(status)

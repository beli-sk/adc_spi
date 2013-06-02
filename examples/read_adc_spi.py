#!/usr/bin/python
#
# read_adc_spi.py

# This file is part of ADC SPI
# 
# Copyright 2013, Michal Belica <devel@beli.sk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#-------------------------------------------------#
# Example of reading ADC over SPI on Raspberry PI #
#-------------------------------------------------#
# 
# Requires python-spi module available at https://github.com/beli-sk/python-spi
#
import spi
import time
import struct
import RPi.GPIO as GPIO

SS_PIN = 25

# GPIO init
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SS_PIN, GPIO.OUT, initial=GPIO.HIGH)

# SPI init
spicon = spi.SPI("/dev/spidev0.0")
spicon.set_speed(10000)
spicon.set_mode(spi.SPI_MODE_2)

# pull SS low to start ADC and wait
GPIO.output(SS_PIN, GPIO.LOW)
time.sleep(0.1)

# receive data
send = struct.pack(">H", 0)
recv = spicon.transfer(send)
Vraw = struct.unpack(">H", recv)[0]

# put SS high again
GPIO.output(SS_PIN, GPIO.HIGH)

# present data

if Vraw == 0xffff:
	print "Value not ready"
elif Vraw > 0x03ff:
	print "Value read error (%d)" % Vraw
else:
	V = 2.56 / 1023.0 * float(Vraw)
	print "%.3f V (raw = %4d)" % (V, Vraw)

#GPIO.cleanup()

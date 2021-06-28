# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Raspberry Pi Sense HAT Emulator library for the Raspberry Pi
# Copyright (c) 2021 Raspberry Pi Foundation <info@raspberrypi.org>
#
# This package is free software; you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License as published by the Free
# Software Foundation; either version 2.1 of the License, or (at your option)
# any later version.
#
# This package is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>

"""
Python library for the TCS34725 Color Sensor
Documentation (including datasheet): https://ams.com/tcs34725#tab/documents
"""

import io, os, sys, errno
import mmap
from struct import Struct
from collections import namedtuple

from time import sleep

_error_str = "Failed to initialise TCS34725 colour sensor."

class HardwareInterface:

    GAIN_VALUES = (1, 4, 16, 60)
    CLOCK_STEP = 0.0024 # the clock step is 2.4ms

    @staticmethod
    def max_value(integration_cycles):
        return 2**16 if integration_cycles >= 64 else 1024*integration_cycles

    def get_enabled(self):
        raise NotImplementedError

    def set_enabled(self, value):
        raise NotImplementedError

    def get_gain(self):
        raise NotImplementedError

    def set_gain(self, value):
        raise NotImplementedError

    def get_integration_cycles(self):
        raise NotImplementedError

    def set_integration_cycles(self, value):
        raise NotImplementedError

    def get_raw(self):
        raise NotImplementedError

    def get_red(self):
        raise NotImplementedError

    def get_green(self):
        raise NotImplementedError

    def get_blue(self):
        raise NotImplementedError

    def get_clear(self):
        raise NotImplementedError

class StatusFile(HardwareInterface):

    COLOUR_DATA = Struct(str(
        '@'   # native mode
        '?'   # enabled
        '4I'  # RGBC
        'H'   # gain
        'H'   # integration cycles
        ))

    ColourData = namedtuple(
        'ColourData', (
            'enabled', 'R', 'G', 'B', 'C', 'gain', 'integration_cycles'))

    def __init__(self):
        self.init_file()

    @staticmethod
    def filename():
        """
        Return the filename used to represent the state of the emulated sense HAT's
        co sensor. On UNIX we try ``/dev/shm`` then fall back to ``/tmp``; on
        Windows we use whatever ``%TEMP%`` contains
        """
        fname = 'rpi-sense-emu-colour'
        if sys.platform.startswith('win'):
            # just use a temporary file on Windows
            return os.path.join(os.environ['TEMP'], fname)
        else:
            if os.path.exists('/dev/shm'):
                return os.path.join('/dev/shm', fname)
            else:
                return os.path.join('/tmp', fname)

    def init_file(self):
        """
        Opens the file representing the state of the colour sensor. The
        file-like object is returned.
        If the file already exists we simply make sure it is the right size. If
        the file does not already exist, it is created and zeroed.
        """
        try:
            # Attempt to open the colour sensor's file and ensure it's the right size
            fd = io.open(self.filename(), 'r+b', buffering=0)
            fd.seek(self.COLOUR_DATA.size)
            fd.truncate()
        except IOError as e:
            # If the colour sensor device file doesn't exist, zero it into existence
            if e.errno == errno.ENOENT:
                fd = io.open(self.filename(), 'w+b', buffering=0)
                fd.write(b'\x00' * self.COLOUR_DATA.size)
            else:
                raise IOError from e
        else:
            self._fd = fd
            self._map = mmap.mmap(self._fd.fileno(), 0, access=mmap.ACCESS_WRITE)

    def close(self):
        self._map.close()
        self._fd.close()
        self._fd = None
        self._map = None

    def read(self):
        return self.ColourData(*self.COLOUR_DATA.unpack_from(self._map))

    def write(self, value):
        self.COLOUR_DATA.pack_into(self._map, 0, *value)

    def get_enabled(self):
        return self.read().enabled

    def set_enabled(self, value):
        self.write(self.read()._replace(enabled=value))

    def get_gain(self):
        return self.read().gain

    def set_gain(self, value):
        self.write(self.read()._replace(gain=value))

    def get_integration_cycles(self):
        return self.read().integration_cycles

    def set_integration_cycles(self, cycles):
        status = self.read()
        prev_cycles = status.integration_cycles
        scaling = self.max_value(cycles) // self.max_value(prev_cycles)
        self.write(status._replace(
            integration_cycles=cycles,
            R=status.R * scaling,
            G=status.G * scaling,
            B=status.B * scaling
        ))

    def get_raw(self):
        _, R, G, B, C, _, _ = self.read()
        return (R, G, B, C)

    def get_red(self):
        return self.read().R

    def get_green(self):
        return self.read().G

    def get_blue(self):
        return self.read().B

    def get_clear(self):
        return self.read().C


class ColourSensor:
    
    def __init__(self, gain=1, integration_cycles=1):
        self.interface = StatusFile()
        self.gain = gain
        self.integration_cycles = integration_cycles
        self.enabled = 1

    @property
    def enabled(self):
        return self.interface.get_enabled()

    @enabled.setter
    def enabled(self, status):
        self.interface.set_enabled(status)

    @property
    def gain(self):
        return self.interface.get_gain()

    @gain.setter
    def gain(self, value):
        if value in self.interface.GAIN_VALUES:
            self.interface.set_gain(value)
        else:
            raise RuntimeError(f'Cannot set gain to {value}. Values: {self.interface.GAIN_VALUES}')

    @property
    def integration_cycles(self):
        return self.interface.get_integration_cycles()

    @integration_cycles.setter
    def integration_cycles(self, cycles):
        if 1 <= cycles <= 256:
            self.interface.set_integration_cycles(cycles)
            self._integration_time = cycles * self.interface.CLOCK_STEP
            self._max_value = self.interface.max_value(cycles)
            self._scaling = self._max_value // 256
            sleep(self.interface.CLOCK_STEP)
        else:
            raise RuntimeError(f'Cannot set integration cycles to {cycles} (1-256)')

    @property
    def integration_time(self):
        return self._integration_time

    @property
    def max_raw(self):
        return self._max_value

    @property
    def colour_raw(self):
        return self.interface.get_raw()

    @property
    def colour(self):
        return tuple(reading // self._scaling for reading in self.colour_raw)

    color_raw = colour_raw
    color = colour

    # For the following, could also use something like:
    # red_raw = property(lambda self: self.interface.get_red())

    @property
    def red_raw(self):
        return self.interface.get_red()
    
    @property
    def green_raw(self):
        return self.interface.get_green()

    @property
    def blue_raw(self):
        return self.interface.get_blue()

    @property
    def clear_raw(self):
        return self.interface.get_clear()

    @property
    def red(self):
        return self.red_raw // self._scaling
    
    @property
    def green(self):
        return self.green_raw // self._scaling

    @property
    def blue(self):
        return self.blue_raw // self._scaling

    @property
    def clear(self):
        return self.clear_raw // self._scaling

######

import time
import subprocess
from random import Random
from threading import Thread, Event
import numpy as np

class ColourServer:

    def __init__(self):
        self.interface = StatusFile()

        # The queue lengths are selected to accurately represent the response
        # time of the sensors
        # self._colours = np.full((10,), self._humidity, dtype=np.float)
        
    def close(self):
        if self.interface:
            self.interface.close()

    def _read(self):
        return self.interface.read()

    def _write(self, value):
        self.interface.write(value)

    def set_colour(self, R, G, B):
        status = self.interface.read()
        cycles = status.integration_cycles
        max_value = self.interface.max_value(cycles)
        scaling = max_value // 256
        self.interface.write(status._replace(
            R=int(R * scaling), 
            G=int(G * scaling), 
            B=int(B * scaling)))

# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Raspberry Pi Sense HAT Emulator library for the Raspberry Pi
# Copyright (c) 2016 Raspberry Pi Foundation <info@raspberrypi.org>
#
# This package is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later
# version.
#
# This package is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <http://www.gnu.org/licenses/>

from __future__ import (
    unicode_literals,
    absolute_import,
    print_function,
    division,
    )
str = type('')


import sys
import os
import csv
import logging
import argparse
from datetime import datetime
from time import time, sleep
from struct import Struct

from . import __version__
from .i18n import _
from .terminal import TerminalApplication, FileType
from .recording import Recording


class DumpApplication(TerminalApplication):
    def __init__(self):
        super(DumpApplication, self).__init__(
            version=__version__,
            description=_("Converts a Sense HAT recording to CSV format, for "
                "the purposes of debugging or analysis."))
        self.parser.add_argument(
            '--timestamp-format', action='store', default='%Y-%m-%dT%H:%M:%S.%f', metavar='FMT',
            help=_('the format to use when outputting the record timestamp '
            '(default: %(default)s)'))
        self.parser.add_argument(
            '--header', action='store_true', default=False,
            help=_('if specified, output column headers'))
        self.parser.add_argument('input')
        # Eurgh ... csv module under Python 2 only outputs byte-strings
        if sys.version_info.major == 2:
            self.parser.add_argument('output', type=FileType('wb'))
        else:
            self.parser.add_argument('output', type=FileType('w', encoding='utf-8'))

    def main(self, args):
        logging.info(_('Reading header'))
        recording = Recording(args.input)
        writer = csv.writer(args.output)
        logging.info(
            _('Dumping recording taken at %s'),
            datetime.fromtimestamp(recording.offset).strftime('%c'))
        if args.header:
            header = (
                'timestamp',
                'pressure', 'pressure_temp',
                'humidity', 'humidity_temp',
                'accel_x', 'accel_y', 'accel_z',
                'gyro_x', 'gyro_y', 'gyro_z',
                'compass_x', 'compass_y', 'compass_z',
                'orient_x', 'orient_y', 'orient_z',
            )
            if recording.version == 2:
                header = header + (
                    'R', 'G', 'B', 'C',
                    'gain',
                    'integration_cycles'
                )
            writer.writerow(header)
        print(args.timestamp_format)
        for rec, data in enumerate(recording.read_rows()):
            data = data._replace(
                timestamp = datetime.fromtimestamp(data.timestamp).strftime(args.timestamp_format)
            )
            writer.writerow(data)
        logging.info(_('Converted %d records'), rec)


app = DumpApplication()

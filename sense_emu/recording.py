from collections import namedtuple
from typing import Sized
from sense_emu.common import DATA_REC, HEADER_REC
from struct import Struct
from time import time
import io

from .terminal import FileType

class Template_v1:

    DataRecord = namedtuple('DataRecord', (
        'timestamp',
        'pressure', 'ptemp',
        'humidity', 'htemp',
        'ax', 'ay', 'az',
        'gx', 'gy', 'gz',
        'cx', 'cy', 'cz',
        'ox', 'oy', 'oz'
    ))

    data_format = str(
        '='   # native order, standard sizing
        'd'   # timestamp
        'dd'  # pressure+temp readings
        'dd'  # humidity+temp readings
        'ddd' # raw accelerometer readings
        'ddd' # raw gyro readings
        'ddd' # raw compass readings
        'ddd' # calculated pose
    )

    DATA_REC = Struct(data_format)

class Template_v2:

    DataRecord = namedtuple('DataRecord', 
        Template_v1.DataRecord._fields + (
        'R', 'G', 'B', 'C',
        'gain',
        'integration_cycles'
    ))

    data_format = Template_v1.data_format + str(
        '4I'  # colour sensor raw RGBC readings
        'H'   # colour sensor gain
        'H'   # colour sensor integration cycles
    )

    DATA_REC = Struct(data_format)


class Recording:

    Header = namedtuple('Header', (
        'version',
        'timestamp',
    ))

    header_format = str(
        '='  # native order, standard sizing
        '8s' # magic number ("SENSEHAT")
        'b'  # version number (1 or 2)
        '7x' # padding
        'd'  # initial timestamp
    )

    HEADER_REC = Struct(header_format)

    def __init__(self, filename, write=False, version=1):
        if write:
            mode = 'wb'
            self.file = FileType(mode)(filename)
            self.write_header(self.file, version)
            self.version = version
        else:
            mode = 'rb'
            self.file = FileType(mode)(filename)
            self.version, self.offset = self.read_header(self.file)
        if self.version == 1:
            self.template = Template_v1
        elif self.version == 2:
            self.template = Template_v2

    @classmethod
    def read_header(cls, file):
        header = file.read(cls.HEADER_REC.size)
        magic, ver, offset = cls.HEADER_REC.unpack(header)
        if magic != b'SENSEHAT':
            raise IOError(f'{file.name} is not a Sense HAT recording')
        if ver > 2:
            raise IOError(f'{file.name} has unrecognized file version number')
        return ver, offset

    @classmethod
    def write_header(cls, file, ver):
        file.write(cls.HEADER_REC.pack(b'SENSEHAT', ver, time()))

    def read_row(self):
        return self.file.read(self.template.DATA_REC.size)

    def read_rows(self):
        self.file.seek(self.header_size)
        offset = time() - self.offset
        while True:
            buf = self.read_row()
            if not buf:
                break
            elif len(buf) < self.template.DATA_REC.size:
                raise IOError(f'Incomplete data record at end of {self.file.name}')
            else:
                record = self.template.DataRecord(*self.template.DATA_REC.unpack(buf))
                yield record._replace(timestamp=record.timestamp + offset)

    def write_row(self, record, flush=False):
        self.file.write(self.template.DATA_REC.pack(*record))
        if flush:
            self.file.flush()

    def close(self):
        self.file.close()

    @property
    def header_size(self):
        return self.HEADER_REC.size

    @property
    def record_size(self):
        return self.template.DATA_REC.size

    @property
    def nb_records(self):
        return (self.file.seek(0, io.SEEK_END) - self.header_size) // self.record_size

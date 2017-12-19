#!/usr/bin/env python
# Copyright 2017 Matthew Wall, all rights reserved
"""
Collect data from Vaisala WXT510 or WXT520 station.

http://www.vaisala.com/Vaisala%20Documents/User%20Guides%20and%20Quick%20Ref%20Guides/M210906EN-C.pdf

The WXT520 is available with the following serial communications:
 - RS232: ASCII automatic and polled; NMEA0183 v3; SDI12 v1.3
 - RS485: ASCII automatic and polled; NMEA0183 v3; SDI12 v1.3
 - RS422: ASCII automatic and polled; NMEA0183 v3; SDI12 v1.3
 - SDI12: v1.3 and v1.3 continuous
"""

from __future__ import with_statement
import syslog
import time

import weewx.drivers

DRIVER_NAME = 'WXT5x0'
DRIVER_VERSION = '0.1'

def loader(config_dict, _):
    return WXT5x0Driver(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return WXT5x0ConfigurationEditor()


def logmsg(level, msg):
    syslog.syslog(level, 'wxt5x0: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)


class WXT5x0ConfigurationEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[WXT5x0]
    # This section is for Vaisala WXT5x0 stations

    # The station model such as WXT510 or WXT520
    model = WXT520

    # The serial port to which the station is connected
    port = /dev/ttyUSB0

    # The communication protocol to use, one of serial, nmea0183, or sdi12
    protocol = serial

    # The driver to use
    driver = weewx.drivers.wxt5x0
"""

    def prompt_for_settings(self):
        print "Specify the serial port on which the station is connected, for"
        print "example /dev/ttyUSB0 or /dev/ttyS0."
        port = self._prompt('port', '/dev/ttyUSB0')
        print "Specify the protocol"
        protocol = self._prompt('protocol', 'serial', ['serial', 'nmea0183', 'sdi12'])
        return {'port': port, 'protocol': protocol}


class WXT5x0Driver(weewx.drivers.AbstractDevice):
    BAUD = {
        'sdi12': 1200,
        'nmea0183': 4800,
    }
    DEFAULT_PORT = '/dev/ttyUSB0'

    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        self._model = stn_dict.get('model', 'WXT520')
        self._num_tries = int(stn_dict.get('num_tries', 5))
        self._retry_wait = int(stn_dict.get('retry_wait', 10))
        self._poll_interval = int(stn_dict.get('poll_interval', 0))
        protocol = stn_dict.get('protocol', 'serial').lower()
        if protocol not in ['sdi12', 'nmea0183', 'serial']:
            raise ValueError("unknown protocol '%s'" % protocol)
        baud = WXT5x0Driver.BAUD.get(protocol, 19200)
        baud = int(stn_dict.get('baud', baud))
        port = stn_dict.get('port', WXT5x0Driver.DEFAULT_PORT)
        if protocol == 'nmea0183':
            self._station = StationNMEA0183(port, baud)
        elif protocol == 'sdi12':
            self._station = StationSDI12(port, baud)
        else:
            self._station = StationSerial(port, baud)
        self._station.open()
                
    def closePort(self):
        self._station.shutdown()

    @property
    def hardware_name(self):
        return self._model
                    
    def genLoopPackets(self):
        while True:
            raw = self._station.get_data()
            logdbg("raw: %s" % raw)
            data = Station.parse(raw)
            logdbg("parsed: %s" % data)
            packet = self._data_to_packet(data)
            yield packet
            if self._poll_interval:
                time.sleep(self._poll_interval)

    def _data_to_packet(self, data):
        return dict()


class Station(object):
    def __init__(self, port, baud):
        self.port = port
        self.baudrate = baud
        self.timeout = 3 # seconds

    def open(self):
        pass

    def shutdown(self):
        pass

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()


class StationSerial(Station):
    def __init__(self, port, baud=19200):
        # baud should be 19200 for RS232, RS485, and RS422
        super(StationSerial, self).__init__(port, baud)
        self.serial_port = None

    def open(self):
        import serial
        logdbg("open serial port %s" % self.port)
        self.serial_port = serial.Serial(
            self.port, self.baudrate, timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
            logdbg("close serial port %s" % self.port)
            self.serial_port.close()
            self.serial_port = None

    def get_data(self):
        return ''


class StationNMEA0183(Station):
    def __init__(self, port, baud=4800):
        super(StationNMEA0183, self).__init__(port, baud)
        self.serial_port = None

    def open(self):
        import serial
        logdbg("open serial port %s" % self.port)
        self.serial_port = serial.Serial(
            self.port, self.baudrate, timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
            logdbg("close serial port %s" % self.port)
            self.serial_port.close()
            self.serial_port = None

    def get_data(self):
        return ''


class StationSDI12(Station):
    def __init__(self, port, baud=19200):
        super(StationSDI12, self).__init__(port, baud)
        self.serial_port = None

    def open(self):
        import serial
        logdbg("open serial port %s" % self.port)
        self.serial_port = serial.Serial(
            self.port, self.baudrate, timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
            logdbg("close serial port %s" % self.port)
            self.serial_port.close()
            self.serial_port = None

    def get_data(self):
        return ''


# define a main entry point for basic testing of the station without weewx
# engine and service overhead.  invoke this as follows from the weewx root dir:
#
# PYTHONPATH=bin python bin/weewx/drivers/wxt5x0.py

if __name__ == '__main__':
    import optparse
    usage = """%prog [options] [--debug] [--help]"""
    syslog.openlog('wxt5x0', syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_INFO))
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('--version', action='store_true',
                      help='display driver version')
    parser.add_option('--debug', action='store_true',
                      help='display diagnostic information while running')
    parser.add_option('--protocol', metavar='PROTOCOL',
                      help='serial, nmea0183, or sdi12',
                      default='serial')
    parser.add_option('--port', metavar='PORT',
                      help='serial port to which the station is connected',
                      default=WXT5x0Driver.DEFAULT_PORT)
    parser.add_option('--poll-interval', metavar='POLL', type=int,
                      help='poll interval, in seconds',
                      default=3)
    (options, args) = parser.parse_args()

    if options.version:
        print "wxt5x0 driver version %s" % DRIVER_VERSION
        exit(1)

    if options.debug:
        syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))

    if options.protocol == 'serial':
        s = StationSerial(options.port)
    elif options.protocol == 'nmea0183':
        s = StationNMEA0183(options.port)
    elif options.protocol == 'sdi12':
        s = StationSDI12(options.port)

    while True:
        print int(time.time()), s.get_data()
        time.sleep(options.poll_interval)

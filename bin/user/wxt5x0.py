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

If polling interval is 0, the driver will put the device into automatic mode.
Otherwise, the driver will poll the device at the specified polling interval.
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


class Station(object):
    def __init__(self, address, port, baud):
        self.terminator = '!'
        self.address = address
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

    @staticmethod
    def parse(data):
        # 0R0,Dn=000#,Dm=106#,Dx=182#,Sn=1.1#,Sm=4.0#,Sx=6.6#,Ta=16.0C,Ua=50.0P,Pa=1018.1H,Rc=0.00M,Rd=0s,Ri=0.0M,Hc=0.0M,Hd=0s,Hi=0.0M,Rp=0.0M,Hp=0.0M,Th=15.6C,Vh=0.0N,Vs=15.2V,Vr=3.498V,Id=Ant
        parts = data.strip().split(',')
        parsed = dict()
        for part in parts:
            if '=' in part:
                name, value = part.split('=')
                parsed[name] = value
        return parsed
        

class StationSerial(Station):
    DEFAULT_BAUD = 19200

    def __init__(self, address, port, baud=DEFAULT_BAUD):
        # baud should be 19200 for RS232, RS485, and RS422
        super(StationSerial, self).__init__(address, port, baud)
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

    def set_automatic_mode(self):
        pass
            
    def get_wind(self):
        return ''

    def get_pth(self):
        return ''

    def get_precip(self):
        return ''

    def get_supervisor(self):
        return ''

    def get_composite(self):
        return self.get_data('R0')
        
    def get_data(self, cmd):
        cmd = "%d%s%s" % (self.address, cmd, self.terminator)
        self.serial_port.write(cmd)
        return self.serial_port.readline()


class StationNMEA0183(Station):
    DEFAULT_BAUD = 4800

    def __init__(self, address, port, baud=DEFAULT_BAUD):
        super(StationNMEA0183, self).__init__(address, port, baud)
        self.serial_port = None
        raise NotImplementedError("NMEA support not implemented")


class StationSDI12(Station):
    DEFAULT_BAUD = 1200

    def __init__(self, address, port, baud=DEFAULT_BAUD):
        super(StationSDI12, self).__init__(address, port, baud)
        self.serial_port = None
        raise NotImplementedError("SDI12 support not implemented")


class WXT5x0ConfigurationEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[WXT5x0]
    # This section is for Vaisala WXT5x0 stations

    # The station model such as WXT510 or WXT520
    model = WXT520

    # The communication protocol to use, one of serial, nmea0183, or sdi12
    protocol = serial

    # The serial port to which the station is connected
    port = /dev/ttyUSB0

    # The device address
    address = 0

    # The driver to use
    driver = weewx.drivers.wxt5x0
"""

    def prompt_for_settings(self):
        print "Specify the protocol"
        protocol = self._prompt('protocol', 'serial', ['serial', 'nmea0183', 'sdi12'])
        print "Specify the serial port on which the station is connected, for"
        print "example /dev/ttyUSB0 or /dev/ttyS0."
        port = self._prompt('port', '/dev/ttyUSB0')
        print "Specify the device address"
        address = self._prompt('address', 0)
        return {'protocol': protocol, 'port': port, 'address': address}


class WXT5x0Driver(weewx.drivers.AbstractDevice):
    STATION = {
        'sdi12': StationSDI12,
        'nmea0183': StationNMEA0183,
        'serial': StationSerial,
    }
    BAUD = {
        'sdi12': StationSDI12.DEFAULT_BAUD,
        'nmea0183': StationNMEA0183.DEFAULT_BAUD,
    }
    DEFAULT_PORT = '/dev/ttyUSB0'

    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        self._model = stn_dict.get('model', 'WXT520')
        self._num_tries = int(stn_dict.get('num_tries', 5))
        self._retry_wait = int(stn_dict.get('retry_wait', 10))
        self._poll_interval = int(stn_dict.get('poll_interval', 0))
        self._address = int(stn_dict.get('address', 0))
        protocol = stn_dict.get('protocol', 'serial').lower()
        if protocol not in ['sdi12', 'nmea0183', 'serial']:
            raise ValueError("unknown protocol '%s'" % protocol)
        baud = WXT5x0Driver.BAUD.get(protocol, 19200)
        baud = int(stn_dict.get('baud', baud))
        port = stn_dict.get('port', WXT5x0Driver.DEFAULT_PORT)
        self._station = WXT5x0Driver.STATION.get(protocol)(address, port, baud)
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
                      help='serial, nmea0183, or sdi12', default='serial')
    parser.add_option('--port', metavar='PORT',
                      help='serial port to which the station is connected',
                      default=WXT5x0Driver.DEFAULT_PORT)
    parser.add_option('--baud', type=int,
                      help='baud rate', default=19200)
    parser.add_option('--address', type=int,
                      help='device address', default=0)
    parser.add_option('--poll-interval', metavar='POLL', type=int,
                      help='poll interval, in seconds', default=3)
    (options, args) = parser.parse_args()

    if options.version:
        print "wxt5x0 driver version %s" % DRIVER_VERSION
        exit(1)

    if options.debug:
        syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))

    if options.protocol == 'serial':
        s = StationSerial(options.address, options.port, options.baud)
    elif options.protocol == 'nmea0183':
        s = StationNMEA0183(options.address, options.port, options.baud)
    elif options.protocol == 'sdi12':
        s = StationSDI12(options.address, options.port, options.baud)
    else:
        print "unknown protocol '%s'" % options.protocol
        exit(1)

    s.open()
    while True:
        data = s.get_composite().strip()
        print int(time.time()), data
        parsed = Station.parse(data)
        print parsed
        time.sleep(options.poll_interval)

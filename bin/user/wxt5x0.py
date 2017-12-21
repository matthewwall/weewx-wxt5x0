#!/usr/bin/env python
# Copyright 2017 Matthew Wall, all rights reserved
"""
Collect data from Vaisala WXT510 or WXT520 station.

Thanks to Antonis Katsonis for providing a Vaisala WXT520 for development.

http://www.vaisala.com/Vaisala%20Documents/User%20Guides%20and%20Quick%20Ref%20Guides/M210906EN-C.pdf

The WXT520 is available with the following serial communications:
 - RS232: ASCII automatic and polled; NMEA0183 v3; SDI12 v1.3
 - RS485: ASCII automatic and polled; NMEA0183 v3; SDI12 v1.3
 - RS422: ASCII automatic and polled; NMEA0183 v3; SDI12 v1.3
 - SDI12: v1.3 and v1.3 continuous

This driver supports only ASCII communications protocol.

The precipitation sensor measures both rain and hail.

The precipitation sensor has three modes: precipitation on/off, tipping bucket,
and time based.  In precipitation on/off, the transmitter ends a precipitation
message 10 seconds after the first recognition of precipitation.  Rain duration
increases in 10 second steps.  Precipitation has ended when Ri=0.  This mode is
used for indication of the start and the end of precipitation.

In tipping bucket, the transmitter sends a precipitation message at each unit
increment (0.1mm/0.01 in).  This simulates conventional tipping bucket method.

In time based mode, the transmitter sends a precipitation message in the
intervals defined in the [I] field.  However, in polled protocols the autosend
mode tipping bucket should not be used as in it the resolution of the output
is decreased (quantized to tipping bucket tips).

The precipitation sensor can also operate in polled mode - it sends a precip
message when requested.

The rain counter reset can be manual, automatic, immediate, or limited.

The supervisor message controls error messaging and heater.
"""

# FIXME: test with and without error messages
# FIXME: test with and without crc

from __future__ import with_statement
import syslog
import time

import weewx.drivers

DRIVER_NAME = 'WXT5x0'
DRIVER_VERSION = '0.1'

MPS_PER_KPH = 0.277778
MPS_PER_MPH = 0.44704
MPS_PER_KNOT = 0.514444
MBAR_PER_PASCAL = 0.01
MBAR_PER_BAR = 1000.0
MBAR_PER_MMHG = 1.33322387415
MBAR_PER_INHG = 33.8639
MM_PER_INCH = 25.4
CM2_PER_IN2 = 6.4516


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
    def __init__(self, address, port, baud, use_crc=False):
        self.crc_prefix = None
        self.terminator = ''
        self.address = address
        self.port = port
        self.baudrate = baud
        self.timeout = 3 # seconds
        self.device = None

    def open(self):
        pass

    def shutdown(self):
        pass

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def send_cmd(self, cmd):
        cmd = "%d%s%s" % (self.address, cmd, self.terminator)
        self.device.write(cmd)

    def get_data(self, cmd):
#        if self.crc_prefix:
#            cmd = cmd.replace('R', 'r')
#            cmd = "%sxxx" % self.crc_prefix
        self.send_cmd(cmd)
        return self.device.readline()

    def get_address(self):
        self.device.write('?%s' % self.terminator)
        return self.device.readline()

    def set_address(self, addr):
        self.send_cmd('A%d' % addr)

    def get_ack(self):
        return self.get_data('')

    def reset(self):
        self.send_cmd('XZ')

    def precip_counter_reset(self):
        self.send_cmd('XZRU')

    def precip_intensity_reset(self):
        self.send_cmd('XZRI')

    def measurement_reset(self):
        self.send_cmd('XZM')

    def set_automatic_mode(self):
        self.send_cmd('XU,M=R')

    def set_polled_mode(self):
        self.send_cmd('XU,M=P')

    def get_wind(self):
        return self.get_data('R1')

    def get_pth(self):
        return self.get_data('R2')

    def get_precip(self):
        return self.get_data('R3')

    def get_supervisor(self):
        return self.get_data('R5')

    def get_composite(self):
        return self.get_data('R0')

    @staticmethod
    def calc_crc(txt):
        crc = 0
        for x in txt:
            crc |= ord(x)
            for cnt in range(1, 9):
                if crc << 16 == 1:
                    crc >>= 1
                    crc |= 0xa001
                else:
                    crc >>= 1
        a = 0x40 | (crc >> 12)
        b = 0x40 | ((crc >> 6) & 0x3f)
        c = 0x40 | (crc & 0x3f)
        return a + b + c

    OBSERVATIONS = {
        # aR1: wind message
        'Dn': 'wind_dir_min',
        'Dm': 'wind_dir_avg',
        'Dx': 'wind_dir_max',
        'Sn': 'wind_speed_min',
        'Sm': 'wind_speed_avg',
        'Sx': 'wind_speed_max',
        # aR2: pressure, temperature, humidity message
        'Ta': 'temperature',
        'Ua': 'humidity',
        'Pa': 'pressure',
        # aR3: precipitation message
        'Rc': 'rain',
        'Rd': 'rain_duration',
        'Ri': 'rain_intensity',
        'Hc': 'hail',
        'Hd': 'hail_duration',
        'Hi': 'hail_intensity',
        'Rp': 'rain_intensity_peak',
        'Hp': 'hail_intensity_peak',
        # dR5: supervisor message
        'Th': 'heating_temperature',
        'Vh': 'heating_voltage',
        'Vs': 'supply_voltage',
        'Vr': 'reference_voltage',
        'Id': 'information',
        }

    @staticmethod
    def parse(raw):
        # 0R0,Dn=000#,Dm=106#,Dx=182#,Sn=1.1#,Sm=4.0#,Sx=6.6#,Ta=16.0C,Ua=50.0P,Pa=1018.1H,Rc=0.00M,Rd=0s,Ri=0.0M,Hc=0.0M,Hd=0s,Hi=0.0M,Rp=0.0M,Hp=0.0M,Th=15.6C,Vh=0.0N,Vs=15.2V,Vr=3.498V,Id=Ant
        parsed = dict()
        for part in raw.strip().split(','):
            if '=' in part:
                abbr, vstr = part.split('=')
                if abbr == 'Id': # skip the information field
                    continue
                obs = Station.OBSERVATIONS.get(abbr)
                if obs:
                    value = None
                    unit = None
                    try:
                        unit = vstr[-1]
                        if unit != '#': # '#' indicates invalid data
                            value = float(vstr[:-1])
                            value = Station.convert(obs, value, unit)
                    except ValueError, e:
                        logerr("parse failed for %s (%s):%s" % (abbr, vstr, e))
                    parsed[obs] = value
                else:
                    logdbg("unknown sensor %s: %s" % (abbr, vstr))
        return parsed

    @staticmethod
    def convert(obs, value, unit):
        # convert from the indicated units to the weewx METRICWX unit system
        if 'temperature' in obs:
            # [T] temperature C=celsius F=fahrenheit
            if unit == 'C':
                pass # already C
            elif unit == 'F':
                value = (value - 32.0) * 5.0 / 9.0
            else:
                loginf("unknown unit '%s' for %s" % (unit, obs))
        elif 'wind_speed' in obs:
            # [U] speed M=m/s K=km/h S=mph N=knots
            if unit == 'M':
                pass # already m/s
            elif unit == 'K':
                value *= MPS_PER_KPH
            elif unit == 'S':
                value *= MPS_PER_MPH
            elif unit == 'N':
                value *= MPS_PER_KNOT
            else:
                loginf("unknown unit '%s' for %s" % (unit, obs))
        elif 'pressure' in obs:
            # [P] pressure H=hPa P=pascal B=bar M=mmHg I=inHg
            if unit == 'H':
                pass # already hPa/mbar
            elif unit == 'P':
                value *= MBAR_PER_PASCAL
            elif unit == 'B':
                value *= MBAR_PER_BAR
            elif unit == 'M':
                value *= MBAR_PER_MMHG
            elif unit == 'I':
                value *= MBAR_PER_INHG
            else:
                loginf("unknown unit '%s' for %s" % (unit, obs))
        elif 'rain' in obs:
            # rain: accumulation duration intensity intensity_peak
            # [U] precip M=(mm s mm/h) I=(in s in/h)
            if unit == 'M':
                pass # already mm
            elif unit == 'I':
                if 'duration' not in obs:
                    value *= MM_PER_INCH
            elif unit == 's':
                pass # already seconds
            else:
                loginf("unknown unit '%s' for %s" % (unit, obs))
        elif 'hail' in obs:
            # hail: accumulation duration intensity intensity_peak
            # [S] hail M=(hits/cm^2 s hits/cm^2h) I=(hits/in^2 s hits/in^2h)
            #          H=hits
            if unit == 'M':
                pass # already cm^2
            elif unit == 'I':
                if 'duration' not in obs:
                    value *= CM2_PER_IN2
            elif unit == 's':
                pass # already seconds
            else:
                loginf("unknown unit '%s' for %s" % (unit, obs))
        return value


class StationSerial(Station):
    # ASCII over RS232, RS485, and RS422 defaults to 19200, 8, N, 1
    DEFAULT_BAUD = 19200

    def __init__(self, address, port, baud=DEFAULT_BAUD):
        super(StationSerial, self).__init__(address, port, baud)
        self.terminator = '\r\n'
        self.device = None

    def open(self):
        import serial
        logdbg("open serial port %s" % self.port)
        self.device = serial.Serial(
            self.port, self.baudrate, timeout=self.timeout)

    def close(self):
        if self.device is not None:
            logdbg("close serial port %s" % self.port)
            self.device.close()
            self.device = None


class StationNMEA(Station):
    # RS422 NMEA defaults to 4800, 8, N, 1
    DEFAULT_BAUD = 4800

    def __init__(self, address, port, baud=DEFAULT_BAUD):
        super(StationNMEA, self).__init__(address, port, baud)
        self.terminator = '\r\n'
        raise NotImplementedError("NMEA support not implemented")


class StationSDI12(Station):
    # SDI12 defaults to 1200, 7, E, 1
    DEFAULT_BAUD = 1200

    def __init__(self, address, port, baud=DEFAULT_BAUD):
        super(StationSDI12, self).__init__(address, port, baud)
        self.terminator = '!'
        raise NotImplementedError("SDI12 support not implemented")


class WXT5x0ConfigurationEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[WXT5x0]
    # This section is for Vaisala WXT5x0 stations

    # The station model such as WXT510 or WXT520
    model = WXT520

    # The communication protocol to use, one of serial, nmea, or sdi12
    protocol = serial

    # The port to which the station is connected
    port = /dev/ttyUSB0

    # The device address
    address = 0

    # The driver to use
    driver = user.wxt5x0
"""

    def prompt_for_settings(self):
        print "Specify the protocol"
        protocol = self._prompt('protocol', 'serial', ['serial', 'nmea', 'sdi12'])
        print "Specify the serial port on which the station is connected, for"
        print "example /dev/ttyUSB0 or /dev/ttyS0."
        port = self._prompt('port', '/dev/ttyUSB0')
        print "Specify the device address"
        address = self._prompt('address', 0)
        return {'protocol': protocol, 'port': port, 'address': address}


class WXT5x0Driver(weewx.drivers.AbstractDevice):
    STATION = {
        'sdi12': StationSDI12,
        'nmea': StationNMEA,
        'serial': StationSerial,
    }
    DEFAULT_PORT = '/dev/ttyUSB0'

    # map sensor names to schema names
    DEFAULT_MAP = {
        'windDir': 'wind_dir_avg',
        'windSpeed': 'wind_speed_avg',
        'windGustDir': 'wind_dir_max',
        'windGust': 'wind_speed_max',
        'outTemp': 'temperature',
        'outHumidity': 'humidity',
        'pressure': 'pressure',
        'rain_total': 'rain',
        'rainRate': 'rain_intensity',
        'hail_total': 'hail',
        'hailRate': 'hail_intensity',
        'heatingTemp': 'heating_temperature',
        'heatingVoltage': 'heating_voltage',
        'supplyVoltage': 'supply_voltage',
        'referenceVoltage': 'reference_voltage',
        }

    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        self._model = stn_dict.get('model', 'WXT520')
        self._max_tries = int(stn_dict.get('max_tries', 5))
        self._retry_wait = int(stn_dict.get('retry_wait', 10))
        self._poll_interval = int(stn_dict.get('poll_interval', 1))
        self._sensor_map = dict(WXT5x0Driver.DEFAULT_MAP)
        address = int(stn_dict.get('address', 0))
        protocol = stn_dict.get('protocol', 'serial').lower()
        if protocol not in WXT5x0Driver.STATION:
            raise ValueError("unknown protocol '%s'" % protocol)
        baud = WXT5x0Driver.STATION[protocol].DEFAULT_BAUD
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
            for cnt in range(self._max_tries):
                try:
                    raw = self._station.get_composite()
                    logdbg("raw: %s" % raw)
                    data = Station.parse(raw)
                    logdbg("parsed: %s" % data)
                    packet = self._data_to_packet(data)
                    logdbg("mapped: %s" % packet)
                    if packet:
                        yield packet
                    break
                except IOError, e:
                    logerr("Failed attempt %d of %d to read data: %s" %
                           (cnt + 1, self.max_tries, e))
                    logdbg("Waiting %d seconds" % self._retry_wait)
                    time.sleep(self._retry_wait)
            else:
                raise weewx.RetriesExceeded("Read failed after %d tries" %
                                            self._max_tries)
            if self._poll_interval:
                time.sleep(self._poll_interval)

    def _data_to_packet(self, data):
        # if there is a mapping to a schema name, use it.  otherwise use the
        # sensor naming native to the hardware.
        packet = dict()
        for name in data:
            obs = name
            for field in self._sensor_map:
                if self._sensor_map[field] == name:
                    obs = field
                    break
            packet[obs] = data[name]
        if packet:
            packet['dateTime'] = int(time.time() + 0.5)
            packet['usUnits'] = weewx.METRICWX
        return packet


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
    parser.add_option('--protocol',
                      help='serial, nmea, or sdi12', default='serial')
    parser.add_option('--port',
                      help='serial port to which the station is connected',
                      default=WXT5x0Driver.DEFAULT_PORT)
    parser.add_option('--baud', type=int,
                      help='baud rate', default=19200)
    parser.add_option('--address', type=int,
                      help='device address', default=0)
    parser.add_option('--poll-interval', metavar='POLL', type=int,
                      help='poll interval, in seconds', default=3)
    parser.add_option('--get-wind',
                      help='get a single wind message')
    parser.add_option('--get-pth',
                      help='get a pressure/temperature/humidity message')
    parser.add_option('--get-precip',
                      help='get a single precipitation message')
    parser.add_option('--get-supervisor',
                      help='get a single supervisor message')
    parser.add_option('--get-composite',
                      help='get a single composite message')
    parser.add_option('--test-crc', metavar='STRING',
                      help='verify the CRC calculation')
    (options, args) = parser.parse_args()

    if options.version:
        print "%s driver version %s" % (DRIVER_NAME, DRIVER_VERSION)
        exit(1)

    if options.debug:
        syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))

    if options.test_crc:
        print "string: '%s'" % options.test_crc
        print "crc: '%s'" % Station.calc_crc(options.test_crc)
        exit(0)

    if options.protocol == 'serial':
        cls = StationSerial
    elif options.protocol == 'nmea':
        cls = StationNMEA
    elif options.protocol == 'sdi12':
        cls = StationSDI12
    else:
        print "unknown protocol '%s'" % options.protocol
        exit(1)

    with cls(options.address, options.port, options.baud) as s:
        if options.get_wind:
            print s.get_wind().strip()
        elif options.get_pth:
            print s.get_pth().strip()
        elif options.get_precip:
            print s.get_precipitation().strip()
        elif options.get_supervisor:
            print s.get_supervisor().strip()
        elif options.get_composite:
            print s.get_composite().strip()
        else:
            while True:
                data = s.get_composite().strip()
                print int(time.time()), data
                parsed = Station.parse(data)
                print parsed
                time.sleep(options.poll_interval)

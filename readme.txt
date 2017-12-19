weewx-wxt5x0

This is a driver for weewx that collects data from WXT5x0 hardware.

Installation

0) install weewx (see the weewx user guide)

1) download the driver

wget -O weewx-wxt5x0.zip https://github.com/matthewwall/weewx-wxt5x0/archive/master.zip

2) install the driver

wee_extension --install weewx-wxt5x0.zip

3) configure the driver

wee_config --reconfigure

4) start weewx

sudo /etc/init.d/weewx start

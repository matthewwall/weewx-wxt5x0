# installer for wxt5x0 driver
# Copyright 2017 Matthew Wall

from setup import ExtensionInstaller

def loader():
    return WXT5x0Installer()

class WXT5x0Installer(ExtensionInstaller):
    def __init__(self):
        super(WXT5x0Installer, self).__init__(
            version="0.5",
            name='wxt5x0',
            description='Collect data from WXT5x0 hardware',
            author="Matthew Wall",
            author_email="mwall@users.sourceforge.net",
            files=[('bin/user', ['bin/user/wxt5x0.py'])]
        )

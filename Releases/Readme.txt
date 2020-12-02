Microchip LAN78xx UEFI PXE (UNDI) driver Release - v03
===================================================

This file provides information about the UEFI drivers (32 bit & 64 bit) for 
LAN78xx devices.

The .efi driver can be merged with UEFI firmware or can be loaded from UEFI shell.

Contents
--------

A. Device Supported
B. Driver/Utility files in this package
C. Lan7800Utility
D. Release History
E. Known Issues

A. Devices Supported
--------------------

The following SMSC devices are supported by this distribution:
- LAN7800
- LAN7801
- LAN780A
- LAN7850

B. Driver and utility files
---------------------------

./IA32/Lan7800Driver.efi	- UEFI driver for x86 platform
./IA32/Lan7800Utility.efi	- UEFI Utility for x86 platform
./X64/Lan7800Driver.efi		- UEFI driver for x64 platform
./X64/Lan7800Utility.efi	- UEFI Utilityr for x64 platform

C. The Lan7800Utility
------------------------
The Lan7800Utility.efi provivides simple utility to access the LAN7800 UEFI driver
supported features like read/wrire EEPROM, Check the link properties and register 
read/writes.

To use the utlity load the utility from the UEFI shell.


D. Release History
------------------

** v0.3  ** 11/25/2020

- The performance improvements.

** v0.2  ** 02/03/2020

- Updated Copyright notice for souce code release.

** v0.01 ** 09/22/2015

- Initial Release


G. Known Issues
---------------

** v0.3  ** 11/25/2020

- Utility's EEPROM/OTP read is not working correctly. Need to debug/fix.

End of file
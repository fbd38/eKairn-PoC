# eKairn-PoC
Proof of Concept Program to be used as eBeacon for Orienteering

To be use with VIKAZIMUT program, running on Android and IOS, developped by the CAEN french university.

Its uses a XIAO BLE rf5280 module with the Adafruit BLE library under the Arduino 2.0 IDE.
It should work with XIAO BLE SENSE module and/or Ardafruit boards, but it have not been tested with these HW.

You can change the setup using standard Nordik tool (UART) by entring commands line.
see eKairn_command_line file for the list of valid commands.

[TODO] found a solution for permanent storage of the setup.
[TODO] implement "Z" command.
[TODO] implement "X" command.
[TODO] implement "K" command.
[TODO] add DFU OTA.
[TODO] add BLE service and characteristics to program the eKairn parameters instead of the Nordik UART propreritary solution.

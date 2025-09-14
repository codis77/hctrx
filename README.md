
a set of application to utilize the HC12 transmitter parameters via serial line
  --- f.m.  02.09.2025

 - hccfg : configure the transmitter via AT commands
 - hctrx : transmit or receive a message

The HC-12 is a 433MHz RF transmitter board, utilizing an onboard microcontroller.
The MCU provides the serial interface to the host, keep important interface settings
in nonvolatile memory, and manages initalisation and operation of the RF transmitter.

The transmitter parameters stored in non-volatile memory are baudrate, power, channel number,
and FU mode. This this settings are persistent, and will remain valid until changed.

However, these parameters are only valid in operational mode.
The configuration mode (SET pin of the HC-12 is LOW) always uses 9600 bps@ 8N1.



*** this is work in progress ***


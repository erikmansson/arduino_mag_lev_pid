arduino_mag_lev_pid
===============
Arduino Magnetic Levitator

PID controller

Written for the Teensy 3

By Erik Månsson

A demo can be seen here: http://youtu.be/AeTEj5-hTBc

####Hardware used in demo:
- Teensy 3.1
- Honeywell SS59ET Hall sensor
- 5 Ohm, 69m, ⌀0,4mm coil
- A decent MOSFET driver (VGS <= Arduino operating voltage)
- 10-15V PSU (add capacitors to cheaper PSUs)
- 200-600uF capacitor over the coil
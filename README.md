# Fuel pressure controller
Raspberry Pi Pico based pwm controller for fuel injected engine.
Project developed for a motorcycle.
Harware used:
* CBR600RR fuel pump
* Fuel pressure sensor from Ford focus
* Rotary dial for pressure adjustment

## Rotary dial calibration

The firmware now calibrates the rotary dial on start-up. Rotate the dial across
its full range during the first two seconds after power on. The dial position is
then used to select between three different variants.

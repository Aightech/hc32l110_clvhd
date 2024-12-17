# HC32L110 firmware for CleverHand project

This repository contains the firmware for the CleverHand project. The firmware is based on the HC32L110 microcontroller from HDSC.
This firmware is built upon the template provided by the [IOsetting HC32L110](https://github.com/IOsetting/hc32l110-template)

## Usage

connect daplink SWD SCK GND 3.3V to module

```bash
make
make flash
```

# File Structure

Here a brief overview of the firmware behavior:

- setups:
  - GPIO (NEOPixel, address pins, clock pins, etc.)
- Addressing:

- Get the address of the module
    -

# How to use this repository

See the [HOWTO.md](HOWTO.md) file for instructions on how to use this repository.

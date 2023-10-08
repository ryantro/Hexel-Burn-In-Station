# Hexel-Burn-In-Station

## Function
---
Powers up to 28 individual laser devices and measures their spectrum, power, and voltage across time at a set current.

## Parts
---
The station consists of a PC running Xubuntu which communicates with:
- 2-axis stage hold a powermeter and integrating sphere
  - Unimotion Stages
  - Applied motion servo and stepper motors via Ethernet
- Thorlabs 350C Powermeter for measuring optical power via USB
- 819D-SF-6 Integrating sphere hooked up to an OSA via USB
- Custom Ocean Insign OSA
- Local network for saving data
- 10 x Raspberry Pi's via Ethernet

The 10 x Raspberry Pi's handle:
- Reading the laser device serial numbers via a multiplexed I2C bus
- Communicating with the 30 laser driver boards
- Setting and reading current
- Reading voltage

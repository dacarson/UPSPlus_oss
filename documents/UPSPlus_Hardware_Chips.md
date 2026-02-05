## UPSPlus HAT Chip Inventory (EP-0136)

This document lists the known ICs on the UPSPlus HAT and their roles. It is a
technical reference intended to be kept current as hardware variants are
confirmed.

## I2C Bus Devices

- (**STM32F030F4P6**)[https://www.st.com/resource/en/datasheet/stm32f030f4.pdf] (MCU, SW I2C slave: 0x17 runtime; 0x18 OTA)
  - Main microcontroller running the UPSPlus firmware.
  - Shares the I2C bus with the current monitors and RTC.
- **INA219** x2 (current monitors, I2C slave: 0x40 & 0x45)
  - One monitors the output/RPi rail, one monitors the battery path.
  - Each INA219 uses an onboard shunt resistor marked **R010**.
- **DS1307** (RTC, I2C slave 0x68)
  - Real-time clock device on the same I2C bus.

## Power / Charging

- **IP5328** (battery charger / power management)
  - Handles battery charging and power path control.
  - Has i2c interface but not connected to i2c bus
  - Charging can be enabled and disabled via MCU
  - Low Battery Cutoff is roughly 2.4–2.6 V
  - Recovery around 3.0 V

## Notes

- If additional regulators, protection ICs, or board revisions are identified,
  add them here with part numbers and roles.

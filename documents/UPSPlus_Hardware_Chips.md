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

- **IP5328** (Injoinic; battery charger / power management)
  - Fully-integrated power bank SoC with **USB-C PD 3.0** (Power Delivery) support.
  - Handles battery charging and power path control.
  - Accepts 5 V and higher PD input voltages (e.g. 9 V when the charger negotiates PD); USB-C register readings around 8–9 V are normal with a PD charger.
  - I2C interface present on the IC but not connected to the HAT I2C bus.
  - Charging can be enabled and disabled via MCU (IP_EN pin).
  - Low Battery Cutoff is roughly 2.4–2.6 V; recovery around 3.0 V.

## Notes

- If additional regulators, protection ICs, or board revisions are identified,
  add them here with part numbers and roles.
- DEBUG UPSPlus HAT pins, Pin 1 on the LHS
  Pin 1 - VDD MCU Pin 16
  Pin 2 - SWCLK MCU Pin 20
  Pin 3 - SWDIO MCU Pin 19
  Pin 4 - GND MCU Pin 15
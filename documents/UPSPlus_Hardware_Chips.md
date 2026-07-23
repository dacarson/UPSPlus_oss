## UPSPlus HAT Chip Inventory (EP-0136)

This document lists the known ICs on the UPSPlus HAT and their roles. It is a
technical reference intended to be kept current as hardware variants are
confirmed.

## I2C Bus Devices

- (**STM32F030F4P6**)[https://www.st.com/resource/en/datasheet/stm32f030f4.pdf] (MCU, SW I2C slave: 0x17 runtime; 0x18 OTA) — **board ref U9**
  - Main microcontroller running the UPSPlus firmware.
  - Shares the I2C bus with the current monitors and RTC.
- **INA219** x2 (current monitors, I2C slave: 0x40 & 0x45) — **board ref U5, U6**
  - One monitors the output/RPi rail, one monitors the battery path.
  - Each INA219 uses an onboard shunt resistor marked **R010**.
  - SOIC-8, TI logo visible on package. Top markings read approx.
    "I219A" / [TI logo]"1BM" / "ASCX G4" (G4 underlined) — consistent with
    an INA219AIDR device+date+lot code, but not yet cross-checked against
    TI's official part marking lookup (https://www.ti.com/packaging/docs/partlookup.tsp).
- **DS1307** (RTC, I2C slave 0x68) — **board ref unconfirmed**
  - Real-time clock device on the same I2C bus.
  - SOIC-8, located adjacent to crystal **Y1** near U5/U6. Top marking
    reads "DS1307N" with date/lot code "2214AG" / "+970AB".

## Power / Charging

- **IP5328** (Injoinic; battery charger / power management) — **board ref U2**
  - Fully-integrated power bank SoC with **USB-C PD 3.0** (Power Delivery) support.
  - Handles battery charging and power path control.
  - Accepts 5 V and higher PD input voltages (e.g. 9 V when the charger negotiates PD); USB-C register readings around 8–9 V are normal with a PD charger.
  - I2C interface present on the IC but not connected to the HAT I2C bus.
  - Charging can be enabled and disabled via MCU (IP_EN pin).
  - Low Battery Cutoff is roughly 2.4–2.6 V; recovery around 3.0 V.
  - **KEY** pin (IP5328 pin 26) is the button-detect input; also multiplexed
    with the WLED flashlight driver.
- **TPS61088** (TI; 10-A fully-integrated synchronous boost converter) — **board ref U1**
  - 20-pin VQFN (RHL), 2.7–12 V input, 4.5–12.6 V adjustable output, 10 A
    switch current. [Product page](https://www.ti.com/product/TPS61088) /
    [datasheet](https://www.ti.com/lit/gpn/tps61088).
  - Located mid-board on the "25A MAX" trace side, next to C9/R2, near
    inductor **L2**. Top marking reads "S61088A" (TI symbolization for
    TPS61088) / [TI logo]+date code / lot-trace code.
  - Likely generates a dedicated regulated rail (probably 5 V) for the
    RPi header, separate from the IP5328's own boost/charge path — output
    plausibly gated onward by **Q7** (NCE20P70G) toward the Pi header.

## MOSFETs / Switches

- **NCE20P70G** (Wuxi NCE Power; P-channel power MOSFET, −20 V / −70 A) — **board ref Q7**
  - Located near R28/C32, close to the gold RPi header — likely the
    high-side load switch gating power to the Pi header, driven by the
    MCU's `MT_EN` (RPi Power Enable) line.
- **FKBB3105** (FETek Technology Corp; P-channel 30 V fast-switching MOSFET) — **board ref Q1, Q2**
  - Package PRPAK3x3 (small leadless power package) — matches the top
    marking read as "B3105" (abbreviated from FKBB3105), second line
    "P33XL5" being a date/lot code.
  - BVDSS −30 V, RDS(ON) ≈14 mΩ typ. (VGS=−10V, ID=−30A), continuous
    ID up to −42 A (TC=25°C). [Datasheet](https://www.jasencan.com/Upload/PicFiles/2020.9.1_18.2.54_3247.pdf).
  - Positioned flanking the IP5328's VIN/VBUS input pins; matches the
    external input-path PMOS switches (VING/VBUSG) the IP5328
    application circuit expects.

## Notes

- If additional regulators, protection ICs, or board revisions are identified,
  add them here with part numbers and roles.
- Reference designators above (U*, Q*) were read directly off the board via
  macro photography. Q1/Q2 (FKBB3105) confirmed against manufacturer
  datasheet; U5/U6's INA219 marking is still a best-effort read and should
  be re-verified against TI's part marking lookup before relying on it for
  a respin.
- DS1307's own reference designator wasn't visible in the photos taken so far —
  worth confirming (likely U3, U4, or U7 depending on board silkscreen order).
- DEBUG UPSPlus HAT pins, Pin 1 on the LHS
  Pin 1 - VDD MCU Pin 16
  Pin 2 - SWCLK MCU Pin 20
  Pin 3 - SWDIO MCU Pin 19
  Pin 4 - GND MCU Pin 15

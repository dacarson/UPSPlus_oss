# UPSPlus Open Source Firmware
[![License: MIT](https://img.shields.io/github/license/dacarson/UPSPlus_oss)](LICENSE)
[![Version](https://img.shields.io/github/v/release/dacarson/UPSPlus_oss)](https://github.com/dacarson/UPSPlus_oss/releases)
[![Downloads](https://img.shields.io/github/downloads/dacarson/UPSPlus_oss/total)](https://github.com/dacarson/UPSPlus_oss/releases)

Open-source firmware for the UPSPlus uninterruptible power supply (UPS) module (EP-0136) designed for Raspberry Pi. [The device has been declared as EOL](https://github.com/geeekpi/upsplus/blob/main/firmware/README.md).

## Hardware Requirements

- [UPSPlus HAT EP-0136](https://wiki.52pi.com/index.php?title=EP-0136)
- Raspberry Pi (compatible models)

## Firmware Features

- Battery monitoring via I2C register map.
- Protection shutdown at configured protection voltage.
- Auto power-on with configurable low-battery threshold and load-on delay.
- Boot brownout backoff learning — automatically increases the power-on delay if the Raspberry Pi repeatedly fails to boot (see [Boot Brownout Backoff](#boot-brownout-backoff)).
- Automatic battery full and empty voltage learning — no manual calibration required (see [Automatic Battery Calibration](#automatic-battery-calibration)).
- Output current reporting via I2C (no separate bus reads required).
- Factory testing pages for diagnostics.
- OTA firmware update support (see [Firmware Update](#firmware-update-ota---legacy-bootloader)).
- Periodic IP5328 reset while charging, to avoid a rare charger-stall condition (see [IP5328 Periodic Reset](#ip5328-periodic-reset)).
- On-demand and periodic battery-level LED indication while running on battery (see [Battery-Level LED Check](#battery-level-led-check)).
- Deep sleep (MCU Stop mode) when the load is off and no charger is connected, to minimize battery drain (see [Deep Sleep When the Load Is Off](#deep-sleep-when-the-load-is-off)).

## Button Behavior

- The Func Key button is debounced at 50 ms.
- If the load is off (or waiting on the load-on delay), a short press powers on the load when conditions are safe (sufficient battery charge), skipping the remaining delay if one is in progress.
- If the load is on and running from the charger, a short press has no action (the charger's own LED display is already showing).
- If the load is on and running from the battery (charger unplugged), a short press lights the battery-level LEDs for a while — see [Battery-Level LED Check](#battery-level-led-check).
- Long press (>= 10 s) when the load is on: powers off.
- Long press (>= 10 s) when the load is off: triggers a factory reset.

## Boot Brownout Backoff

If the Raspberry Pi loses power shortly after turn-on (battery voltage drops to or below the protection threshold within 5 minutes of the Pi being powered on), the firmware treats this as a failed boot and automatically increases the load-on delay by 1 minute before the next attempt. This gives the battery more time to stabilize before the Pi draws full power.

Learning stops after a successful 5-minute run without a protection-triggered shutdown. The learned delay is capped at 60 minutes, is remembered across power cycles, and is cleared only by factory reset. If you manually set a load-on delay, any further learning adds on top of that value.

## Automatic Battery Calibration

The firmware learns your battery's actual full and empty voltages from real use, so battery percentage is accurate for your specific batteries.

- **Learn full:** Plug in the charger and let the batteries charge fully with the load on (about 30 minutes or more at a stable high voltage). Full is detected when the battery voltage stays within a narrow band near the top (~4.18 V) for 30 minutes. If current readings are available, the firmware also confirms the charge current has tapered before declaring full. Full clears when the charger is unplugged or the voltage stays below the reset level for ~45 seconds.
- **Learn empty:** Run the unit on battery until the Raspberry Pi turns off. The firmware records the lowest voltage under load just before shutdown as the learned empty value.

**Note:** 18650 batteries can safely discharge to ~2.75 V, but at that level they can no longer supply enough power for the Raspberry Pi's 5 V rail. Empty is typically learned around 3 V.

Learned values are used for battery percentage and protection. You can read or override them via registers `0x0D–0x0E` (full) and `0x0F–0x10` (empty). Write `1` to register `0x2A` to disable self-programming and keep manual values.

## Battery-Level LED Check

The UPSPlus's onboard battery-level LED bar is driven natively by the IP5328 power chip. While a charger is connected, IP5328 lights the bar automatically to show charge progress. While running on battery alone, the bar is normally off — the firmware can ask the chip to light it briefly in two ways:

- **On demand:** a short press of the Func Key button while the load is on and the charger is unplugged.
- **Periodically:** automatically, once every `sample_period_minutes` (register `0x15–0x16`, default and minimum 2 minutes, maximum 1440 minutes/24 hours) — so a glance at the unit shows a recent battery level without needing to press anything.

Either trigger lights the LED bar for roughly 16–32 seconds, after which it turns off on its own (no button sequence needed to clear it). Both triggers are no-ops while a charger is connected, since the LED bar is already showing charge status in that case. The periodic timer only runs while genuinely on battery power with the load on; it pauses (without losing its place) whenever the charger is connected or the load is off, and restarts a fresh interval the next time both conditions are met again.

## IP5328 Periodic Reset

The IP5328 chip has an internal charge safety timer that, on rare occasions with this board's wiring, can fail to reset itself once a charge cycle finishes — leaving the charger connected but silently no longer charging. To guard against this, the firmware resets the IP5328 chip every 20 hours while a charger has been continuously connected. The reset is quick (about 10 seconds) and does not power-cycle the Raspberry Pi; you may briefly see the battery-level LEDs glitch or blink during the reset — this is expected and stops as soon as the reset completes.

While the reset is in progress, register `0x17` bit1 reads `1` (it reads `0` the rest of the time, including during a battery-level LED check). You may also see `battery_current` (register `0x30–0x31`) briefly read as a discharge value during the reset, even though the charger is still connected — the reset can momentarily interrupt the chip's internal charge path, so the Raspberry Pi's power draw is briefly sourced from the battery instead. This reading is real, not a fault or a stale sample, and bit1 lets anything monitoring battery current attribute the blip to the reset rather than an actual loss of charging.

## Deep Sleep When the Load Is Off

When the load is off (Raspberry Pi powered off, or protection-latched) **and** no charger is connected — i.e. running on battery alone with nothing to power — the firmware drops the MCU into Stop mode to minimize battery drain. It wakes briefly about every 5 seconds to check whether a charger has been connected, and immediately on a button press; either wake resumes normal operation automatically.

**While in this state, the I2C port to the MCU does not respond** — its clock is disabled along with the rest of the MCU's peripherals to save power. This doesn't affect normal use, since the Raspberry Pi that would otherwise be the I2C host is powered off in this state anyway; it only matters if something else (e.g. an external diagnostic tool) tries to poll the bus while the unit is in this state — it will see no response until the unit wakes.

The INA219 current sensor and the RTC continue to operate independently of the MCU during this time: INA219 keeps measuring current on its own, and the RTC is what drives the periodic wake.

## Firmware Update (OTA - Legacy Bootloader)

### Entering OTA mode

**Button sequence (recommended)**
1. Remove all power and batteries.
2. Hold the Func Key button and insert batteries.
3. The device starts in OTA mode.

An I2C command can also be used to enter OTA mode. See `documents/UPSPlus_Debugging_Guide.md` for details. Note that the I2C method reboots the device immediately and restarts the Raspberry Pi.

### Flashing the new firmware

1. Verify OTA mode with `i2cdetect -y 1`; the bootloader should be visible at address `0x18`.
2. Copy the compiled binary to the same directory as `tools/OTA_upgrade.py` and rename it to `upsplus_oss.bin` if needed.
3. Run:

```bash
python3 tools/OTA_upgrade.py
```

After a successful update, remove power and batteries to force a reboot; the device does not reset automatically.

## I2C Register Map

I2C address: `0x17` on bus `1` (Raspberry Pi default). Multi-byte registers are little-endian (LSB at lower address). For read/write examples and diagnostic commands, see `documents/UPSPlus_Debugging_Guide.md`.

| Address range | Description | Access | Default |
| --- | --- | --- | --- |
| `0x01–0x02` | MCU voltage (mV) | RO | - |
| `0x03–0x04` | Pogopin voltage (mV) | RO | - |
| `0x05–0x06` | Battery voltage (mV) | RO | - |
| `0x07–0x08` | USB-C voltage (mV). 5 V or higher with PD chargers (e.g. ~9 V is normal). | RO | - |
| `0x09–0x0A` | Micro-USB voltage (mV) | RO | - |
| `0x0B–0x0C` | Temperature (°C, integer) | RO | - |
| `0x0D–0x0E` | Full voltage (mV) | RW | 4200 |
| `0x0F–0x10` | Empty voltage (mV) | RW | 3000 |
| `0x11–0x12` | Protection voltage (mV) | RW | 2800 |
| `0x13–0x14` | Battery percent (LSB=percent, MSB=0x00) | RO | - |
| `0x15–0x16` | Battery-level LED check interval, minutes (see [Battery-Level LED Check](#battery-level-led-check)) | RW | 2 |
| `0x17` | Power status (bit0=power; bit1=IP5328 reset in progress, see [IP5328 Periodic Reset](#ip5328-periodic-reset)) | RO | - |
| `0x18` | Shutdown countdown seconds | RW | 0 |
| `0x19` | Auto power on (0/1) | RW | 0 |
| `0x1A` | Restart countdown seconds | RW | 0 |
| `0x1B` | Factory reset (write 1 to reset) | RW | N/A |
| `0x1C–0x1F` | Cumulative runtime (s) | RO | - |
| `0x20–0x23` | Charging time (s) | RO | - |
| `0x24–0x27` | Current runtime (s) | RO | - |
| `0x28–0x29` | Firmware version | RO | - |
| `0x2A` | Battery parameters self-programmed (0=self-program, 1=manual) | RW | 0 |
| `0x2B` | Low battery percent threshold | RW | 10 |
| `0x2C–0x2D` | Load on delay (seconds) | RW | 60 |
| `0x2E–0x2F` | Output current (int16, 1 mA/LSB) | RO | - |
| `0x30–0x31` | Battery current (int16, 1 mA/LSB) | RO | - |
| `0x32` | Current valid flags (bit0=output, bit1=battery) | RO | - |
| `0x33–0xEF` | Reserved (reads 0x00, writes ignored) | RO | 0 |
| `0xF0–0xFB` | MCU serial number | RO | - |
| `0xFC–0xFF` | Factory Testing (selector + pages). Write **0x7F** to **0xFC** = OTA command (one-shot; see Firmware Update) | RW | selector=0 |

## Documentation

- Development setup and build steps: `documents/Development_Guide.md`
- Behavior specification (contract for features): `documents/UPSPlus_Behavior_Spec.md`
- Debugging guide, I2C examples, and test script usage: `documents/UPSPlus_Debugging_Guide.md`
- Feature development plans are archived in `documents/archive/`.

## NUT Driver

A pre-built Linux NUT driver for UPSPlus is available here:
[https://github.com/dacarson/nut/releases/tag/v2.8.5-upsplus](https://github.com/dacarson/nut/releases/tag/v2.8.5-upsplus)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Disclaimer

This firmware is provided as-is. Use at your own risk. The authors are not responsible for any damage to hardware or data loss resulting from the use of this firmware.

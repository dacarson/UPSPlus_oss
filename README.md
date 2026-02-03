# UPSPlus Open Source Firmware

Open-source firmware for the UPSPlus uninterruptible power supply (UPS) module (EP-0136) designed for Raspberry Pi. [The device has been declared as EOL](https://github.com/geeekpi/upsplus/blob/main/firmware/README.md).
This firmware runs on the STM32F030F4P6 microcontroller and provides battery monitoring, power management, and I2C communication capabilities.

The code has been re-written from the ground up using a combination of ChatGPT and Cursor and the `documents/UPSPlus_Refactoring_Plan.md` document.

## Hardware Requirements

- [UPSPlus HAT EP-0136](https://wiki.52pi.com/index.php?title=EP-0136)
- Raspberry Pi (compatible models)

## Firmware Features

Items marked with `(*)` did not exist in the original factory firmware.

- Battery monitoring via I2C register map.
- Protection shutdown at configured protection voltage.
- Auto power-on with configurable low-battery threshold and load-on delay.(*)
- Boot brownout backoff learning (see [Boot Brownout Backoff](#boot-brownout-backoff)).(*)
- Charging plateau detection for adaptive full-battery learning.(*)
- INA219 current reporting via the 0x17 I2C registers (no separate INA bus reads required).(*)
- Factory Testing pages (0xFC–0xFF) for diagnostics.(*)
- OTA firmware update support (legacy bootloader mode only; not part of this runtime firmware).

## Button Behavior

- The Func Key button is debounced at 50 ms.
- If the load is off, short press powers on the load when the UPS is off and conditions are safe (aka there is sufficient charge in the batteries).
- If the load is on, short press has no action.
- Long press (>= 10 s) when the load is on: powers off.
- Long press (>= 10 s) when the load is off: triggers a factory reset.

## Boot Brownout Backoff

If the RPi browns out shortly after power-on (battery voltage drops to or below the protection
threshold within 5 minutes of RPi being powerd on), the firmware treats this as a boot failure
and increments the internal `load.on.delay` by 1 minute before the next auto power-on attempt.
Learning stops after a successful 5-minute run without a protection-triggered shutdown. The
learned delay is clamped to 60 minutes, persists across power cycles, and is cleared only by
factory reset. A user write to `load.on.delay` becomes the new baseline; any further learning
adds minutes on top of that value.

## Firmware Update (OTA - Legacy Bootloader)

This workflow targets the legacy bootloader mode and is not supported by the
runtime firmware register map in this branch.

1. Ensure the UPSPlus is connected to your Raspberry Pi.
2. Put the UPSPlus into OTA mode by removing all power supplies and batteries.
3. Hold the Func Key button down and insert batteries.
4. Verify OTA mode with `i2cdetect -y 1`; register 0x18 should be visible.
5. Copy the compiled binary to the same directory as `tools/OTA_upgrade.py` and rename it to `upsplus_oss.bin` if needed.
6. Run:

```
python3 tools/OTA_upgrade.py
```

## I2C Register Map and Reading

I2C address: `0x17` on bus `1` (Raspberry Pi default).

Read examples:

```
i2cget -y 1 0x17 0x01
i2cget -y 1 0x17 0x0D w
i2cdump -y 1 0x17
```

Note: multi-byte registers are little-endian (LSB at lower address).

### Register Summary

| Address range | Description | Access | Default |
| --- | --- | --- | --- |
| `0x01–0x02` | MCU voltage (mV) | RO | - |
| `0x03–0x04` | Pogopin voltage (mV) | RO | - |
| `0x05–0x06` | Battery voltage (mV) | RO | - |
| `0x07–0x08` | USB-C voltage (mV) | RO | - |
| `0x09–0x0A` | Micro-USB voltage (mV) | RO | - |
| `0x0B–0x0C` | Temperature (°C, integer) | RO | - |
| `0x0D–0x0E` | Full voltage (mV) | RW | 4200 |
| `0x0F–0x10` | Empty voltage (mV) | RW | 3000 |
| `0x11–0x12` | Protection voltage (mV) | RW | 2800 |
| `0x13–0x14` | Battery percent (LSB=percent, MSB=0x00) | RO | - |
| `0x15–0x16` | Sample period minutes | RW | 2 |
| `0x17` | Power status (bit0=power, bit1=calibration window active) | RO | - |
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
| `0xFC–0xFF` | Factory Testing (RW selector + RO pages) | RW | selector=0 |


## Documentation

- Development setup and build steps: `documents/Development_Guide.md`
- Behavior specification (contract for features): `documents/UPSPlus_Behavior_Spec.md`
- Debugging guide and test script usage: `documents/UPSPlus_Debugging_Guide.md`
- Feature development plans are archived in `documents/archive/`.

## NUT Driver

A pre-built Linux NUT driver for UPSPlus is available here:
https://github.com/dacarson/nut/releases/tag/v2.8.2-upsplus

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Disclaimer

This firmware is provided as-is. Use at your own risk. The authors are not responsible for any damage to hardware or data loss resulting from the use of this firmware.

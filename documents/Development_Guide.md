# UPSPlus Development Guide

This guide contains the development setup, build, flashing, and testing details for the UPSPlus firmware.

---

## 1. Prerequisites

- Install STM32CubeIDE: https://www.st.com/en/development-tools/stm32cubeide.html
- Ensure Python 3 is available on the host used for I2C testing.

---

## 2. Project Setup (STM32CubeIDE)

1. Create a new project in STM32CubeIDE named `UPSPlus_oss` and select the **STM32F030F4P6** MCU.
2. Clone this repository into the new project directory.
3. Clone the required STM32 driver repositories into your project:

```
git clone https://github.com/STMicroelectronics/STM32CubeF0.git
mv STM32CubeF0/Drivers Drivers
rm -rf STM32CubeF0

git clone https://github.com/STMicroelectronics/stm32f0xx-hal-driver.git Drivers/STM32F0xx_HAL_Driver
git clone https://github.com/STMicroelectronics/cmsis-device-f0.git Drivers/CMSIS/Device/ST/STM32F0xx
git clone https://github.com/ARM-software/CMSIS_5.git Drivers/CMSIS
```

4. Add drivers to the source path (Src & Inc):
   - Right-click the project → **Properties**
   - Navigate to **C/C++ Build** → **Settings**
   - Go to **MCU GCC Compiler** → **Include paths**
   - Add:
     - `../Drivers/STM32F0xx_HAL_Driver/Inc`
     - `../Drivers/CMSIS/Device/ST/STM32F0xx/Include`
     - `../Drivers/CMSIS/Include`

5. Set project definitions:
   - Right-click the project → **Properties**
   - Navigate to **C/C++ Build** → **Settings**
   - Go to **MCU GCC Compiler** → **Preprocessor**
   - Add:
     - `STM32F030x6`
     - `USE_FULL_LL_DRIVER`

6. Generate a .bin (as well as .elf):
   - Right-click the project → **Properties**
   - Navigate to **C/C++ Build** → **Settings**
   - Go to **MCU/MPU Post build outputs**
   - Select:
     - `Convert to binary file (-O binary)`

---

## 3. Build

1. Open the project in STM32CubeIDE.
2. Build the project using **Project** → **Build All** (or `Ctrl+B` / `Cmd+B`).
3. The compiled binary will be generated in the `Debug` or `Release` folder.

---

## 4. Flashing (OTA)

1. Ensure the UPSPlus is connected to the Raspberry Pi.
2. Put the UPSPlus into OTA mode by removing all power supplies and batteries.
3. Hold the Func Key button down and insert batteries.
4. Check that it is in OTA mode with `i2cdetect -y 1`; register 0x18 should be visible.
5. Copy the compiled binary to the same directory as `tools/OTA_upgrade.py` and rename it to `upsplus_oss.bin` if needed.
6. Run the OTA upgrade script:

```
python3 tools/OTA_upgrade.py
```

---

## 5. I2C Test Script

The automated I2C test script validates the register map, Factory Testing pages, and state-machine
monitoring.

Install dependency:

```
pip3 install smbus2
```

Run:

```
python3 tools/testing/upsplus_i2c_test.py
```

Optional flags:
- `--allow-power-actions`
- `--allow-destructive`
- `--state-monitor-seconds <sec>`
- `--state-monitor-interval <sec>`

---

## 6. Project Structure (reference)

```
UPSPlus_oss/
├── Drivers/
├── Inc/
├── Src/
├── Startup/
├── tools/
│   ├── OTA_upgrade.py
│   └── testing/
│       └── upsplus_i2c_test.py
└── documents/
```

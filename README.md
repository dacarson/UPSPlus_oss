# UPSPlus Open Source Firmware

Open-source firmware for the UPSPlus uninterruptible power supply (UPS) module (EP-0136) designed for Raspberry Pi. [The device has been declared as EOL](https://github.com/geeekpi/upsplus/blob/main/firmware/README.md). 
This firmware runs on the STM32F030F4P6 microcontroller and provides battery monitoring, power management, and I2C communication capabilities.

## Additional Features

This code base adds two additional features that are needed for the UPS:
- Low Battery percent setting (0x2B)
- Time delay for power on (0x2C - 0x2D)

When the device gets to Protection Voltage, it will force a power off to the Raspberry Pi. When power is reconnected, it waits until it reaches the Low Battery percent and then the further delay for power on. 
This allows for a safe power down of the Raspberry Pi, performed by an external manager, when the Battery Remaining reaches the Low Battery percent. 
It also allows for a safe power on, when a surge of power is needed, by waiting until there is sufficient charge.

## Hardware Requirements

- [UPSPlus HAT EP-0136](https://wiki.52pi.com/index.php?title=EP-0136)
- Raspberry Pi (compatible models)

## Development Environment Setup

### Prerequisites

1. Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

### Project Setup

1. Create a new project in STM32CubeIDE called UPSPlus_oss and select the **STM32F030F4P6** MCU

2. Clone this repository into the new project directory

3. Clone the required STM32 driver repositories into your project:

```bash
git clone https://github.com/STMicroelectronics/STM32CubeF0.git
mv STM32CubeF0/Drivers Drivers
rm -rf STM32CubeF0

git clone https://github.com/STMicroelectronics/stm32f0xx-hal-driver.git Drivers/STM32F0xx_HAL_Driver
git clone https://github.com/STMicroelectronics/cmsis-device-f0.git Drivers/CMSIS/Device/ST/STM32F0xx
git clone https://github.com/ARM-software/CMSIS_5.git Drivers/CMSIS
```

4. Add drivers to source path (Src & Inc)
   - Right-click the project → **Properties**
   - Navigate to **C/C++ Build** → **Settings**
   - Go to **MCU GCC Compiler** → **Include paths**
   - Add the following paths:
    - `../Drivers/STM32F0xx_HAL_Driver/Inc`
    - `../Drivers/CMSIS/Device/ST/STM32F0xx/Include`
    - `../Drivers/CMSIS/Include`
    
5. Set project definitions
   - Right-click the project → **Properties**
   - Navigate to **C/C++ Build** → **Settings**
   - Go to **MCU GCC Compiler** → **Preprocessor**
   - Add the following definitions:
     - `STM32F030x6`
     - `USE_FULL_LL_DRIVER`

6. Generate a .bin (as well as .elf)
   - Right-click the project → **Properties**
   - Navigate to **C/C++ Build** → **Settings**
   - Go to **MCU/MPU Post build outputs** 
   - Select:
     - `Convert to binary file (-O binary)`

## Building the Project

1. Open the project in STM32CubeIDE
2. Build the project using **Project** → **Build All** (or press `Ctrl+B` / `Cmd+B`)
3. The compiled binary will be generated in the `Debug` or `Release` folder

## Flashing the Firmware

1. Ensure the UPSPlus is connected to your RPi
2. Put the UPSPlus into OTA mode by removing all power supplies and batteries. Hold the Func Key button down and insert batteries
3. Check that it is in OTA mode with 'i2cdetect -y 1' and register 0x18 should be there
4. Copy the compiled binary to the same directory as OTA_upgrade.py script, and rename it to upsplus_oss.bin if needed
4. Run the OTA_upgrade.py script.

## Project Structure

```
UPSPlus_oss/
├── Drivers/          # STM32 HAL and CMSIS drivers
├── Inc/              # Header files
│   ├── I2C_Slave.h
│   ├── pindef.h
│   └── ...
├── Src/              # Source files
│   ├── main.c        # Main application code
│   ├── I2C_Slave.c   # I2C slave implementation
│   └── ...
├── Startup/          # Startup assembly files
└── STM32F030F4PX_FLASH.ld  # Linker script
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Disclaimer

This firmware is provided as-is. Use at your own risk. The authors are not responsible for any damage to hardware or data loss resulting from the use of this firmware.

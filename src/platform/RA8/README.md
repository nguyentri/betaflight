# Betaflight RA8E1 Target

This directory contains the Betaflight target implementation for the Renesas RA8E1 microcontroller, specifically designed for the FPB-RA8E1 development board.

## Overview

The RA8E1 is a high-performance ARM Cortex-M85 microcontroller from Renesas with the following key features:
- 360MHz Arm® Cortex®-M85 with Helium™ MVE running at up to 360MHz
- 1MB Flash memory and 544KB of SRAM including TCM
- 32KB I/D caches and 12KB data flash
- 8-bit CEU camera I/F
- Low power consumption
- Rich connectivity options

## Hardware Configuration

### FPB-RA8E1 Board Pinout

#### SPI0 (IMU and Barometer)
- **MOSI**: P411 (Pin 37)
- **MISO**: P410 (Pin 38)
- **SCK**: P412 (Pin 36)
- **IMU CS**: P413 (Pin 35) - MPU9250
- **BARO CS**: P414 (Pin 34) - BMP280

#### UART9 (Receiver Input)
- **RX**: P602 (Pin 45)
- **TX**: P601 (Pin 46)

#### PWM Outputs (ESC Control)
- **Motor 1**: P500 (Pin 53) - GPT4 Channel A
- **Motor 2**: P501 (Pin 54) - GPT4 Channel B
- **Motor 3**: P502 (Pin 55) - GPT5 Channel A
- **Motor 4**: P503 (Pin 56) - GPT5 Channel B

#### Additional Peripherals
- **LED**: P106 (Pin 16)
- **Buzzer**: P107 (Pin 15)
- **Battery Voltage**: P000 (Pin 64) - ADC Channel 0
- **Current Sensor**: P001 (Pin 63) - ADC Channel 1

## File Structure

```
src/platform/RA8/
├── target/RA8E1/
│   ├── target.h              # Target-specific configuration
│   ├── target.c              # Board initialization
│   └── target.mk             # Target makefile
├── mk/
│   └── RA8E1.mk              # Platform makefile
├── startup/
│   └── startup_ra8e1.s       # Startup assembly code
├── link/
│   └── ra8e1.ld              # Linker script
├── include/
│   └── renesas.h             # Renesas FSP includes
├── system_ra8e1.c            # System initialization
├── platform.h               # Platform definitions
├── bus_spi_ra8.c             # SPI driver
├── serial_uart_ra8.c         # UART driver
├── pwm_output_ra8.c          # PWM output driver
├── io_ra8.c                  # GPIO driver
├── adc_ra8.c                 # ADC driver
├── flash_ra8.c               # Flash driver
├── timer_ra8.c               # Timer driver
└── dma_ra8.c                 # DMA driver
```

## Building

### Prerequisites

1. **ARM GCC Toolchain**: Install arm-none-eabi-gcc
2. **Make**: Standard make utility
3. **Renesas FSP**: Flexible Software Package (included in references)

### Build Commands

```bash
# Clean build
make clean TARGET=RA8E1

# Build firmware
make TARGET=RA8E1

# Build with specific options
make TARGET=RA8E1 DEBUG=1 VERBOSE=1
```

### Using the Build Script

A convenient build script is provided:

```bash
# Make executable (first time only)
chmod +x build_ra8e1.sh

# Run build and validation
./build_ra8e1.sh
```

The script will:
- Validate build environment
- Check required files
- Build the target
- Validate output files
- Display memory usage
- Check for common issues

## Flashing

### Using OpenOCD

```bash
openocd -f interface/jlink.cfg -f target/renesas_ra8e1.cfg \
        -c "program obj/betaflight_RA8E1.hex verify reset exit"
```

### Using J-Link

```bash
JLinkExe -device RA8E1 -if SWD -speed 4000 \
         -CommanderScript flash_ra8e1.jlink
```

### Using Renesas Flash Programmer

1. Open Renesas Flash Programmer
2. Select RA8E1 device
3. Connect via J-Link or E2 Lite
4. Load `obj/betaflight_RA8E1.hex`
5. Program and verify

## Testing

### Automated Build Testing

```bash
./build_ra8e1.sh
```

### Hardware-in-the-Loop Testing

```bash
# Make executable (first time only)
chmod +x test_ra8e1_hardware.sh

# Run hardware tests
./test_ra8e1_hardware.sh
```

The hardware test script will:
- Flash firmware to the board
- Test serial communication
- Test MSP protocol
- Verify sensor detection
- Test motor outputs
- Test receiver inputs

### Manual Testing Checklist

1. **Power On Test**
   - Connect 7-26V power supply
   - Verify LED indicates normal operation
   - Check current consumption (~100mA idle)

2. **Sensor Detection**
   - Connect via Betaflight Configurator
   - Verify IMU (MPU9250) is detected
   - Verify barometer (BMP280) is detected
   - Check sensor data is updating

3. **Motor Output Test**
   - **SAFETY**: Remove propellers first!
   - Connect ESCs to motor outputs
   - Test motor spin direction in configurator
   - Verify PWM signal integrity with oscilloscope

4. **Receiver Input Test**
   - Connect receiver to UART9
   - Configure receiver protocol (SBUS/CRSF/etc.)
   - Verify channel inputs in configurator
   - Test failsafe behavior

5. **Configuration Test**
   - Modify settings in configurator
   - Save configuration
   - Power cycle board
   - Verify settings are retained

## Configuration

### Betaflight Configurator Setup

1. **Connection**
   - Baud rate: 115200
   - Port: Auto-detect or manual selection

2. **Initial Setup**
   - Board orientation: Check and adjust if needed
   - Motor order: Verify correct mapping
   - Receiver: Configure protocol and channels

3. **Sensor Calibration**
   - Accelerometer: Level calibration
   - Magnetometer: Compass calibration (if used)
   - ESC: Calibrate throttle range

### Common Configuration Issues

1. **Sensors Not Detected**
   - Check SPI connections
   - Verify CS pin assignments
   - Check power supply voltage

2. **Motor Outputs Not Working**
   - Verify PWM pin assignments
   - Check ESC power and signal connections
   - Test with multimeter/oscilloscope

3. **Receiver Not Working**
   - Check UART configuration
   - Verify receiver protocol settings
   - Check wiring and power

## Troubleshooting

### Build Issues

1. **Missing FSP Headers**
   ```
   Error: hal_data.h not found
   ```
   - Ensure FSP is properly installed
   - Check include paths in makefile

2. **Linker Errors**
   ```
   Error: undefined reference to 'R_GPT_Open'
   ```
   - Verify FSP libraries are linked
   - Check library paths

3. **Memory Overflow**
   ```
   Error: region 'FLASH' overflowed
   ```
   - Reduce feature set in target.h
   - Optimize compiler flags

### Runtime Issues

1. **Board Not Booting**
   - Check power supply voltage (7-26V)
   - Verify flash programming was successful
   - Check for short circuits

2. **USB Not Recognized**
   - Install Betaflight drivers
   - Try different USB cable/port
   - Check USB configuration in target.h

3. **Sensors Not Working**
   - Verify SPI bus configuration
   - Check sensor power supply
   - Test with logic analyzer

## Development

### Adding New Features

1. **New Peripheral Support**
   - Add FSP driver configuration
   - Implement Betaflight driver interface
   - Update target.h with new pins/settings

2. **Performance Optimization**
   - Use DMA for high-throughput operations
   - Optimize interrupt priorities
   - Consider FAST_CODE placement

3. **Power Management**
   - Implement low-power modes
   - Optimize peripheral clock settings
   - Add battery monitoring

### Code Style

- Follow Betaflight coding standards
- Use FSP HAL functions where possible
- Document platform-specific implementations
- Add error handling for all FSP calls

## Support

### Resources

- [Renesas RA8E1 Datasheet](docs/r01uh0994ej0120-ra8m1.pdf)
- [FPB-RA8E1 User Manual](docs/r20ut5474eg0100-fpb-ra8e1-v1-um.pdf)
- [FSP Documentation](https://www.renesas.com/fsp)
- [Betaflight Wiki](https://betaflight.com/docs/wiki)

### Getting Help

1. **Build Issues**: Check build logs and verify toolchain
2. **Hardware Issues**: Verify connections and power supply
3. **Configuration Issues**: Consult Betaflight documentation
4. **FSP Issues**: Refer to Renesas documentation

### Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes following code style
4. Test thoroughly on hardware
5. Submit pull request with detailed description

## License

This code is licensed under the GNU General Public License v3.0.
See LICENSE file for details.

## Changelog

### v1.0.0 (Initial Release)
- Basic RA8E1 target support
- SPI, UART, PWM, ADC drivers
- FSP integration
- Build and test scripts
- Documentation

### Future Enhancements
- I2C driver implementation
- Advanced timer features
- Low-power mode support
- Additional sensor support
- Performance optimizations
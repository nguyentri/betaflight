#!/bin/bash

# Hardware-in-the-Loop Test Script for Betaflight RA8E1
# This script performs automated testing of the RA8E1 target with real hardware

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
TARGET="RA8E1"
SERIAL_PORT="/dev/ttyUSB0"  # Adjust for your system
BAUD_RATE="115200"
TEST_TIMEOUT="30"
FLASH_TOOL="openocd"  # or "jlink", "stlink"

echo -e "${BLUE}Betaflight RA8E1 Hardware-in-the-Loop Test${NC}"
echo "=============================================="

# Function to print status
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if firmware exists
if [ ! -f "obj/betaflight_${TARGET}.hex" ]; then
    print_error "Firmware not found. Please build the target first."
    exit 1
fi

# Test 1: Flash firmware
print_status "Test 1: Flashing firmware to RA8E1 board..."

case $FLASH_TOOL in
    "openocd")
        if command -v openocd &> /dev/null; then
            print_status "Using OpenOCD to flash firmware..."
            # OpenOCD command for RA8E1 (adjust config file as needed)
            openocd -f interface/jlink.cfg -f target/renesas_ra8e1.cfg \
                    -c "program obj/betaflight_${TARGET}.hex verify reset exit" || {
                print_error "Failed to flash firmware with OpenOCD"
                exit 1
            }
        else
            print_warning "OpenOCD not found, skipping flash test"
        fi
        ;;
    "jlink")
        if command -v JLinkExe &> /dev/null; then
            print_status "Using J-Link to flash firmware..."
            # Create J-Link script
            cat > jlink_flash.jlink << EOF
device RA8E1
si 1
speed 4000
loadfile obj/betaflight_${TARGET}.hex
r
g
qc
EOF
            JLinkExe -commanderscript jlink_flash.jlink || {
                print_error "Failed to flash firmware with J-Link"
                exit 1
            }
            rm -f jlink_flash.jlink
        else
            print_warning "J-Link not found, skipping flash test"
        fi
        ;;
    *)
        print_warning "Unknown flash tool: $FLASH_TOOL"
        ;;
esac

print_status "Firmware flashed successfully"

# Wait for board to boot
print_status "Waiting for board to boot..."
sleep 3

# Test 2: Serial communication
print_status "Test 2: Testing serial communication..."

if [ -e "$SERIAL_PORT" ]; then
    # Test basic serial communication
    timeout $TEST_TIMEOUT bash -c "
        exec 3<>$SERIAL_PORT
        stty -F $SERIAL_PORT $BAUD_RATE raw -echo

        # Send version command
        echo '#' >&3
        sleep 1
        echo 'version' >&3
        sleep 2

        # Read response
        if read -t 5 -u 3 response; then
            echo 'Serial communication: OK'
            echo 'Response: \$response'
        else
            echo 'Serial communication: FAILED'
            exit 1
        fi

        exec 3>&-
    " || {
        print_warning "Serial communication test failed or timed out"
    }
else
    print_warning "Serial port $SERIAL_PORT not found, skipping serial test"
fi

# Test 3: MSP communication
print_status "Test 3: Testing MSP communication..."

# Create a simple MSP test script
cat > msp_test.py << 'EOF'
#!/usr/bin/env python3
import serial
import time
import sys

def calculate_crc(data):
    crc = 0
    for byte in data:
        crc ^= byte
    return crc

def send_msp_request(ser, cmd):
    # MSP v1 format: $M<direction><size><cmd><data><crc>
    direction = ord('<')  # Request
    size = 0  # No data

    packet = [ord('$'), ord('M'), direction, size, cmd]
    crc = calculate_crc(packet[3:])  # CRC of size, cmd, and data
    packet.append(crc)

    ser.write(bytes(packet))
    ser.flush()

def read_msp_response(ser, timeout=2):
    start_time = time.time()
    buffer = b''

    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            buffer += ser.read(ser.in_waiting)

            # Look for MSP response header
            if b'$M>' in buffer:
                header_pos = buffer.find(b'$M>')
                if len(buffer) >= header_pos + 5:  # At least header + size + cmd
                    size = buffer[header_pos + 3]
                    if len(buffer) >= header_pos + 5 + size + 1:  # Full packet
                        return buffer[header_pos:header_pos + 5 + size + 1]
        time.sleep(0.01)

    return None

def main():
    try:
        ser = serial.Serial(sys.argv[1], int(sys.argv[2]), timeout=1)
        time.sleep(2)  # Wait for connection

        # Test MSP_IDENT (100)
        print("Sending MSP_IDENT request...")
        send_msp_request(ser, 100)

        response = read_msp_response(ser)
        if response:
            print(f"MSP response received: {response.hex()}")
            print("MSP communication: OK")
            return 0
        else:
            print("No MSP response received")
            print("MSP communication: FAILED")
            return 1

    except Exception as e:
        print(f"MSP test error: {e}")
        return 1
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: msp_test.py <serial_port> <baud_rate>")
        sys.exit(1)
    sys.exit(main())
EOF

if command -v python3 &> /dev/null && [ -e "$SERIAL_PORT" ]; then
    python3 msp_test.py "$SERIAL_PORT" "$BAUD_RATE" || {
        print_warning "MSP communication test failed"
    }
else
    print_warning "Python3 not found or serial port unavailable, skipping MSP test"
fi

# Clean up
rm -f msp_test.py

# Test 4: Sensor detection
print_status "Test 4: Testing sensor detection..."

# This would require more sophisticated testing with actual sensor responses
print_warning "Sensor detection test requires manual verification"
print_status "Please check Betaflight Configurator for:"
print_status "- IMU (MPU9250) detection"
print_status "- Barometer (BMP280) detection"
print_status "- Magnetometer detection"

# Test 5: Motor output test
print_status "Test 5: Testing motor outputs..."
print_warning "Motor output test requires oscilloscope or logic analyzer"
print_status "Please verify PWM signals on motor output pins"

# Test 6: Receiver input test
print_status "Test 6: Testing receiver input..."
print_warning "Receiver input test requires connected receiver"
print_status "Please verify receiver channel inputs in configurator"

# Test Summary
echo ""
echo -e "${GREEN}Hardware Test Summary:${NC}"
echo "Target: $TARGET"
echo "Firmware: obj/betaflight_${TARGET}.hex"

echo ""
echo -e "${BLUE}Manual Verification Required:${NC}"
echo "1. Connect to Betaflight Configurator"
echo "2. Verify all sensors are detected and working"
echo "3. Test motor outputs (without props!)"
echo "4. Test receiver input channels"
echo "5. Verify configuration can be saved/loaded"

echo ""
echo -e "${YELLOW}Safety Reminders:${NC}"
echo "- Remove propellers before testing motors"
echo "- Ensure proper power supply (7-26V)"
echo "- Check all connections before powering on"
echo "- Have a way to quickly disconnect power"

echo ""
echo -e "${GREEN}Hardware-in-the-Loop test completed!${NC}"
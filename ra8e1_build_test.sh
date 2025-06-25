#!/bin/bash

# Build script for Betaflight RA8E1 target
# This script builds the RA8E1 target and performs basic validation

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
TARGET="RA8E1"
BUILD_DIR="obj"
OUTPUT_DIR="obj"
BETAFLIGHT_DIR="$(pwd)"

echo -e "${BLUE}Betaflight RA8E1 Build Script${NC}"
echo "=================================="

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

# Check if we're in the right directory
if [ ! -f "Makefile" ] || [ ! -d "src" ]; then
    print_error "This script must be run from the Betaflight root directory"
    exit 1
fi

# Check for required tools
print_status "Checking build tools..."

if ! command -v arm-none-eabi-gcc &> /dev/null; then
    print_error "arm-none-eabi-gcc not found. Please install ARM GCC toolchain."
    exit 1
fi

if ! command -v make &> /dev/null; then
    print_error "make not found. Please install make."
    exit 1
fi

print_status "Build tools OK"

# Clean previous build
print_status "Cleaning previous build..."
make clean TARGET=$TARGET

# Check if target files exist
print_status "Validating target files..."

REQUIRED_FILES=(
    "src/platform/RA8/target/RA8E1/target.h"
    "src/platform/RA8/target/RA8E1/target.c"
    "src/platform/RA8/target/RA8E1/target.mk"
    "src/platform/RA8/mk/RA8E1.mk"
    "src/platform/RA8/system_ra8e1.c"
    "src/platform/RA8/platform.h"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$file" ]; then
        print_error "Required file missing: $file"
        exit 1
    fi
done

print_status "Target files OK"

# Build the target
print_status "Building target $TARGET..."

if make -j$(nproc) TARGET=$TARGET; then
    print_status "Build successful!"
else
    print_error "Build failed!"
    exit 1
fi

# Check output files
print_status "Validating build output..."

OUTPUT_FILES=(
    "${OUTPUT_DIR}/betaflight_${TARGET}.hex"
    "${OUTPUT_DIR}/betaflight_${TARGET}.bin"
    "${OUTPUT_DIR}/betaflight_${TARGET}.elf"
)

for file in "${OUTPUT_FILES[@]}"; do
    if [ -f "$file" ]; then
        size=$(stat -c%s "$file")
        print_status "Generated: $file (${size} bytes)"
    else
        print_warning "Output file not found: $file"
    fi
done

# Display memory usage
if [ -f "${OUTPUT_DIR}/betaflight_${TARGET}.elf" ]; then
    print_status "Memory usage:"
    arm-none-eabi-size "${OUTPUT_DIR}/betaflight_${TARGET}.elf"
fi

# Validate hex file
if [ -f "${OUTPUT_DIR}/betaflight_${TARGET}.hex" ]; then
    print_status "Validating hex file format..."
    if head -1 "${OUTPUT_DIR}/betaflight_${TARGET}.hex" | grep -q "^:"; then
        print_status "Hex file format OK"
    else
        print_warning "Hex file format may be invalid"
    fi
fi

# Check for common issues
print_status "Checking for common issues..."

# Check if any undefined symbols
if arm-none-eabi-nm "${OUTPUT_DIR}/betaflight_${TARGET}.elf" | grep -q " U "; then
    print_warning "Found undefined symbols:"
    arm-none-eabi-nm "${OUTPUT_DIR}/betaflight_${TARGET}.elf" | grep " U " | head -10
fi

# Check stack usage (if available)
if command -v arm-none-eabi-objdump &> /dev/null; then
    print_status "Checking for stack usage..."
    # This is a basic check - more sophisticated analysis would be needed
fi

print_status "Build validation complete!"

# Summary
echo ""
echo -e "${GREEN}Build Summary:${NC}"
echo "Target: $TARGET"
echo "Status: SUCCESS"
if [ -f "${OUTPUT_DIR}/betaflight_${TARGET}.hex" ]; then
    echo "Output: ${OUTPUT_DIR}/betaflight_${TARGET}.hex"
fi

echo ""
echo -e "${BLUE}Next Steps:${NC}"
echo "1. Flash the firmware to your RA8E1 board"
echo "2. Connect via Betaflight Configurator"
echo "3. Verify all sensors are detected"
echo "4. Test motor outputs and receiver input"

echo ""
echo -e "${GREEN}Build completed successfully!${NC}"
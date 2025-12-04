# Zephyr MAX32664C Driver

Out-of-tree Zephyr driver for the Maxim MAX32664C biometric sensor hub.

## Overview

This repository provides a Zephyr driver for the MAX32664C sensor hub, which is
designed for heart rate and SpO2 monitoring applications. The driver supports:

- Heart rate measurement (WHRM)
- Blood oxygen saturation (SpO2)
- Respiration rate
- Activity classification
- Skin contact detection (SCD)

## Repository Structure

```
├── drivers/           # Driver source code
├── dts/               # Device tree bindings
├── include/           # Public API headers
├── zephyr/            # Zephyr module configuration (module.yml)
├── samples/app/       # Sample application
├── west.yml           # West manifest for standalone development
└── Kconfig.max32664c  # Driver Kconfig
```

## Using as a Zephyr Module

### Option 1: Add to your west.yml (recommended for your own projects)

Add this repository as a project in your `west.yml`:

```yaml
manifest:
  remotes:
    - name: zswatch
      url-base: https://github.com/ZSWatch

  projects:
    - name: Zephyr-MAX32664C
      remote: zswatch
      revision: master
      path: modules/lib/max32664c
```

### Option 2: Use the included west manifest for standalone development

```bash
# Create a workspace directory and clone this repo
mkdir my-workspace && cd my-workspace
git clone https://github.com/ZSWatch/Zephyr-MAX32664C.git max32664c
cd max32664c
west init -l .
west update

# Build the sample app
cd samples/app
source ./env.sh
west build -b nrf54l15dk/nrf54l15/cpuapp -- -DDTC_OVERLAY_FILE=boards/nrf54l15dk_nrf54l15_cpuapp_v2.overlay

# Flash
west flash
```

## Configuration

Enable the driver in your `prj.conf`:

```
CONFIG_SENSOR=y
CONFIG_I2C=y
CONFIG_SENSOR_MAX32664C=y
```

### Kconfig Options

- `CONFIG_SENSOR_MAX32664C` - Enable the MAX32664C driver
- `CONFIG_MAX32664C_USE_FIRMWARE_LOADER` - Enable I2C firmware update
- `CONFIG_MAX32664C_USE_EXTERNAL_ACC` - Use external accelerometer
- `CONFIG_MAX32664C_USE_EXTENDED_REPORTS` - Enable extended algorithm reports
- `CONFIG_MAX32664C_USE_STATIC_MEMORY` - Use static memory allocation
- `CONFIG_MAX32664C_USE_INTERRUPT` - Enable MFIO interrupt support

## Device Tree

Example device tree configuration:

```dts
&i2c0 {
    biometric_hub: max32664c@55 {
        compatible = "maxim,max32664c";
        reg = <0x55>;
        status = "okay";
        reset-gpios = <&gpio0 0 GPIO_ACTIVE_HIGH>;
        mfio-gpios = <&gpio0 1 GPIO_ACTIVE_HIGH>;
        use-max86141;
    };
};
```

## Sample Application

The `samples/app/` directory contains a complete sample application demonstrating:
- BLE Heart Rate Service integration
- Sensor configuration and data reading
- Firmware update capability

## Supported Boards

- nRF54L15DK

## License

Apache-2.0
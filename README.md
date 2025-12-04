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


## Using as a Zephyr Module

### Option 1: Add to your west.yml

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

### Option 2: Use as a standalone sample

```bash
git clone https://github.com/ZSWatch/Zephyr-MAX32664C.git max32664c
cd max32664c

cd samples/app
west build -b nrf54l15dk/nrf54l15/cpuapp -- -DDTC_OVERLAY_FILE=boards/nrf54l15dk_nrf54l15_cpuapp_v2.overlay

# Flash
west flash
```

## Device Tree
See `samples/app/boards/nrf54l15dk_nrf54l15_cpuapp_v2.overlay`

## Sample Application

The `samples/app/` directory contains a complete sample application demonstrating:
- BLE Heart Rate Service integration
- Sensor configuration and data reading
- Firmware update capability

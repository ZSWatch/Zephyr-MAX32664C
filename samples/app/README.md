# MAX32664C Sample Application

Sample application demonstrating the MAX32664C biometric sensor hub driver.

## Building

See the main repository README for full setup instructions.

```bash
# From within a properly initialized west workspace:
cd samples/app
source ./env.sh
west build -b nrf54l15dk/nrf54l15/cpuapp -- -DDTC_OVERLAY_FILE=boards/nrf54l15dk_nrf54l15_cpuapp_v2.overlay

# Flash
west flash
```

## Maintainer

- [Daniel Kampert](mailto:daniel.kameprt@kampis-elektroecke.de)

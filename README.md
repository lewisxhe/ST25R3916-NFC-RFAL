# ST25R3916-NFC-RFAL

Self-contained Arduino/PlatformIO library that merges:

- `ST25R3916-fork`: ST25R3916/ST25R3916B RF driver
- `NFC-RFAL-fork`: RFAL, NFC activity layer, NDEF helpers, and examples

The merged `src` directory contains both layers, so examples do not need
separate `lib_deps`.

## Quick Test

From this directory:

```bash
pio run -e card_emulation
pio run -e card_emulation_t4t
pio run -e card_emulation_t3t
pio run -e card_emulation_wifi
pio run -e card_emulation_wifi_f
pio run -e card_emulation_wifi_mixed
pio run -e polling
pio run -e cap_sense_mode_switch
pio run -e cap_sense_mode_switch_i2c
pio run -e ndef_rw
```

Upload one example:

```bash
pio run -e card_emulation -t upload
pio device monitor -b 115200
```

Default pins are defined in each sketch and can be overridden with build flags:

```ini
build_flags =
  -DST25R3916_PIN_MOSI=23
  -DST25R3916_PIN_MISO=19
  -DST25R3916_PIN_SCK=18
  -DST25R3916_PIN_CS=5
  -DST25R3916_PIN_IRQ=2
  -DST25R3916_PIN_SDA=21
  -DST25R3916_PIN_SCL=22
  -DMCU_LED6=14
  -DMCU_LED1=4
```

SPI is the default host interface. All Arduino examples can be switched to I2C
with:

```ini
build_flags =
  -DST25R3916_USE_I2C=1
  -DST25R3916_PIN_SDA=21
  -DST25R3916_PIN_SCL=22
```

When I2C is enabled, `CS` is unused and `IRQ` is still required. The I2C clock
defaults to 400 kHz and can be overridden with `-DST25R3916_I2C_CLOCK=...`.

## Examples

- `CardEmulation`: NFC-A Type 4 Tag + NFC-F Type 3 Tag card emulation
- `CardEmulationT4T`: NFC-A Type 4 Tag only
- `CardEmulationT3T`: NFC-F Type 3 Tag only
- `CardEmulationWifi`: NFC-F card emulation with Wi-Fi NDEF data based on `record_wifi.c`
- `card_emulation_wifi_f`: PlatformIO-only alias for the NFC-F Wi-Fi CE build
- `card_emulation_wifi_mixed`: PlatformIO-only Wi-Fi CE build with NFC-A and NFC-F enabled for ISO-DEP debugging
- `Polling`: basic card polling and NFCID logging
- `CapSenseModeSwitch`: ST25R3916 capacitive sensor monitor with Serial commands to switch reader and card-emulation modes
- `cap_sense_mode_switch_i2c`: PlatformIO-only I2C build of `CapSenseModeSwitch`
- `NdefReadWrite`: NDEF read by default, optional write with `NDEF_EXAMPLE_ENABLE_WRITE`

## Logs

Sketch logs are enabled by default. Library-level CE logs are enabled in the CE
PlatformIO environments with `RFAL_CE_DEBUG`.

Fine-grained CE debug switches:

```ini
build_flags =
  -DRFAL_ST25R3916_LISTEN_DEBUG
  -DRFAL_NFC_CE_DEBUG
  -DRFAL_ISODEP_CE_DEBUG
  -DRFAL_DEMO_CE_DEBUG
```

The ESP32 GPIO ISR only marks an interrupt pending. RFAL workers and protocol
processing must keep running from the Arduino `loop()`.

`CapSenseModeSwitch` starts in `IDLE`. Serial commands: `poll`, `ce`, `idle`,
`cap`, `cal`, `th <n>`, `help`. Capacitive samples and calibration run only
while the example is idle, so RF activity is not disturbed. The card-emulation
side defaults to NFC-F; add `-DCAP_SWITCH_CE_MIXED_TECHS` if NFC-A + NFC-F
mixed listen mode is needed for debugging.

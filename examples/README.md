# Arduino Examples

These sketches are Arduino entry points for the self-contained
`ST25R3916-NFC-RFAL` library. The RFAL layer and ST25R3916 driver are both in
the library `src` directory.

## ST Example Mapping

- ST `Common/Src/demo_ce.c`: `CardEmulation`, `CardEmulationT4T`,
  `CardEmulationT3T`, `CardEmulationWifi`
- ST `Common/Src/demo_polling.c`: `Polling`
- ST25R3916 capacitive sensor APIs: `CapSenseModeSwitch`
- ST `X-NUCLEO-NFC06A1/ndef_rw`: `NdefReadWrite`
- Local nRF52 `record_wifi.c`: `CardEmulationWifi` Wi-Fi NDEF payload data

The ST `pairing`, `FTM`, `edta`, `ap2p_proprietary`, and ST25DV PWM application
projects depend on extra board UI, ST25FTM, certification harnesses, secure
element flows, or app-specific payload handling. They are not duplicated here as
empty Arduino shells.

## Default Pins

- MOSI: 23
- MISO: 19
- SCK: 18
- CS: 5
- IRQ: 2
- I2C SDA: 21
- I2C SCL: 22
- MCU_LED6: 14
- MCU_LED1: 4

SPI is used by default. Add this build flag to use I2C instead:

```ini
build_flags =
  -DST25R3916_USE_I2C=1
```

I2C pins can be overridden with `ST25R3916_PIN_SDA` and
`ST25R3916_PIN_SCL`; `ST25R3916_I2C_CLOCK` defaults to 400 kHz.

## Debug Switches

Sketch logs are enabled by default and can be disabled with:

```ini
build_flags =
  -DCE_EXAMPLE_DEBUG=0
  -DCE_WIFI_EXAMPLE_DEBUG=0
  -DCAP_SWITCH_EXAMPLE_DEBUG=0
  -DPOLLING_EXAMPLE_DEBUG=0
  -DNDEF_EXAMPLE_DEBUG=0
```

Library-level card-emulation logs are disabled by default. Enable all CE logs
with:

```ini
build_flags =
  -DRFAL_CE_DEBUG
```

Or enable narrower modules:

```ini
build_flags =
  -DRFAL_ST25R3916_LISTEN_DEBUG
  -DRFAL_NFC_CE_DEBUG
  -DRFAL_ISODEP_CE_DEBUG
  -DRFAL_DEMO_CE_DEBUG
```

`CardEmulationWifi` also supports focused technology tests:

```ini
build_flags =
  -DCE_WIFI_NFCA_ONLY
  -DCE_WIFI_MIXED_TECHS
```

By default `CardEmulationWifi` enables NFC-F only, because NFC-A ISO-DEP
activation is still under diagnosis on ESP32.

`CapSenseModeSwitch` starts in `IDLE` and accepts Serial commands: `poll`,
`ce`, `idle`, `cap`, `cal`, `th <n>`, `help`. It samples the ST25R3916
capacitive sensor in `loop()` only while idle and does not use ST's wake-up
interrupt flow.

The ESP32 IRQ handler in this library only marks the interrupt as pending.
RFAL worker and protocol processing must keep running from `loop()`.

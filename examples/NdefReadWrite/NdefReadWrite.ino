#include <Arduino.h>
#include <SPI.h>
#include <string.h>

#include <rfal_rfst25r3916.h>
#include <st25r3916_arduino_bus.h>
#include <rfal_nfc.h>
#include <ndef_class.h>

#ifndef NDEF_EXAMPLE_DEBUG
#define NDEF_EXAMPLE_DEBUG 1
#endif

#ifndef NDEF_EXAMPLE_ENABLE_WRITE
#define NDEF_EXAMPLE_ENABLE_WRITE 0
#endif

#if NDEF_EXAMPLE_DEBUG
#define NDEF_LOG(msg) do { Serial.print("[NDEF EXAMPLE] "); Serial.println(msg); } while (0)
#define NDEF_LOG_VAL(msg, val) do { Serial.print("[NDEF EXAMPLE] "); Serial.print(msg); Serial.println((int32_t)(val)); } while (0)
#else
#define NDEF_LOG(msg) do { } while (0)
#define NDEF_LOG_VAL(msg, val) do { } while (0)
#endif

#ifndef ST25R3916_PIN_CS
#define ST25R3916_PIN_CS 5
#endif

#ifndef ST25R3916_PIN_IRQ
#define ST25R3916_PIN_IRQ 2
#endif

#ifndef ST25R3916_PIN_SCK
#define ST25R3916_PIN_SCK 18
#endif

#ifndef ST25R3916_PIN_MISO
#define ST25R3916_PIN_MISO 19
#endif

#ifndef ST25R3916_PIN_MOSI
#define ST25R3916_PIN_MOSI 23
#endif

#ifndef MCU_LED6
#define MCU_LED6 14
#endif

#ifndef MCU_LED1
#define MCU_LED1 4
#endif

static RfalRfST25R3916Class rfalRf(ST25R3916_RFAL_HOST_INTERFACE);
static RfalNfcClass nfc(&rfalRf);
static NdefClass ndef(&nfc);
static rfalNfcDiscoverParam discParam;

static bool activeHandled = false;
static uint8_t ndefBuf[512];

static const uint8_t rawUriMessage[] = {
  0xD1, 0x01, 0x07, 0x55, 0x04, 's', 't', '.', 'c', 'o', 'm'
};

static void printHex(const uint8_t *data, uint32_t len)
{
#if NDEF_EXAMPLE_DEBUG
  if (data == NULL) {
    return;
  }

  for (uint32_t i = 0; i < len; i++) {
    if (data[i] < 0x10U) {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
    if ((i + 1U) < len) {
      Serial.print(' ');
    }
  }
#else
  (void)data;
  (void)len;
#endif
}

static const char *ndefStateName(ndefState state)
{
  switch (state) {
    case NDEF_STATE_INITIALIZED:
      return "initialized";
    case NDEF_STATE_READWRITE:
      return "readwrite";
    case NDEF_STATE_READONLY:
      return "readonly";
    default:
      return "invalid";
  }
}

static void onNfcState(rfalNfcState state)
{
  if (state == RFAL_NFC_STATE_POLL_SELECT) {
    rfalNfcDevice *devList = NULL;
    uint8_t devCnt = 0;
    if (nfc.rfalNfcGetDevicesFound(&devList, &devCnt) == ST_ERR_NONE) {
      NDEF_LOG_VAL("Multiple devices found count=", devCnt);
      (void)nfc.rfalNfcSelect(0U);
    }
  } else if (state == RFAL_NFC_STATE_START_DISCOVERY) {
    activeHandled = false;
  }
}

static void configureDiscovery()
{
  memset(&discParam, 0, sizeof(discParam));
  discParam.compMode = RFAL_COMPLIANCE_MODE_NFC;
  discParam.techs2Find = (RFAL_NFC_POLL_TECH_A |
                          RFAL_NFC_POLL_TECH_B |
                          RFAL_NFC_POLL_TECH_F |
                          RFAL_NFC_POLL_TECH_V);
  discParam.totalDuration = 1000U;
  discParam.devLimit = 1U;
  discParam.nfcfBR = RFAL_BR_212;
  discParam.ap2pBR = RFAL_BR_424;
  discParam.notifyCb = onNfcState;
  discParam.wakeupConfigDefault = true;
}

static void restartDiscovery()
{
  activeHandled = false;
  ReturnCode err = nfc.rfalNfcDiscover(&discParam);
  if (err != ST_ERR_NONE) {
    NDEF_LOG_VAL("rfalNfcDiscover failed err=", err);
  } else {
    NDEF_LOG("Polling for an NDEF tag");
  }
}

static void handleNdefTag(rfalNfcDevice *dev)
{
  ndefInfo info;
  uint32_t rawLen = 0U;

  ReturnCode err = ndef.ndefPollerContextInitialization(dev);
  if (err != ST_ERR_NONE) {
    NDEF_LOG_VAL("NDEF context init failed err=", err);
    return;
  }

  err = ndef.ndefPollerNdefDetect(&info);
  if (err != ST_ERR_NONE) {
    NDEF_LOG_VAL("NDEF detect failed err=", err);
    return;
  }

#if NDEF_EXAMPLE_DEBUG
  Serial.print("[NDEF EXAMPLE] state=");
  Serial.print(ndefStateName(info.state));
  Serial.print(" area=");
  Serial.print(info.areaLen);
  Serial.print(" message=");
  Serial.println(info.messageLen);
#endif

#if NDEF_EXAMPLE_ENABLE_WRITE
  err = ndef.ndefPollerWriteRawMessage(rawUriMessage, sizeof(rawUriMessage));
  if (err != ST_ERR_NONE) {
    NDEF_LOG_VAL("NDEF write failed err=", err);
    return;
  }
  NDEF_LOG("Wrote URI NDEF message");
#else
  (void)rawUriMessage;
#endif

  err = ndef.ndefPollerReadRawMessage(ndefBuf, sizeof(ndefBuf), &rawLen);
  if (err != ST_ERR_NONE) {
    NDEF_LOG_VAL("NDEF read failed err=", err);
    return;
  }

#if NDEF_EXAMPLE_DEBUG
  Serial.print("[NDEF EXAMPLE] Raw NDEF len=");
  Serial.println(rawLen);
  Serial.print("[NDEF EXAMPLE] Raw NDEF=");
  printHex(ndefBuf, rawLen);
  Serial.println();
#endif
}

static void handleActivated()
{
  rfalNfcDevice *dev = NULL;
  ReturnCode err = nfc.rfalNfcGetActiveDevice(&dev);
  if ((err != ST_ERR_NONE) || (dev == NULL)) {
    NDEF_LOG_VAL("rfalNfcGetActiveDevice failed err=", err);
  } else {
    handleNdefTag(dev);
  }

  activeHandled = true;
  nfc.rfalNfcDeactivate(true);
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  st25r3916BeginArduinoBus();

  NDEF_LOG("ST25R3916 NDEF read/write example");
  NDEF_LOG(st25r3916ArduinoBusName());

  ReturnCode err = nfc.rfalNfcInitialize();
  if (err != ST_ERR_NONE) {
    NDEF_LOG_VAL("rfalNfcInitialize failed err=", err);
    while (true) {
      delay(1000);
    }
  }

  configureDiscovery();
  restartDiscovery();
}

void loop()
{
  nfc.rfalNfcWorker();

  switch (nfc.rfalNfcGetState()) {
    case RFAL_NFC_STATE_ACTIVATED:
      if (!activeHandled) {
        handleActivated();
      }
      break;

    case RFAL_NFC_STATE_IDLE:
      restartDiscovery();
      break;

    default:
      break;
  }
}

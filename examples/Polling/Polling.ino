#include <Arduino.h>
#include <SPI.h>
#include <string.h>

#include <rfal_rfst25r3916.h>
#include <st25r3916_arduino_bus.h>
#include <rfal_nfc.h>

#ifndef POLLING_EXAMPLE_DEBUG
#define POLLING_EXAMPLE_DEBUG 1
#endif

#if POLLING_EXAMPLE_DEBUG
#define POLL_LOG(msg) do { Serial.print("[POLL EXAMPLE] "); Serial.println(msg); } while (0)
#define POLL_LOG_VAL(msg, val) do { Serial.print("[POLL EXAMPLE] "); Serial.print(msg); Serial.println((int32_t)(val)); } while (0)
#else
#define POLL_LOG(msg) do { } while (0)
#define POLL_LOG_VAL(msg, val) do { } while (0)
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
static rfalNfcDiscoverParam discParam;
static bool activeReported = false;

static const char *deviceTypeName(rfalNfcDevType type)
{
  switch (type) {
    case RFAL_NFC_LISTEN_TYPE_NFCA:
      return "NFC-A";
    case RFAL_NFC_LISTEN_TYPE_NFCB:
      return "NFC-B";
    case RFAL_NFC_LISTEN_TYPE_NFCF:
      return "NFC-F";
    case RFAL_NFC_LISTEN_TYPE_NFCV:
      return "NFC-V";
    case RFAL_NFC_LISTEN_TYPE_ST25TB:
      return "ST25TB";
    case RFAL_NFC_LISTEN_TYPE_AP2P:
      return "AP2P target";
    default:
      return "unknown";
  }
}

static const char *interfaceName(rfalNfcRfInterface iface)
{
  switch (iface) {
    case RFAL_NFC_INTERFACE_RF:
      return "RF";
    case RFAL_NFC_INTERFACE_ISODEP:
      return "ISO-DEP";
    case RFAL_NFC_INTERFACE_NFCDEP:
      return "NFC-DEP";
    default:
      return "unknown";
  }
}

static void printHex(const uint8_t *data, uint8_t len)
{
#if POLLING_EXAMPLE_DEBUG
  if (data == NULL) {
    return;
  }

  for (uint8_t i = 0; i < len; i++) {
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

static void onNfcState(rfalNfcState state)
{
  if (state == RFAL_NFC_STATE_POLL_SELECT) {
    rfalNfcDevice *devList = NULL;
    uint8_t devCnt = 0;
    if (nfc.rfalNfcGetDevicesFound(&devList, &devCnt) == ST_ERR_NONE) {
      POLL_LOG_VAL("Multiple devices found count=", devCnt);
      (void)nfc.rfalNfcSelect(0U);
    }
  } else if (state == RFAL_NFC_STATE_START_DISCOVERY) {
    activeReported = false;
  }
}

static void configureDiscovery()
{
  memset(&discParam, 0, sizeof(discParam));
  discParam.compMode = RFAL_COMPLIANCE_MODE_NFC;
  discParam.techs2Find = (RFAL_NFC_POLL_TECH_A |
                          RFAL_NFC_POLL_TECH_B |
                          RFAL_NFC_POLL_TECH_F |
                          RFAL_NFC_POLL_TECH_V |
                          RFAL_NFC_POLL_TECH_ST25TB);
  discParam.totalDuration = 1000U;
  discParam.devLimit = 1U;
  discParam.nfcfBR = RFAL_BR_212;
  discParam.ap2pBR = RFAL_BR_424;
  discParam.notifyCb = onNfcState;
  discParam.wakeupConfigDefault = true;
}

static void restartDiscovery()
{
  activeReported = false;
  ReturnCode err = nfc.rfalNfcDiscover(&discParam);
  if (err != ST_ERR_NONE) {
    POLL_LOG_VAL("rfalNfcDiscover failed err=", err);
  } else {
    POLL_LOG("Polling for NFC devices");
  }
}

static void reportActiveDevice()
{
  rfalNfcDevice *dev = NULL;
  ReturnCode err = nfc.rfalNfcGetActiveDevice(&dev);
  if ((err != ST_ERR_NONE) || (dev == NULL)) {
    POLL_LOG_VAL("rfalNfcGetActiveDevice failed err=", err);
    nfc.rfalNfcDeactivate(true);
    return;
  }

#if POLLING_EXAMPLE_DEBUG
  Serial.print("[POLL EXAMPLE] Type=");
  Serial.print(deviceTypeName(dev->type));
  Serial.print(" Interface=");
  Serial.print(interfaceName(dev->rfInterface));
  Serial.print(" NFCID=");
  printHex(dev->nfcid, dev->nfcidLen);
  Serial.println();
#endif

  activeReported = true;
  nfc.rfalNfcDeactivate(true);
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  st25r3916BeginArduinoBus();

  POLL_LOG("ST25R3916 polling example");
  POLL_LOG(st25r3916ArduinoBusName());

  ReturnCode err = nfc.rfalNfcInitialize();
  if (err != ST_ERR_NONE) {
    POLL_LOG_VAL("rfalNfcInitialize failed err=", err);
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
      if (!activeReported) {
        reportActiveDevice();
      }
      break;

    case RFAL_NFC_STATE_IDLE:
      restartDiscovery();
      break;

    default:
      break;
  }
}

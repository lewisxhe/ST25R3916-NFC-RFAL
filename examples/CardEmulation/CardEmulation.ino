#include <Arduino.h>
#include <SPI.h>
#include <string.h>

#include <rfal_rfst25r3916.h>
#include <st25r3916_arduino_bus.h>
#include <rfal_nfc.h>
#include <demo_ce.h>

/*
 * Library-level CE logs are controlled at build time with:
 *   -DRFAL_CE_DEBUG
 * or the narrower switches:
 *   -DRFAL_ST25R3916_LISTEN_DEBUG -DRFAL_NFC_CE_DEBUG
 *   -DRFAL_ISODEP_CE_DEBUG -DRFAL_DEMO_CE_DEBUG
 */
#ifndef CE_EXAMPLE_DEBUG
#define CE_EXAMPLE_DEBUG 1
#endif

#if CE_EXAMPLE_DEBUG
#define CE_LOG(msg) do { Serial.print("[CE EXAMPLE] "); Serial.println(msg); } while (0)
#define CE_LOG_VAL(msg, val) do { Serial.print("[CE EXAMPLE] "); Serial.print(msg); Serial.println((int32_t)(val)); } while (0)
#else
#define CE_LOG(msg) do { } while (0)
#define CE_LOG_VAL(msg, val) do { } while (0)
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

#ifndef CE_EXAMPLE_TECHS
#define CE_EXAMPLE_TECHS (RFAL_NFC_LISTEN_TECH_A | RFAL_NFC_LISTEN_TECH_F)
#endif

static RfalRfST25R3916Class rfalRf(ST25R3916_RFAL_HOST_INTERFACE);
static RfalNfcClass nfc(&rfalRf);
static rfalNfcDiscoverParam discParam;

static uint8_t *rxData = NULL;
static uint16_t *rxLen = NULL;
static uint8_t txBuf[RFAL_NFC_RF_BUF_LEN];
static bool exchangePrimed = false;

static uint8_t ceNfcfNfcid2[RFAL_NFCF_NFCID2_LEN] = {
  0x02, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

static void printHex(const uint8_t *data, uint16_t len)
{
#if CE_EXAMPLE_DEBUG
  if (data == NULL) {
    return;
  }

  for (uint16_t i = 0; i < len; i++) {
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

static const char *stateName(rfalNfcState state)
{
  switch (state) {
    case RFAL_NFC_STATE_IDLE:
      return "IDLE";
    case RFAL_NFC_STATE_START_DISCOVERY:
      return "START_DISCOVERY";
    case RFAL_NFC_STATE_POLL_TECHDETECT:
      return "POLL_TECHDETECT";
    case RFAL_NFC_STATE_LISTEN_TECHDETECT:
      return "LISTEN_TECHDETECT";
    case RFAL_NFC_STATE_LISTEN_COLAVOIDANCE:
      return "LISTEN_COLAVOIDANCE";
    case RFAL_NFC_STATE_LISTEN_ACTIVATION:
      return "LISTEN_ACTIVATION";
    case RFAL_NFC_STATE_LISTEN_SLEEP:
      return "LISTEN_SLEEP";
    case RFAL_NFC_STATE_ACTIVATED:
      return "ACTIVATED";
    case RFAL_NFC_STATE_DATAEXCHANGE:
      return "DATAEXCHANGE";
    case RFAL_NFC_STATE_DATAEXCHANGE_DONE:
      return "DATAEXCHANGE_DONE";
    default:
      return "OTHER";
  }
}

static void onNfcState(rfalNfcState state)
{
#if CE_EXAMPLE_DEBUG
  Serial.print("[CE EXAMPLE] NFC state=");
  Serial.print(stateName(state));
  Serial.print(" (");
  Serial.print((int)state);
  Serial.println(')');
#else
  (void)state;
#endif
}

static void configureDiscovery(uint16_t techs)
{
  static const uint8_t nfcid1[RFAL_NFCID1_TRIPLE_LEN] = {
    0x5F, 'S', 'T', 'M', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  static const uint8_t pmm[8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x00
  };

  memset(&discParam, 0, sizeof(discParam));
  discParam.compMode = RFAL_COMPLIANCE_MODE_NFC;
  discParam.techs2Find = techs;
  discParam.totalDuration = 1000U;
  discParam.devLimit = 1U;
  discParam.nfcfBR = RFAL_BR_212;
  discParam.ap2pBR = RFAL_BR_424;
  discParam.notifyCb = onNfcState;

  discParam.lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_04;
  memcpy(discParam.lmConfigPA.nfcid, nfcid1, sizeof(nfcid1));
  discParam.lmConfigPA.SENS_RES[0] = 0x02;
  discParam.lmConfigPA.SENS_RES[1] = 0x00;
  discParam.lmConfigPA.SEL_RES = 0x20;

  discParam.lmConfigPF.SC[0] = 0x12;
  discParam.lmConfigPF.SC[1] = 0xFC;
  discParam.lmConfigPF.SENSF_RES[0] = RFAL_NFCF_CMD_POLLING_RES;
  memcpy(&discParam.lmConfigPF.SENSF_RES[1], ceNfcfNfcid2, RFAL_NFCF_NFCID2_LEN);
  memcpy(&discParam.lmConfigPF.SENSF_RES[9], pmm, sizeof(pmm));
  discParam.lmConfigPF.SENSF_RES[17] = 0x00;
  discParam.lmConfigPF.SENSF_RES[18] = 0x00;
}

static void restartDiscovery()
{
  exchangePrimed = false;
  rxData = NULL;
  rxLen = NULL;

  ReturnCode err = nfc.rfalNfcDiscover(&discParam);
  if (err != ST_ERR_NONE) {
    CE_LOG_VAL("rfalNfcDiscover failed err=", err);
  } else {
    CE_LOG("Waiting for a reader field");
  }
}

static bool primeListenDataExchange()
{
  ReturnCode err = nfc.rfalNfcDataExchangeStart(NULL, 0U, &rxData, &rxLen, RFAL_FWT_NONE);
  if (err != ST_ERR_NONE) {
    CE_LOG_VAL("prime exchange failed err=", err);
    nfc.rfalNfcDeactivate(true);
    return false;
  }

  exchangePrimed = true;
  (void)nfc.rfalNfcDataExchangeGetStatus();
  CE_LOG("Data exchange primed");
  return true;
}

static void handleDataExchangeDone()
{
  ReturnCode err = nfc.rfalNfcDataExchangeGetStatus();
  if (err == ST_ERR_BUSY) {
    return;
  }

  if (err == ST_ERR_SLEEP_REQ) {
    CE_LOG("Reader requested sleep");
    exchangePrimed = false;
    return;
  }

  if (err != ST_ERR_NONE) {
    CE_LOG_VAL("data exchange failed err=", err);
    nfc.rfalNfcDeactivate(true);
    exchangePrimed = false;
    return;
  }

  rfalNfcDevice *dev = NULL;
  if ((nfc.rfalNfcGetActiveDevice(&dev) != ST_ERR_NONE) || (dev == NULL) || (rxData == NULL) || (rxLen == NULL)) {
    CE_LOG("No active device context");
    nfc.rfalNfcDeactivate(true);
    exchangePrimed = false;
    return;
  }

  uint16_t inLen = *rxLen;
  uint16_t outLen = 0U;

  if ((dev->type == RFAL_NFC_POLL_TYPE_NFCA) && (dev->rfInterface == RFAL_NFC_INTERFACE_ISODEP)) {
    CE_LOG_VAL("T4T APDU len=", inLen);
    outLen = demoCeT4T(rxData, inLen, txBuf, sizeof(txBuf));
  } else if ((dev->type == RFAL_NFC_POLL_TYPE_NFCF) && (dev->rfInterface == RFAL_NFC_INTERFACE_RF)) {
    inLen = rfalConvBitsToBytes(inLen);
    CE_LOG_VAL("T3T command len=", inLen);
    outLen = demoCeT3T(rxData, inLen, txBuf, sizeof(txBuf));
  } else {
    CE_LOG_VAL("Unsupported activated type=", dev->type);
  }

  if (outLen == 0U) {
    CE_LOG("No CE response generated");
    nfc.rfalNfcDeactivate(true);
    exchangePrimed = false;
    return;
  }

#if CE_EXAMPLE_DEBUG
  Serial.print("[CE EXAMPLE] TX ");
  printHex(txBuf, outLen);
  Serial.println();
#endif

  err = nfc.rfalNfcDataExchangeStart(txBuf, outLen, &rxData, &rxLen, RFAL_FWT_NONE);
  if (err != ST_ERR_NONE) {
    CE_LOG_VAL("response exchange failed err=", err);
    nfc.rfalNfcDeactivate(true);
    exchangePrimed = false;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  st25r3916BeginArduinoBus();

  CE_LOG("ST25R3916 card emulation example");
  CE_LOG(st25r3916ArduinoBusName());
  CE_LOG("Use build flag -DRFAL_CE_DEBUG for library logs");

  ReturnCode err = nfc.rfalNfcInitialize();
  if (err != ST_ERR_NONE) {
    CE_LOG_VAL("rfalNfcInitialize failed err=", err);
    while (true) {
      delay(1000);
    }
  }

  configureDiscovery(CE_EXAMPLE_TECHS);
  demoCeInit(ceNfcfNfcid2);
  restartDiscovery();
}

void loop()
{
  nfc.rfalNfcWorker();

  switch (nfc.rfalNfcGetState()) {
    case RFAL_NFC_STATE_ACTIVATED:
      if (!exchangePrimed) {
        (void)primeListenDataExchange();
      }
      break;

    case RFAL_NFC_STATE_DATAEXCHANGE_DONE:
      handleDataExchangeDone();
      break;

    case RFAL_NFC_STATE_IDLE:
      restartDiscovery();
      break;

    default:
      break;
  }
}

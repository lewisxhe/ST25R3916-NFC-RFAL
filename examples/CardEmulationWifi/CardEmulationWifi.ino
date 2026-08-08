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
#ifndef CE_WIFI_EXAMPLE_DEBUG
#define CE_WIFI_EXAMPLE_DEBUG 1
#endif

#if CE_WIFI_EXAMPLE_DEBUG
#define CE_LOG(msg) do { Serial.print("[CE WIFI] "); Serial.println(msg); } while (0)
#define CE_LOG_VAL(msg, val) do { Serial.print("[CE WIFI] "); Serial.print(msg); Serial.println((int32_t)(val)); } while (0)
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
#if defined(CE_WIFI_NFCA_ONLY)
#define CE_EXAMPLE_TECHS RFAL_NFC_LISTEN_TECH_A
#elif defined(CE_WIFI_MIXED_TECHS)
#define CE_EXAMPLE_TECHS (RFAL_NFC_LISTEN_TECH_A | RFAL_NFC_LISTEN_TECH_F)
#else
#define CE_EXAMPLE_TECHS RFAL_NFC_LISTEN_TECH_F
#endif
#endif

#ifndef CE_WIFI_SSID
#define CE_WIFI_SSID "xinyuandianzi"
#endif

#ifndef CE_WIFI_PASSWORD
#define CE_WIFI_PASSWORD "AA15994823428"
#endif

#ifndef CE_WIFI_AUTH_TYPE
#define CE_WIFI_AUTH_TYPE 0x0020U
#endif

#ifndef CE_WIFI_ENC_TYPE
#define CE_WIFI_ENC_TYPE 0x0010U
#endif

#define CE_WIFI_NDEF_MAX_LEN 256U

static RfalRfST25R3916Class rfalRf(ST25R3916_RFAL_HOST_INTERFACE);
static RfalNfcClass nfc(&rfalRf);
static rfalNfcDiscoverParam discParam;

static uint8_t *rxData = NULL;
static uint16_t *rxLen = NULL;
static uint8_t txBuf[RFAL_NFC_RF_BUF_LEN];
static bool exchangePrimed = false;
static uint8_t wifiNdefFile[2U + CE_WIFI_NDEF_MAX_LEN];
static uint32_t wifiNdefFileLen = 0U;

static uint8_t ceNfcfNfcid2[RFAL_NFCF_NFCID2_LEN] = {
  0x02, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

static void printHex(const uint8_t *data, uint16_t len)
{
#if CE_WIFI_EXAMPLE_DEBUG
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

static bool appendBytes(uint8_t *buf, uint32_t bufLen, uint32_t *pos, const uint8_t *data, uint32_t len)
{
  if ((buf == NULL) || (pos == NULL) || ((len > 0U) && (data == NULL)) || ((*pos + len) > bufLen)) {
    return false;
  }

  if (len > 0U) {
    memcpy(&buf[*pos], data, len);
    *pos += len;
  }
  return true;
}

static bool appendByte(uint8_t *buf, uint32_t bufLen, uint32_t *pos, uint8_t value)
{
  return appendBytes(buf, bufLen, pos, &value, 1U);
}

static bool appendU16BE(uint8_t *buf, uint32_t bufLen, uint32_t *pos, uint16_t value)
{
  uint8_t tmp[2] = {
    (uint8_t)(value >> 8U),
    (uint8_t)(value & 0xFFU)
  };
  return appendBytes(buf, bufLen, pos, tmp, sizeof(tmp));
}

static bool appendWpsAttributeHeader(uint8_t *buf, uint32_t bufLen, uint32_t *pos, uint16_t tag, uint16_t len)
{
  return appendU16BE(buf, bufLen, pos, tag) && appendU16BE(buf, bufLen, pos, len);
}

static bool appendWpsAttributeBytes(uint8_t *buf, uint32_t bufLen, uint32_t *pos, uint16_t tag, const uint8_t *data, uint16_t len)
{
  return appendWpsAttributeHeader(buf, bufLen, pos, tag, len) && appendBytes(buf, bufLen, pos, data, len);
}

static bool appendWpsAttributeU8(uint8_t *buf, uint32_t bufLen, uint32_t *pos, uint16_t tag, uint8_t value)
{
  return appendWpsAttributeHeader(buf, bufLen, pos, tag, 1U) && appendByte(buf, bufLen, pos, value);
}

static bool appendWpsAttributeU16(uint8_t *buf, uint32_t bufLen, uint32_t *pos, uint16_t tag, uint16_t value)
{
  return appendWpsAttributeHeader(buf, bufLen, pos, tag, 2U) && appendU16BE(buf, bufLen, pos, value);
}

static bool buildWifiOobPayload(uint8_t *buf, uint32_t bufLen, uint32_t *payloadLen)
{
  static const uint8_t macSkip[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  const char ssid[] = CE_WIFI_SSID;
  const char password[] = CE_WIFI_PASSWORD;
  const uint16_t ssidLen = (uint16_t)(sizeof(ssid) - 1U);
  const uint16_t passwordLen = (uint16_t)(sizeof(password) - 1U);
  const uint16_t credentialLen = (uint16_t)(
    5U +
    4U + ssidLen +
    6U +
    6U +
    4U + passwordLen +
    10U
  );
  uint32_t pos = 0U;

  if ((buf == NULL) || (payloadLen == NULL)) {
    return false;
  }

  if (!appendWpsAttributeU8(buf, bufLen, &pos, 0x104AU, 0x10U)) {
    return false;
  }

  if (!appendWpsAttributeHeader(buf, bufLen, &pos, 0x100EU, credentialLen)) {
    return false;
  }

  if (!appendWpsAttributeU8(buf, bufLen, &pos, 0x1026U, 0x01U)) {
    return false;
  }

  if (!appendWpsAttributeBytes(buf, bufLen, &pos, 0x1045U, (const uint8_t *)ssid, ssidLen)) {
    return false;
  }

  if (!appendWpsAttributeU16(buf, bufLen, &pos, 0x1003U, (uint16_t)CE_WIFI_AUTH_TYPE)) {
    return false;
  }

  if (!appendWpsAttributeU16(buf, bufLen, &pos, 0x100FU, (uint16_t)CE_WIFI_ENC_TYPE)) {
    return false;
  }

  if (!appendWpsAttributeBytes(buf, bufLen, &pos, 0x1027U, (const uint8_t *)password, passwordLen)) {
    return false;
  }

  if (!appendWpsAttributeBytes(buf, bufLen, &pos, 0x1020U, macSkip, sizeof(macSkip))) {
    return false;
  }

  *payloadLen = pos;
  return true;
}

static bool buildWifiNdefFile()
{
  static const uint8_t mimeType[] = "application/vnd.wfa.wsc";
  uint8_t payload[128];
  uint32_t payloadLen = 0U;
  uint32_t pos = 2U;
  uint32_t messageLen;

  if (!buildWifiOobPayload(payload, sizeof(payload), &payloadLen)) {
    CE_LOG("WiFi OOB payload build failed");
    return false;
  }

  messageLen = 3U + (sizeof(mimeType) - 1U) + payloadLen;
  if ((messageLen > CE_WIFI_NDEF_MAX_LEN) || (messageLen > 255U)) {
    CE_LOG_VAL("WiFi NDEF too large len=", messageLen);
    return false;
  }

  wifiNdefFile[0] = (uint8_t)(messageLen >> 8U);
  wifiNdefFile[1] = (uint8_t)(messageLen & 0xFFU);

  if (!appendByte(wifiNdefFile, sizeof(wifiNdefFile), &pos, 0xD2U) ||
      !appendByte(wifiNdefFile, sizeof(wifiNdefFile), &pos, (uint8_t)(sizeof(mimeType) - 1U)) ||
      !appendByte(wifiNdefFile, sizeof(wifiNdefFile), &pos, (uint8_t)payloadLen) ||
      !appendBytes(wifiNdefFile, sizeof(wifiNdefFile), &pos, mimeType, sizeof(mimeType) - 1U) ||
      !appendBytes(wifiNdefFile, sizeof(wifiNdefFile), &pos, payload, payloadLen)) {
    CE_LOG("WiFi NDEF encode failed");
    return false;
  }

  wifiNdefFileLen = pos;

#if CE_WIFI_EXAMPLE_DEBUG
  Serial.print("[CE WIFI] SSID=");
  Serial.println(CE_WIFI_SSID);
  Serial.print("[CE WIFI] NDEF message len=");
  Serial.println(messageLen);
  Serial.print("[CE WIFI] WiFi payload len=");
  Serial.println(payloadLen);
#endif

  return true;
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
#if CE_WIFI_EXAMPLE_DEBUG
  Serial.print("[CE WIFI] NFC state=");
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

static bool waitForNextReaderFrame()
{
  ReturnCode err = nfc.rfalNfcDataExchangeStart(NULL, 0U, &rxData, &rxLen, RFAL_FWT_NONE);
  if (err != ST_ERR_NONE) {
    CE_LOG_VAL("wait for next frame failed err=", err);
    nfc.rfalNfcDeactivate(true);
    exchangePrimed = false;
    return false;
  }

  exchangePrimed = true;
  CE_LOG("Waiting for next reader frame");
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
    if ((dev->type == RFAL_NFC_POLL_TYPE_NFCF) && (dev->rfInterface == RFAL_NFC_INTERFACE_RF)) {
      (void)waitForNextReaderFrame();
      return;
    }

    nfc.rfalNfcDeactivate(true);
    exchangePrimed = false;
    return;
  }

#if CE_WIFI_EXAMPLE_DEBUG
  Serial.print("[CE WIFI] TX ");
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

  CE_LOG("ST25R3916 WiFi card emulation example");
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
  if (!buildWifiNdefFile()) {
    CE_LOG("WiFi NDEF init failed");
    while (true) {
      delay(1000);
    }
  }
  demoCeSetNdefFile(wifiNdefFile, wifiNdefFileLen);
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

#include <Arduino.h>
#include <SPI.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include <rfal_rfst25r3916.h>
#include <st25r3916_arduino_bus.h>
#include <rfal_nfc.h>
#include <demo_ce.h>

/*
 * Example:
 *   - Measures the ST25R3916 capacitive sensor from loop()
 *   - Switches between polling(reader) and card-emulation modes by Serial
 *
 * Library-level CE logs can be enabled with:
 *   -DRFAL_CE_DEBUG
 */
#ifndef CAP_SWITCH_EXAMPLE_DEBUG
#define CAP_SWITCH_EXAMPLE_DEBUG 1
#endif

#if CAP_SWITCH_EXAMPLE_DEBUG
#define CAP_LOG(msg) do { Serial.print("[CAP SWITCH] "); Serial.println(msg); } while (0)
#define CAP_LOG_VAL(msg, val) do { Serial.print("[CAP SWITCH] "); Serial.print(msg); Serial.println((int32_t)(val)); } while (0)
#else
#define CAP_LOG(msg) do { } while (0)
#define CAP_LOG_VAL(msg, val) do { } while (0)
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

#ifndef CAP_SWITCH_SAMPLE_MS
#define CAP_SWITCH_SAMPLE_MS 250UL
#endif

#ifndef CAP_SWITCH_REPORT_MS
#define CAP_SWITCH_REPORT_MS 1000UL
#endif

#ifndef CAP_SWITCH_THRESHOLD
#define CAP_SWITCH_THRESHOLD 4U
#endif

#ifndef CAP_SWITCH_CE_TECHS
#if defined(CAP_SWITCH_CE_MIXED_TECHS)
#define CAP_SWITCH_CE_TECHS (RFAL_NFC_LISTEN_TECH_A | RFAL_NFC_LISTEN_TECH_F)
#else
#define CAP_SWITCH_CE_TECHS RFAL_NFC_LISTEN_TECH_F
#endif
#endif

enum AppMode {
  APP_MODE_IDLE,
  APP_MODE_POLL,
  APP_MODE_CE
};

static RfalRfST25R3916Class rfalRf(ST25R3916_RFAL_HOST_INTERFACE);
static RfalNfcClass nfc(&rfalRf);
static rfalNfcDiscoverParam discParam;

static AppMode appMode = APP_MODE_IDLE;
static bool activeReported = false;
static bool exchangePrimed = false;

static uint8_t *rxData = NULL;
static uint16_t *rxLen = NULL;
static uint8_t txBuf[RFAL_NFC_RF_BUF_LEN];

static uint8_t capBaseline = 0U;
static uint8_t capLast = 0U;
static uint8_t capThreshold = CAP_SWITCH_THRESHOLD;
static bool capAvailable = false;
static bool capPresent = false;
static bool capLastPresent = false;
static unsigned long lastCapSampleMs = 0UL;
static unsigned long lastCapReportMs = 0UL;

static char cmdBuf[48];
static uint8_t cmdLen = 0U;

static uint8_t ceNfcfNfcid2[RFAL_NFCF_NFCID2_LEN] = {
  0x02, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

static const char *modeName(AppMode mode)
{
  switch (mode) {
    case APP_MODE_POLL:
      return "POLL";
    case APP_MODE_CE:
      return "CARD_EMULATION";
    case APP_MODE_IDLE:
    default:
      return "IDLE";
  }
}

static const char *stateName(rfalNfcState state)
{
  switch (state) {
    case RFAL_NFC_STATE_IDLE:
      return "IDLE";
    case RFAL_NFC_STATE_START_DISCOVERY:
      return "START_DISCOVERY";
    case RFAL_NFC_STATE_WAKEUP_MODE:
      return "WAKEUP_MODE";
    case RFAL_NFC_STATE_POLL_TECHDETECT:
      return "POLL_TECHDETECT";
    case RFAL_NFC_STATE_POLL_COLAVOIDANCE:
      return "POLL_COLAVOIDANCE";
    case RFAL_NFC_STATE_POLL_SELECT:
      return "POLL_SELECT";
    case RFAL_NFC_STATE_POLL_ACTIVATION:
      return "POLL_ACTIVATION";
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
    case RFAL_NFC_STATE_DEACTIVATION:
      return "DEACTIVATION";
    default:
      return "OTHER";
  }
}

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

static void printHex(const uint8_t *data, uint16_t len)
{
#if CAP_SWITCH_EXAMPLE_DEBUG
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

static uint8_t capDelta(uint8_t value)
{
  return (value >= capBaseline) ? (value - capBaseline) : (capBaseline - value);
}

static ReturnCode measureCapacitance(uint8_t *value)
{
  if (value == NULL) {
    return ST_ERR_PARAM;
  }

  ReturnCode err = rfalRf.st25r3916MeasureCapacitance(value);
  if (err == ST_ERR_NOTSUPP) {
    capAvailable = false;
  }
  return err;
}

static bool canUseCapSensor()
{
  return ((appMode == APP_MODE_IDLE) && (nfc.rfalNfcGetState() == RFAL_NFC_STATE_IDLE));
}

static bool calibrateCapSensor()
{
  if (!canUseCapSensor()) {
    CAP_LOG("send idle before capacitive calibration");
    return false;
  }

  uint8_t calibration = 0U;
  ReturnCode err = rfalRf.st25r3916CalibrateCapacitiveSensor(&calibration);
  if (err == ST_ERR_NOTSUPP) {
    capAvailable = false;
    CAP_LOG("Capacitive sensor not supported on ST25R3916B");
    return false;
  }

  if (err != ST_ERR_NONE) {
    CAP_LOG_VAL("capacitive calibration failed err=", err);
    return false;
  }

  uint16_t sum = 0U;
  uint8_t samples = 0U;
  for (uint8_t i = 0; i < 8U; i++) {
    uint8_t value = 0U;
    err = measureCapacitance(&value);
    if (err == ST_ERR_NONE) {
      sum += value;
      samples++;
    }
    delay(4);
  }

  if (samples == 0U) {
    CAP_LOG("capacitive baseline failed");
    return false;
  }

  capBaseline = (uint8_t)(sum / samples);
  capLast = capBaseline;
  capPresent = false;
  capLastPresent = false;
  capAvailable = true;

#if CAP_SWITCH_EXAMPLE_DEBUG
  Serial.print("[CAP SWITCH] cap calibration=");
  Serial.print(calibration);
  Serial.print(" baseline=");
  Serial.println(capBaseline);
#endif
  return true;
}

static void reportCap(bool force)
{
  if (!capAvailable) {
    if (force) {
      CAP_LOG("capacitive sensor unavailable");
    }
    return;
  }

  const unsigned long now = millis();
  const uint8_t delta = capDelta(capLast);
  const bool changed = (capPresent != capLastPresent);
  if (!force && !changed && ((now - lastCapReportMs) < CAP_SWITCH_REPORT_MS)) {
    return;
  }

  lastCapReportMs = now;
  capLastPresent = capPresent;

#if CAP_SWITCH_EXAMPLE_DEBUG
  Serial.print("[CAP SWITCH] cap raw=");
  Serial.print(capLast);
  Serial.print(" base=");
  Serial.print(capBaseline);
  Serial.print(" delta=");
  Serial.print(delta);
  Serial.print(" threshold=");
  Serial.print(capThreshold);
  Serial.print(" prox=");
  Serial.println(capPresent ? "YES" : "NO");
#else
  (void)force;
#endif
}

static void sampleCapSensor()
{
  if (!capAvailable || !canUseCapSensor()) {
    return;
  }

  const unsigned long now = millis();
  if ((now - lastCapSampleMs) < CAP_SWITCH_SAMPLE_MS) {
    return;
  }
  lastCapSampleMs = now;

  uint8_t value = 0U;
  ReturnCode err = measureCapacitance(&value);
  if (err != ST_ERR_NONE) {
    CAP_LOG_VAL("measure capacitance failed err=", err);
    return;
  }

  capLast = value;
  capPresent = (capDelta(value) >= capThreshold);
  reportCap(false);
}

static void onNfcState(rfalNfcState state)
{
#if CAP_SWITCH_EXAMPLE_DEBUG
  Serial.print("[CAP SWITCH] NFC state=");
  Serial.print(stateName(state));
  Serial.print(" (");
  Serial.print((int)state);
  Serial.println(')');
#else
  (void)state;
#endif

  if (state == RFAL_NFC_STATE_POLL_SELECT) {
    rfalNfcDevice *devList = NULL;
    uint8_t devCnt = 0U;
    if (nfc.rfalNfcGetDevicesFound(&devList, &devCnt) == ST_ERR_NONE) {
      CAP_LOG_VAL("Multiple devices found count=", devCnt);
      (void)nfc.rfalNfcSelect(0U);
    }
  } else if (state == RFAL_NFC_STATE_START_DISCOVERY) {
    activeReported = false;
    exchangePrimed = false;
  }
}

static void configurePollingDiscovery()
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

static void configureCeDiscovery()
{
  static const uint8_t nfcid1[RFAL_NFCID1_TRIPLE_LEN] = {
    0x5F, 'S', 'T', 'M', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  static const uint8_t pmm[8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x00
  };

  memset(&discParam, 0, sizeof(discParam));
  discParam.compMode = RFAL_COMPLIANCE_MODE_NFC;
  discParam.techs2Find = CAP_SWITCH_CE_TECHS;
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

static void stopRfActivity()
{
  if (nfc.rfalNfcGetState() > RFAL_NFC_STATE_IDLE) {
    (void)nfc.rfalNfcDeactivate(false);
  }
  (void)rfalRf.rfalFieldOff();

  rxData = NULL;
  rxLen = NULL;
  activeReported = false;
  exchangePrimed = false;
  digitalWrite(MCU_LED1, LOW);
  digitalWrite(MCU_LED6, LOW);
}

static void startDiscoveryForMode(AppMode mode)
{
  if (mode == APP_MODE_POLL) {
    configurePollingDiscovery();
  } else if (mode == APP_MODE_CE) {
    configureCeDiscovery();
  } else {
    return;
  }

  rxData = NULL;
  rxLen = NULL;
  activeReported = false;
  exchangePrimed = false;

  ReturnCode err = nfc.rfalNfcDiscover(&discParam);
  if (err != ST_ERR_NONE) {
    CAP_LOG_VAL("rfalNfcDiscover failed err=", err);
    appMode = APP_MODE_IDLE;
    return;
  }

#if CAP_SWITCH_EXAMPLE_DEBUG
  Serial.print("[CAP SWITCH] mode=");
  Serial.print(modeName(mode));
  if (mode == APP_MODE_CE) {
    Serial.print(" listenTechs=0x");
    Serial.print((uint16_t)CAP_SWITCH_CE_TECHS, HEX);
  }
  Serial.println();
#endif

  digitalWrite(MCU_LED1, (mode == APP_MODE_POLL) ? HIGH : LOW);
  digitalWrite(MCU_LED6, (mode == APP_MODE_CE) ? HIGH : LOW);
}

static void setMode(AppMode mode)
{
  if (mode == appMode) {
    CAP_LOG_VAL("already in mode=", mode);
    return;
  }

  stopRfActivity();
  appMode = mode;

  if (appMode == APP_MODE_IDLE) {
    CAP_LOG("mode=IDLE");
    return;
  }

  startDiscoveryForMode(appMode);
}

static void reportActivePollingDevice()
{
  rfalNfcDevice *dev = NULL;
  ReturnCode err = nfc.rfalNfcGetActiveDevice(&dev);
  if ((err != ST_ERR_NONE) || (dev == NULL)) {
    CAP_LOG_VAL("rfalNfcGetActiveDevice failed err=", err);
    nfc.rfalNfcDeactivate(true);
    return;
  }

#if CAP_SWITCH_EXAMPLE_DEBUG
  Serial.print("[CAP SWITCH] card Type=");
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

static bool primeListenDataExchange()
{
  ReturnCode err = nfc.rfalNfcDataExchangeStart(NULL, 0U, &rxData, &rxLen, RFAL_FWT_NONE);
  if (err != ST_ERR_NONE) {
    CAP_LOG_VAL("prime exchange failed err=", err);
    nfc.rfalNfcDeactivate(true);
    return false;
  }

  exchangePrimed = true;
  (void)nfc.rfalNfcDataExchangeGetStatus();
  CAP_LOG("Data exchange primed");
  return true;
}

static void handleCeDataExchangeDone()
{
  ReturnCode err = nfc.rfalNfcDataExchangeGetStatus();
  if (err == ST_ERR_BUSY) {
    return;
  }

  if (err == ST_ERR_SLEEP_REQ) {
    CAP_LOG("Reader requested sleep");
    exchangePrimed = false;
    return;
  }

  if (err != ST_ERR_NONE) {
    CAP_LOG_VAL("data exchange failed err=", err);
    nfc.rfalNfcDeactivate(true);
    exchangePrimed = false;
    return;
  }

  rfalNfcDevice *dev = NULL;
  if ((nfc.rfalNfcGetActiveDevice(&dev) != ST_ERR_NONE) || (dev == NULL) || (rxData == NULL) || (rxLen == NULL)) {
    CAP_LOG("No active device context");
    nfc.rfalNfcDeactivate(true);
    exchangePrimed = false;
    return;
  }

  uint16_t inLen = *rxLen;
  uint16_t outLen = 0U;

  if ((dev->type == RFAL_NFC_POLL_TYPE_NFCA) && (dev->rfInterface == RFAL_NFC_INTERFACE_ISODEP)) {
    CAP_LOG_VAL("T4T APDU len=", inLen);
    outLen = demoCeT4T(rxData, inLen, txBuf, sizeof(txBuf));
  } else if ((dev->type == RFAL_NFC_POLL_TYPE_NFCF) && (dev->rfInterface == RFAL_NFC_INTERFACE_RF)) {
    inLen = rfalConvBitsToBytes(inLen);
    CAP_LOG_VAL("T3T command len=", inLen);
    outLen = demoCeT3T(rxData, inLen, txBuf, sizeof(txBuf));
  } else {
    CAP_LOG_VAL("Unsupported activated type=", dev->type);
  }

  if (outLen == 0U) {
    CAP_LOG("No CE response generated");
    nfc.rfalNfcDeactivate(true);
    exchangePrimed = false;
    return;
  }

#if CAP_SWITCH_EXAMPLE_DEBUG
  Serial.print("[CAP SWITCH] TX ");
  printHex(txBuf, outLen);
  Serial.println();
#endif

  err = nfc.rfalNfcDataExchangeStart(txBuf, outLen, &rxData, &rxLen, RFAL_FWT_NONE);
  if (err != ST_ERR_NONE) {
    CAP_LOG_VAL("response exchange failed err=", err);
    nfc.rfalNfcDeactivate(true);
    exchangePrimed = false;
  }
}

static void handleNfcState()
{
  if (appMode == APP_MODE_IDLE) {
    return;
  }

  nfc.rfalNfcWorker();

  switch (nfc.rfalNfcGetState()) {
    case RFAL_NFC_STATE_ACTIVATED:
      if (appMode == APP_MODE_POLL) {
        if (!activeReported) {
          reportActivePollingDevice();
        }
      } else if ((appMode == APP_MODE_CE) && !exchangePrimed) {
        (void)primeListenDataExchange();
      }
      break;

    case RFAL_NFC_STATE_DATAEXCHANGE_DONE:
      if (appMode == APP_MODE_CE) {
        handleCeDataExchangeDone();
      }
      break;

    case RFAL_NFC_STATE_IDLE:
      startDiscoveryForMode(appMode);
      break;

    default:
      break;
  }
}

static void printHelp()
{
  CAP_LOG("commands:");
  CAP_LOG("  poll | reader | r  - reader polling mode");
  CAP_LOG("  ce | card | c      - NFC-F card-emulation mode");
  CAP_LOG("  idle | off | i     - stop RF activity");
  CAP_LOG("  cap                - print one capacitance sample");
  CAP_LOG("  cal                - recalibrate capacitive baseline");
  CAP_LOG("  th <n>             - set proximity threshold");
  CAP_LOG("  help | ?           - print this help");
}

static char *trimCommand(char *line)
{
  while ((*line != '\0') && isspace((unsigned char)*line)) {
    line++;
  }

  char *end = line + strlen(line);
  while ((end > line) && isspace((unsigned char)*(end - 1))) {
    end--;
  }
  *end = '\0';

  for (char *p = line; *p != '\0'; p++) {
    *p = (char)tolower((unsigned char)*p);
  }
  return line;
}

static void processCommand(char *line)
{
  char *cmd = trimCommand(line);
  if (*cmd == '\0') {
    return;
  }

  if ((strcmp(cmd, "help") == 0) || (strcmp(cmd, "?") == 0)) {
    printHelp();
  } else if ((strcmp(cmd, "poll") == 0) || (strcmp(cmd, "reader") == 0) || (strcmp(cmd, "r") == 0)) {
    setMode(APP_MODE_POLL);
  } else if ((strcmp(cmd, "ce") == 0) || (strcmp(cmd, "card") == 0) || (strcmp(cmd, "c") == 0)) {
    setMode(APP_MODE_CE);
  } else if ((strcmp(cmd, "idle") == 0) || (strcmp(cmd, "off") == 0) || (strcmp(cmd, "i") == 0)) {
    setMode(APP_MODE_IDLE);
  } else if (strcmp(cmd, "cap") == 0) {
    if (!canUseCapSensor()) {
      CAP_LOG("send idle before capacitance sample");
      return;
    }
    uint8_t value = 0U;
    ReturnCode err = measureCapacitance(&value);
    if (err == ST_ERR_NONE) {
      capLast = value;
      capPresent = (capDelta(value) >= capThreshold);
      reportCap(true);
    } else {
      CAP_LOG_VAL("measure capacitance failed err=", err);
    }
  } else if (strcmp(cmd, "cal") == 0) {
    (void)calibrateCapSensor();
  } else if ((strncmp(cmd, "th ", 3) == 0) || (strncmp(cmd, "threshold ", 10) == 0)) {
    const char *num = (cmd[1] == 'h') ? (cmd + 3) : (cmd + 10);
    long value = strtol(num, NULL, 10);
    if ((value < 1L) || (value > 127L)) {
      CAP_LOG("threshold must be 1..127");
    } else {
      capThreshold = (uint8_t)value;
      CAP_LOG_VAL("threshold=", capThreshold);
      reportCap(true);
    }
  } else {
    CAP_LOG("unknown command");
    printHelp();
  }
}

static void handleSerial()
{
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      processCommand(cmdBuf);
      cmdLen = 0U;
      continue;
    }

    if (cmdLen < (sizeof(cmdBuf) - 1U)) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdLen = 0U;
      CAP_LOG("command too long");
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  pinMode(MCU_LED1, OUTPUT);
  pinMode(MCU_LED6, OUTPUT);
  digitalWrite(MCU_LED1, LOW);
  digitalWrite(MCU_LED6, LOW);

  st25r3916BeginArduinoBus();

  CAP_LOG("ST25R3916 capacitive sensor mode-switch example");
  CAP_LOG(st25r3916ArduinoBusName());
  CAP_LOG("Use Serial commands to switch reader/card-emulation modes");

  ReturnCode err = nfc.rfalNfcInitialize();
  if (err != ST_ERR_NONE) {
    CAP_LOG_VAL("rfalNfcInitialize failed err=", err);
    while (true) {
      delay(1000);
    }
  }

  demoCeInit(ceNfcfNfcid2);
  (void)calibrateCapSensor();
  printHelp();
  CAP_LOG("mode=IDLE");
}

void loop()
{
  handleSerial();
  sampleCapSensor();
  handleNfcState();
}

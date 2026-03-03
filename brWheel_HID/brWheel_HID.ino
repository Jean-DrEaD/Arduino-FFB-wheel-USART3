/* Arduino Leonardo/Pro Micro Force Feedback Wheel firmware (adaptado)
   Objetivo: PC/HID -> Pro Micro -> Serial1 -> STM32 (hoverboard-firmware-hack-FOC)
             STM32 -> Serial1 -> Pro Micro (feedback encoder)

   IMPORTANTE:
   - Este sketch NÃO usa SetPWM() (PWM externo).
   - Ele envia torque via Serial1 no formato SerialCommand do STM32:
       start=0xABCD, steer(int16), speed(int16), checksum = start ^ steer ^ speed
   - Ele recebe feedback do STM32 no formato SerialFeedback (ver Src/main.c no STM32):
       start=0xABCD, encPos(int32), cmd1(int16), cmd2(int16), checksum
*/

// ========================= CONFIG RÁPIDA =========================
#define USE_FIXED_TORQUE_TEST   0   // 1 = ignora FFB e manda torque fixo
#define FIXED_TORQUE_VALUE      50  // torque (em -1000..1000), use baixo

#define PRINT_DEBUG_EVERY_MS  1000  // debug no CONFIG_SERIAL
#define ENC_LINK_TIMEOUT_MS    100  // se não receber feedback em 100ms, considera link caiu
// ================================================================

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include "Config.h"
#include "ConfigHID.h"
#include "debug.h"
#include "ffb_pro.h"
#include "USBDesc.h"

#ifdef USE_QUADRATURE_ENCODER
#include "QuadEncoder.h"
#endif
#ifdef USE_VNH5019
#include "DualVNH5019MotorShield.h"
#endif
#include <Wire.h>
#ifdef USE_EEPROM
#include <EEPROM.h>
#endif
#ifdef USE_LOAD_CELL
#include <HX711_ADC.h>
#endif
#ifdef USE_LCD
#include <LiquidCrystal_I2C.h>
#endif
#ifdef USE_ADS1015
#include <Adafruit_ADS1015.h>
#endif
#ifdef USE_MCP4725
#include <Adafruit_MCP4725.h>
#endif
#ifdef USE_AS5600
#include "AS5600.h"
#endif

// ===================== STM32 feedback serial frame (USART3 -> Serial1) =====================
// Confirmado pelo seu grep em Src/main.c:
//   typedef struct {
//     uint16_t start;
//     int32_t  encPos;
//     int16_t  cmd1;
//     int16_t  cmd2;
//     uint16_t checksum;
//   } SerialFeedback;
typedef struct __attribute__((packed)) {
  uint16_t start;   // 0xABCD
  int32_t  encPos;
  int16_t  cmd1;
  int16_t  cmd2;
  uint16_t checksum;
} StmEncFrame;

static volatile bool   gHaveEnc = false;
static int32_t         gLastEncPos = 0;
static int16_t         gLastCmd1 = 0;
static int16_t         gLastCmd2 = 0;
static uint32_t        gLastEncRxMs = 0;

static uint16_t stmFbChecksum(const StmEncFrame &f) {
  uint16_t cs = 0;
  cs ^= f.start;
  cs ^= (uint16_t)(f.encPos);
  cs ^= (uint16_t)((uint32_t)f.encPos >> 16);
  cs ^= (uint16_t)f.cmd1;
  cs ^= (uint16_t)f.cmd2;
  return cs;
}

static void stmEncPoll() {
  // Parser por sincronização no header 0xCD 0xAB (little-endian de 0xABCD)
  static uint8_t buf[sizeof(StmEncFrame)];
  static uint8_t idx = 0;

  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();

    if (idx == 0) { if (b != 0xCD) continue; buf[idx++] = b; continue; }
    if (idx == 1) { if (b != 0xAB) { idx = 0; continue; } buf[idx++] = b; continue; }

    buf[idx++] = b;

    if (idx >= sizeof(StmEncFrame)) {
      idx = 0;

      StmEncFrame f;
      memcpy(&f, buf, sizeof(f));

      if (f.start != 0xABCD) continue;
      if (f.checksum != stmFbChecksum(f)) continue;

      gLastEncPos = f.encPos;
      gLastCmd1   = f.cmd1;
      gLastCmd2   = f.cmd2;
      gHaveEnc    = true;
      gLastEncRxMs = millis();
    }
  }

  // timeout de link
  if (gHaveEnc) {
    uint32_t now = millis();
    if ((now - gLastEncRxMs) > ENC_LINK_TIMEOUT_MS) {
      gHaveEnc = false;
    }
  }
}
// ==========================================================================================


// ===================== STM32 control serial frame (Pro Micro -> STM32) ====================
// Confirmado pelo util.h do STM32 (sem IBUS):
//   typedef struct { uint16_t start; int16_t steer; int16_t speed; uint16_t checksum; } SerialCommand;
typedef struct __attribute__((packed)) {
  uint16_t start;     // 0xABCD
  int16_t  steer;     // -1000..1000
  int16_t  speed;     // -1000..1000 (usado como torque no seu TRQ_MODE)
  uint16_t checksum;  // XOR
} StmCmdFrame;

static uint16_t stmCmdChecksum(const StmCmdFrame &c) {
  return (uint16_t)(c.start ^ (uint16_t)c.steer ^ (uint16_t)c.speed);
}

static void stmSendCmd(int16_t steer, int16_t speed) {
  StmCmdFrame c;
  c.start = 0xABCD;
  c.steer = steer;
  c.speed = speed;
  c.checksum = stmCmdChecksum(c);
  Serial1.write((const uint8_t *)&c, sizeof(c));
}
// ==========================================================================================


// ===================== Recomendações/segurança (FFB DD) =====================
static const int16_t TORQUE_DEADBAND = 5;     // unidades no range -1000..1000
static const int16_t TORQUE_MAX      = 250;   // comece baixo; aumente depois (ex 500, 800)
static const int16_t TORQUE_SLEW     = 20;    // máx mudança por ciclo (anti-tranco)

static int16_t gTorqueOut = 0;

static int16_t applyDeadband(int16_t x, int16_t db) {
  return (x > -db && x < db) ? 0 : x;
}

static int16_t slewLimit(int16_t target, int16_t current, int16_t step) {
  int16_t delta = (int16_t)(target - current);
  if (delta > step)  return (int16_t)(current + step);
  if (delta < -step) return (int16_t)(current - step);
  return target;
}
// ============================================================================


// Globals do projeto original
fwOpt fwOptions;
s16a accel, clutch, hbrake;
#ifdef USE_SPLITAXIS
s16 combinedAxis;
s16 gasAxis;
#endif
#ifdef USE_XY_SHIFTER
xysh shifter;
#endif
s32a brake;

s32v turn;
s32v axis;
s32v ffbs;
u32 button = 0;

#ifdef USE_ADS1015
Adafruit_ADS1015 ads(0x48);
#endif
#ifdef USE_MCP4725
Adafruit_MCP4725 dac0;
Adafruit_MCP4725 dac1;
#endif

cFFB gFFB;
BRFFB brWheelFFB;

#ifdef AVG_INPUTS
extern s32 analog_inputs[];
u8 asc = 0;
#endif

u32 last_ConfigSerial = 0;
u32 last_refresh = 0;
u32 now_micros = 0;
u32 timeDiffConfigSerial = 0;

uint16_t dz, bdz;
uint8_t last_LC_scaling;

#ifdef USE_LOAD_CELL
HX711_ADC LoadCell(4, 5);
#endif

#ifdef USE_QUADRATURE_ENCODER
cQuadEncoder myEnc;
#endif

#ifdef USE_LCD
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif

#ifdef USE_VNH5019
DualVNH5019MotorShield ms;
void stopIfFault() {
  if (ms.getM1Fault()) { DEBUG_SERIAL.println("M1 fault"); while (1) {} }
  if (ms.getM2Fault()) { DEBUG_SERIAL.println("M2 fault"); while (1) {} }
}
#endif

#ifdef USE_AS5600
AS5600L as5600x(0x36);
#endif

void setup() {
  CONFIG_SERIAL.begin(115200);

  // Serial1: link com STM32 (RX1/TX1 do Pro Micro)
  Serial1.begin(500000);
  while (Serial1.available()) { Serial1.read(); }

  accel.val = 0;
  brake.val = 0;
  clutch.val = 0;

  axis.x = 0;
  turn.x = 0;

#ifdef USE_EEPROM
  SetEEPROMConfig();
  LoadEEPROMConfig();
#else
  ROTATION_DEG = 1080;
  CPR = 4096;

  configGeneralGain  = 100;
  configDamperGain   = 50;
  configFrictionGain = 50;
  configConstantGain = 100;
  configPeriodicGain = 100;
  configSpringGain   = 50;
  configInertiaGain  = 50;
  configCenterGain   = 70;
  configStopGain     = 100;

  effstate = 0b00000001;
  LC_scaling = 128;
  pwmstate = 0b00001100;

  MM_MIN_MOTOR_TORQUE = 0;
  minTorquePP = 0.0;

  accel.min = 0;   accel.max = maxCal;
  brake.min = 0;   brake.max = maxCal;
  clutch.min = 0;  clutch.max = maxCal;
  hbrake.min = 0;  hbrake.max = maxCal;
#endif

  if (CPR == 0) CPR = 4096;

  ROTATION_MAX = int32_t(float(CPR) / 360.0f * float(ROTATION_DEG));
  ROTATION_MID = ROTATION_MAX >> 1;

  InitInputs();
  FfbSetDriver(0);

  ffbs.x = 0;
  gTorqueOut = 0;

  dz = 0;
  bdz = 2047;
  last_LC_scaling = LC_scaling;

  last_refresh = micros();
}

void loop() {
  now_micros = micros();
  timeDiffConfigSerial = now_micros - last_ConfigSerial;

  if ((now_micros - last_refresh) >= CONTROL_PERIOD) {
    last_refresh = now_micros;

    // 1) feedback STM32
    stmEncPoll();

  // 2) posição — encPos do STM32 já é cumulativo
    turn.x = gHaveEnc ? gLastEncPos : 0;
    axis.x = turn.x;

    // 3) torque
#if USE_FIXED_TORQUE_TEST
    int32_t torque = FIXED_TORQUE_VALUE;
#else
    ffbs = gFFB.CalcTorqueCommands(&axis);
    int32_t torque = ffbs.x;
#endif

    torque = constrain(torque, -1000, 1000);
    int16_t t16 = (int16_t)torque;

    t16 = applyDeadband(t16, TORQUE_DEADBAND);
    t16 = constrain(t16, (int16_t)-TORQUE_MAX, (int16_t)TORQUE_MAX);
    t16 = slewLimit(t16, gTorqueOut, TORQUE_SLEW);
    gTorqueOut = t16;

    stmSendCmd(0, gTorqueOut);

    // 4) HID report
    turn.x = (int32_t)(turn.x * (f32(X_AXIS_PHYS_MAX) / f32(ROTATION_MAX)));
    turn.x = constrain(turn.x, -MID_REPORT_X - 1, MID_REPORT_X);

    accel.val  = 0;
    clutch.val = 0;
    hbrake.val = 0;
    brake.val  = 0;

    button = readInputButtons();
    SendInputReport(turn.x + MID_REPORT_X + 1, brake.val, accel.val, clutch.val, hbrake.val, button);

#ifdef USE_CONFIGCDC
    if (timeDiffConfigSerial >= CONFIG_SERIAL_PERIOD) {
      configCDC();
      last_ConfigSerial = now_micros;
    }
#endif

    // 5) debug 1Hz
    static uint32_t lastDbg = 0;
    uint32_t nowMs = millis();
    if ((nowMs - lastDbg) >= PRINT_DEBUG_EVERY_MS) {
      lastDbg = nowMs;
      CONFIG_SERIAL.print("haveEnc=");
      CONFIG_SERIAL.print(gHaveEnc ? 1 : 0);
      CONFIG_SERIAL.print(" encPos=");
      CONFIG_SERIAL.print((long)gLastEncPos);
      CONFIG_SERIAL.print(" cmd1=");
      CONFIG_SERIAL.print((int)gLastCmd1);
      CONFIG_SERIAL.print(" cmd2=");
      CONFIG_SERIAL.print((int)gLastCmd2);
      CONFIG_SERIAL.print(" torqueOut=");
      CONFIG_SERIAL.println((int)gTorqueOut);
    }
  }
}

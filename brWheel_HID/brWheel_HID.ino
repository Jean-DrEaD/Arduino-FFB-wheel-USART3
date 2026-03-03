/* Arduino Leonardo/Pro Micro Force Feedback Wheel firmware
   PC/HID -> Pro Micro -> Serial1 -> STM32 (hoverboard-firmware-hack-FOC)
   STM32 -> Serial1 -> Pro Micro (feedback velocidade)

   NOTAS:
   - Torque via Serial1 formato SerialCommand: start=0xABCD, steer, speed, checksum
   - Feedback via Serial1 formato SerialFeedback: 18 bytes (ver StmFrames.h)
   - Motor único DD: steer=0, speed=torque
   - Encoder de posição: MT6701 via ABZ direto no STM32 (não retorna posição no feedback)
     → Posição deve ser lida por encoder ABZ externo no Arduino OU integrada da velocidade
*/

// ========================= CONFIG RÁPIDA =========================
#define USE_FIXED_TORQUE_TEST   1     // 1 = ignora FFB, manda torque fixo para teste
#define FIXED_TORQUE_VALUE      400   // -1000..1000 (comece com 400, suba depois)

#define PRINT_DEBUG_EVERY_MS    500   // debug no Serial (2x por segundo)
#define ENC_LINK_TIMEOUT_MS     200   // timeout do link STM32
// =================================================================

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include "StmFrames.h"
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

// =================================================================
// STM32 Link — Feedback (RX)
// =================================================================
static bool     gHaveEnc     = false;
static int16_t  gSpeedR      = 0;   // velocidade motor (rpm*10 aprox)
static int16_t  gCmd1        = 0;
static int16_t  gCmd2        = 0;
static int16_t  gBatVoltage  = 0;   // batVoltage / 100.0f = Volts
static int16_t  gBoardTemp   = 0;
static uint32_t gLastEncRxMs = 0;

// Posição integrada da velocidade (substituto até ter encoder no Arduino)
static float    gEncPos_f    = 0.0f;

static uint16_t stmFbChecksum(const StmEncFrame &f) {
  return (uint16_t)(
    f.start        ^
    (uint16_t)f.cmd1       ^
    (uint16_t)f.cmd2       ^
    (uint16_t)f.speedR_meas ^
    (uint16_t)f.speedL_meas ^
    (uint16_t)f.batVoltage  ^
    (uint16_t)f.boardTemp   ^
    f.cmdLed
  );
}

static void stmEncPoll() {
  static uint8_t buf[sizeof(StmEncFrame)];
  static uint8_t idx = 0;

  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();

    // Sincronização pelo header 0xCD 0xAB (little-endian de 0xABCD)
    if (idx == 0) { if (b != 0xCD) continue; buf[idx++] = b; continue; }
    if (idx == 1) { if (b != 0xAB) { idx = 0; continue; } buf[idx++] = b; continue; }

    buf[idx++] = b;

    if (idx >= sizeof(StmEncFrame)) {
      idx = 0;
      StmEncFrame f;
      memcpy(&f, buf, sizeof(f));

      if (f.start != 0xABCD) continue;
      if (f.checksum != stmFbChecksum(f)) continue;

      gSpeedR     = f.speedR_meas;
      gCmd1       = f.cmd1;
      gCmd2       = f.cmd2;
      gBatVoltage = f.batVoltage;
      gBoardTemp  = f.boardTemp;
      gHaveEnc    = true;
      gLastEncRxMs = millis();

      // Posicao real do encoder (MT6701 via STM32, campo speedL_meas)
      // Range: -32768..32767 mapeado para -ROTATION_MAX..ROTATION_MAX
      gEncPos_f = (float)f.speedL_meas / 32767.0f * (float)ROTATION_MAX;
      gEncPos_f = constrain(gEncPos_f, (float)-ROTATION_MAX, (float)ROTATION_MAX);
    }
  }

  // Timeout de link
  if (gHaveEnc && (millis() - gLastEncRxMs) > ENC_LINK_TIMEOUT_MS) {
    gHaveEnc  = false;
    gSpeedR   = 0;
  }
}

// =================================================================
// STM32 Link — Command (TX)
// =================================================================
static uint16_t stmCmdChecksum(const StmCmdFrame &c) {
  return (uint16_t)(c.start ^ (uint16_t)c.steer ^ (uint16_t)c.speed);
}

static void stmSendCmd(int16_t steer, int16_t speed) {
  StmCmdFrame c;
  c.start    = 0xABCD;
  c.steer    = steer;
  c.speed    = speed;
  c.checksum = stmCmdChecksum(c);
  Serial1.write((const uint8_t *)&c, sizeof(c));
}

// =================================================================
// Segurança / filtros de torque
// =================================================================
static const int16_t TORQUE_DEADBAND = 5;    // ignora ruído < 5 unidades
static const int16_t TORQUE_MAX      = 600;  // limite absoluto (aumentar gradual: 400→600→800)
static const int16_t TORQUE_SLEW     = 80;   // máx variação por ciclo (500Hz → rampa ~25ms)

static int16_t gTorqueOut = 0;

static int16_t applyDeadband(int16_t x, int16_t db) {
  return (x > -db && x < db) ? 0 : x;
}

static int16_t slewLimit(int16_t target, int16_t current, int16_t step) {
  int16_t delta = target - current;
  if (delta >  step) return current + step;
  if (delta < -step) return current - step;
  return target;
}

// =================================================================
// Globals do projeto original
// =================================================================
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
u32 last_refresh      = 0;
u32 now_micros        = 0;
u32 timeDiffConfigSerial = 0;

uint16_t dz, bdz;
uint8_t  last_LC_scaling;

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

// =================================================================
void setup() {
  CONFIG_SERIAL.begin(115200);

  // Serial1: link STM32 (RX1/TX1 do Pro Micro / Leonardo)
  Serial1.begin(500000);
  while (Serial1.available()) Serial1.read();   // flush

  accel.val = 0;
  brake.val = 0;
  clutch.val = 0;
  axis.x    = 0;
  turn.x    = 0;

#ifdef USE_EEPROM
  SetEEPROMConfig();
  LoadEEPROMConfig();
#else
  ROTATION_DEG = 1080;
  CPR          = 4096;

  configGeneralGain   = 100;
  configDamperGain    = 50;
  configFrictionGain  = 50;
  configConstantGain  = 100;
  configPeriodicGain  = 100;
  configSpringGain    = 50;
  configInertiaGain   = 50;
  configCenterGain    = 70;
  configStopGain      = 100;

  effstate    = 0b00000001;
  LC_scaling  = 128;
  pwmstate    = 0b00001100;

  MM_MIN_MOTOR_TORQUE = 0;
  minTorquePP         = 0.0;

  accel.min  = 0; accel.max  = maxCal;
  brake.min  = 0; brake.max  = maxCal;
  clutch.min = 0; clutch.max = maxCal;
  hbrake.min = 0; hbrake.max = maxCal;
#endif

  if (CPR == 0) CPR = 4096;

  ROTATION_MAX = int32_t(float(CPR) / 360.0f * float(ROTATION_DEG));
  ROTATION_MID = ROTATION_MAX >> 1;

  InitInputs();
  FfbSetDriver(0);

  ffbs.x     = 0;
  gTorqueOut = 0;
  gEncPos_f  = 0.0f;

  dz  = 0;
  bdz = 2047;
  last_LC_scaling = LC_scaling;
  last_refresh    = micros();
}

// =================================================================
void loop() {
  now_micros           = micros();
  timeDiffConfigSerial = now_micros - last_ConfigSerial;

  if ((now_micros - last_refresh) >= CONTROL_PERIOD) {
    last_refresh = now_micros;

    // 1) Recebe feedback do STM32 + integra posição
    stmEncPoll();

    // 2) Posição do volante
    //    - Se tiver encoder real no Arduino: substitua gEncPos_f pela leitura do encoder
    //    - Por ora usa integração da velocidade (deriva ao longo do tempo)
    turn.x = (int32_t)gEncPos_f;
    axis.x = turn.x;

    // 3) Calcula torque
#if USE_FIXED_TORQUE_TEST
    int32_t torque = FIXED_TORQUE_VALUE;   // positivo = gira num sentido
#else
    ffbs   = gFFB.CalcTorqueCommands(&axis);
    int32_t torque = ffbs.x;
#endif

    // 4) Aplica limites e filtros
    torque = constrain(torque, -1000L, 1000L);
    int16_t t16 = (int16_t)torque;
    t16 = applyDeadband(t16, TORQUE_DEADBAND);
    t16 = constrain(t16, (int16_t)-TORQUE_MAX, (int16_t)TORQUE_MAX);
    t16 = slewLimit(t16, gTorqueOut, TORQUE_SLEW);
    gTorqueOut = t16;

    // 5) Envia comando ao STM32 (steer=0, speed=torque para motor DD único)
    stmSendCmd(0, gTorqueOut);

    // 6) HID report
    //    Mapeia posição do encoder para range HID (-32767..32767)
    int32_t hidPos = 0;
    if (ROTATION_MAX > 0) {
      hidPos = (int32_t)((float)turn.x / (float)ROTATION_MAX * (float)MID_REPORT_X);
    }
    hidPos = constrain(hidPos, -MID_REPORT_X - 1, MID_REPORT_X);

    accel.val  = 0;
    clutch.val = 0;
    hbrake.val = 0;
    brake.val  = 0;

    button = readInputButtons();
    SendInputReport(hidPos + MID_REPORT_X + 1,
                    brake.val, accel.val, clutch.val, hbrake.val, button);

#ifdef USE_CONFIGCDC
    if (timeDiffConfigSerial >= CONFIG_SERIAL_PERIOD) {
      configCDC();
      last_ConfigSerial = now_micros;
    }
#endif

    // 7) Debug 2x/segundo
    static uint32_t lastDbg = 0;
    uint32_t nowMs = millis();
    if ((nowMs - lastDbg) >= PRINT_DEBUG_EVERY_MS) {
      lastDbg = nowMs;
      CONFIG_SERIAL.print(F("link="));    CONFIG_SERIAL.print(gHaveEnc ? 1 : 0);
      CONFIG_SERIAL.print(F(" speedR=")); CONFIG_SERIAL.print(gSpeedR);
      CONFIG_SERIAL.print(F(" pos="));    CONFIG_SERIAL.print((long)turn.x);
      CONFIG_SERIAL.print(F(" bat="));    CONFIG_SERIAL.print(gBatVoltage / 100.0f, 1);
      CONFIG_SERIAL.print(F("V temp="));  CONFIG_SERIAL.print(gBoardTemp / 10.0f, 1);
      CONFIG_SERIAL.print(F("C torq="));  CONFIG_SERIAL.println(gTorqueOut);
    }
  }
}
/* Arduino Leonardo/Pro Micro Force Feedback Wheel firmware
   PC/HID -> Pro Micro -> Serial1 -> STM32 (hoverboard-firmware-hack-FOC)
   STM32 -> Serial1 -> Pro Micro (feedback velocidade)

   NOTAS:
   - Torque via Serial1 formato SerialCommand: start=0xABCD, steer, speed, checksum
   - Feedback via Serial1 formato SerialFeedback: 18 bytes (ver StmFrames.h)
   - Motor único DD: steer=0, speed=torque
   - Encoder de posição: MT6701 via ABZ direto no STM32 (não retorna posição no feedback)
     → Posição deve ser lida por encoder ABZ externo no Arduino OU integrada da velocidade
*/

// ========================= CONFIG RÁPIDA =========================
#define USE_FIXED_TORQUE_TEST   1     // 1 = ignora FFB, manda torque fixo para teste
#define FIXED_TORQUE_VALUE      400   // -1000..1000 (comece com 400, suba depois)

#define PRINT_DEBUG_EVERY_MS    500   // debug no Serial (2x por segundo)
#define ENC_LINK_TIMEOUT_MS     200   // timeout do link STM32
// =================================================================

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include "StmFrames.h"
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


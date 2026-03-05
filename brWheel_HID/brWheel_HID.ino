/* Arduino Leonardo/Pro Micro Force Feedback Wheel firmware
   PC/HID -> Pro Micro -> Serial1 -> STM32 (hoverboard-firmware-hack-FOC)
   STM32 -> Serial1 -> Pro Micro (feedback velocidade)

   NOTAS:
   - Torque via Serial1 formato SerialCommand: start=0xABCD, steer, speed, checksum
   - Feedback via Serial1 formato SerialFeedback: 18 bytes (ver StmFrames.h)
   - Motor único DD: steer=0, speed=torque
   - Posição encoder: STM32 envia get_x_TotalCount() (ticks acumulados) em speedL_meas.
     Mapeamento 1:1 tick→unidade. Com MT6701 1024PPR (CPR=4096) e ±540°: ±6144 ticks máx.
     Negação aplicada em gEncPos_f para compensar INVERT_R_DIRECTION no STM32.
*/

// ========================= CONFIG RÁPIDA =========================
#define USE_FIXED_TORQUE_TEST   0     // 0 = FFB real do jogo (1 = torque fixo para teste)
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
static float    gEncPos_f  = 0.0f;
static float    gPosOffset = 0.0f;  // offset de centro: subtraído de cada leitura STM32
volatile bool   gResetPosition = false; // setado pelo comando C do Wheel Control

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
      // Auto-centro no primeiro frame: salva posição atual do STM32 como offset.
      // Sem isso, o volante começa deslocado (ex: speedL_meas=3997 com gPosOffset=0).
      if (!gHaveEnc) { gPosOffset = (float)f.speedL_meas; }
      gHaveEnc    = true;
      gLastEncRxMs = millis();

      // Posicao real do encoder (MT6701 via STM32, campo speedL_meas)
      // O firmware envia get_x_TotalCount() clamped para int16 (-32768..32767).
      // get_x_TotalCount() retorna contagem acumulada em ticks de encoder.
      // Com MT6701 1024 PPR (CPR=4096) e ROTATION_DEG=1080:
      //   ROTATION_MAX = CPR/360 * ROTATION_DEG = 4096/360 * 1080 = 12288 ticks
      // Portanto 1 tick de encoder = 1 unidade de posição do Arduino.
      //
      // BUG FIX 4: equação anterior estava ERRADA:
      //   gEncPos_f = speedL_meas / 32767.0f * ROTATION_MAX
      //   → comprimia a posição em ~37% (12288/32767 ≈ 0.375)
      //   → ex: volante a 540° (6144 ticks) reportava ~2304 em vez de 6144
      //   → FFB calculava forças com ângulo errado (efeito spring/stop deslocado)
      //
      // Correto: ticks encoder = unidades de posição, mapeamento 1:1
      gEncPos_f = (float)f.speedL_meas - gPosOffset; // direita=positivo; subtrai offset de centro
      gEncPos_f = constrain(gEncPos_f, (float)-ROTATION_MAX, (float)ROTATION_MAX);
    }
  }

  // Timeout de link
  if (gHaveEnc && (millis() - gLastEncRxMs) > ENC_LINK_TIMEOUT_MS) {
    gHaveEnc  = false;
    gSpeedR   = 0;
    gEncPos_f = 0.0f;  // centra o volante: evita que spring/stop mantenha torque com posição congelada
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
// TORQUE_MAX: pico de torque enviado ao STM32 (escala -1000..+1000).
//   600 era conservador demais — impactos saíam em ~30% da força real.
//   950 deixa headroom de 5% contra clipping e permite picos de meio-fio completos.
//   Para motores mais fortes ou usuários experientes: 1000 (máximo).
static const int16_t TORQUE_MAX      = 950;
// TORQUE_SLEW: variação máxima por ciclo a 500Hz (2ms/ciclo).
//   80 → 0→950 em ~19 ciclos = 38ms — matava completamente impactos de <30ms.
//   250 → 0→950 em ~4 ciclos = 8ms — transientes de meio-fio chegam com força real.
//   Se sentir oscilação/buzz em alta frequência, reduzir para 180.
static const int16_t TORQUE_SLEW     = 250;

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
// Keepalive público: chamado durante calibrate() para evitar timeout no STM32.
// Envia frame zero (torque=0, steer=0) via Serial1 mantendo o link ativo.
void stmSendCmdKeepalive() { stmSendCmd(0, 0); }

void setup() {
  // Desabilita reset por DTR: quando o Wheel Control fecha, o Windows derruba o DTR
  // da porta CDC. No Leonardo/Pro Micro, DTR caindo causa reset do MCU, interrompendo
  // o Serial1 por ~5s (boot + alinhamento) e fazendo o STM32 entrar em timeout.
  // UDCON &= ~(1<<DETACH); não é necessário aqui — o boot já passou.
  // A proteção real é o keepalive no loop: mesmo sem Wheel Control, o Arduino
  // continua enviando stmSendCmd(0,0) a cada 2ms → STM32 nunca faz timeout.
  CONFIG_SERIAL.begin(115200);
  // FIX: reduz timeout do parseInt() de 1000ms para 10ms.
  // Sem isso, bytes lixo ao fechar o Wheel Control disparam parseInt() que bloqueia
  // por 1s → HID reports param → Windows remove o dispositivo HID.
  CONFIG_SERIAL.setTimeout(10);

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

  // CRÍTICO — Serial DD: sem isso EffectDivider()=INF → todos os efeitos = 0.
  TOP                 = 1000;
  MM_MAX_MOTOR_TORQUE = 1000;
  MM_MIN_MOTOR_TORQUE = 0;

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
    // Botão Center do Wheel Control: salva posição atual como offset.
    // Na próxima leitura stmEncPoll subtrai gPosOffset → gEncPos_f=0.
    if (gResetPosition) {
      gResetPosition = false;
      // Fórmula corrigida: new_gPosOffset = gPosOffset + gEncPos_f = speedL_meas atual.
      // Prova: gEncPos_f = speedL_meas - gPosOffset → novo centro: speedL_meas - novo_gPosOffset = 0
      //        → novo_gPosOffset = speedL_meas = gPosOffset + gEncPos_f
      gPosOffset += (float)gEncPos_f; // correto: acumula offset = speedL_meas corrente
      gEncPos_f   = 0.0f;
      gTorqueOut  = 0;
      stmSendCmd(0, 0);
    }

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
    // Envia torque ao STM32 SEMPRE (mesmo com Wheel Control fechado).
    // Isso mantém o keepalive serial ativo e impede timeout no STM32.
    // Sem FFB ativo (jogo fechado/sem efeitos), gTorqueOut=0 → motor livre.
    stmSendCmd(0, -gTorqueOut); // neg: INVERT_R_DIRECTION no STM32 re-inverte

    // 6) HID report
    //    Mapeia posição do encoder para range HID (-32767..32767)
    int32_t hidPos = 0;
    if (ROTATION_MAX > 0) {
      hidPos = -(int32_t)((float)turn.x / (float)ROTATION_MAX * (float)MID_REPORT_X);
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

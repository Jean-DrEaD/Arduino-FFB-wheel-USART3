/*
 * brWheel FFB — ESP32-S3 Zero / Mini
 * Port completo do brWheel_HID (Pro Micro) para ESP32-S3.
 *
 * Funcionalidades:
 *   - USB HID idêntico ao Pro Micro (compatível com WheelControl, BeamNG, AC, iRacing)
 *   - Force Feedback completo via gFFB.CalcTorqueCommands() (ffb_pro.ino original)
 *   - Serial 500kbps → GD32/STM32 (hoverboard-firmware-hack-FOC-USART3)
 *   - Pedais analógicos ADC 12-bit | Câmbio sequencial | Botões extras
 *   - Auto-center via alinhamento do GD32 (sem encoder local)
 *
 * Requer: arduino-esp32 >= 3.0
 *   Board: "ESP32S3 Dev Module"
 *   USB Mode: "USB-OTG (TinyUSB)"
 *   USB CDC On Boot: Disabled
 *   Upload Mode: UART0 / Hardware CDC
 *   CPU Frequency: 240MHz
 *
 * Autores originais: Tero Loimunema, Etienne Saint-Paul, Fernando Igor,
 *                    Milos Rankovic (ranenbg)
 * Port ESP32-S3 + GD32: Jean-DrEaD, 2026
 */

// ============================================================================
//  HEADERS
// ============================================================================
#include "Arduino.h"
#include "USB.h"
#include "USBHID.h"
#include "Config_ESP32S3.h"
#include "HID_Wheel_ESP32.h"
#include "StmFrames.h"
#include "ffb.h"
#include "ffb_pro.h"

// ============================================================================
//  INSTÂNCIAS GLOBAIS USB
// ============================================================================
USBHID HID;
WheelFFBDevice WheelFFB;

// ============================================================================
//  GLOBAIS COMPARTILHADOS COM ffb_pro.ino / ffb.ino
//  (espelham Config.h e brWheel_HID.ino do Pro Micro)
// ============================================================================

// Instâncias FFB (referenciadas extern em ffb_pro.h)
cFFB  gFFB;
BRFFB brWheelFFB;

// Estado de efeitos desktop
u8  effstate  = 0b00000001;  // bit0=autospring ativo por padrão
u8  pwmstate  = 0b00001100;  // compatibilidade (não usado no DD serial)

// Ganhos FFB (overwrite via WheelControl CDC)
u8  configGeneralGain  = DEFAULT_GENERAL_GAIN;
u8  configSpringGain   = DEFAULT_SPRING_GAIN;
u8  configDamperGain   = DEFAULT_DAMPER_GAIN;
u8  configFrictionGain = DEFAULT_FRICTION_GAIN;
u8  configInertiaGain  = DEFAULT_INERTIA_GAIN;
u8  configConstantGain = DEFAULT_CONSTANT_GAIN;
u8  configPeriodicGain = DEFAULT_PERIODIC_GAIN;
u8  configCenterGain   = DEFAULT_CENTER_GAIN;
u8  configStopGain     = DEFAULT_STOP_GAIN;

// Encoder / Rotação
int32_t CPR          = ENCODER_CPR_DEFAULT;   // 4096 (MT6701)
int16_t ROTATION_DEG = ROTATION_DEG_DEFAULT;  // 900°
int32_t ROTATION_MAX = 0;                     // calculado em setup()
int32_t ROTATION_MID = 0;                     // ROTATION_MAX / 2

// Motor / PWM (escala compatível com ffb_pro.ino)
// TOP=1000 → EffectDivider=32.767 → CalcTorqueCommands devolve ±1000
uint16_t TOP                 = 1000;
uint16_t MM_MAX_MOTOR_TORQUE = 1000;
uint16_t MM_MIN_MOTOR_TORQUE = 0;

// Slew limiter (clip adicional antes de enviar ao GD32)
static const int16_t TORQUE_MAX  = TORQUE_MAX_DEFAULT;   // 700
static const int16_t TORQUE_SLEW = TORQUE_SLEW_DEFAULT;  // 150

// ============================================================================
//  STUB configHID (report 0xF1 do WheelControl)
//  Impl. completa idêntica ao ConfigHID.ino do Pro Micro
// ============================================================================
void configHID(USB_ConfigReport* data) {
  if (data->Info == 1) {
    int16_t temp = constrain(data->Rotation, 30, 1800);
    ROTATION_DEG = temp;
    ROTATION_MAX = (int32_t)CPR * temp / 360;
    ROTATION_MID = ROTATION_MAX / 2;

    configGeneralGain  = constrain(data->GeneralGain,  0, 200);
    configConstantGain = constrain(data->ConstantGain, 0, 200);
    configDamperGain   = constrain(data->DamperGain,   0, 200);
    configFrictionGain = constrain(data->FrictionGain, 0, 200);
    configPeriodicGain = constrain(data->PeriodicGain, 0, 200);
    configSpringGain   = constrain(data->SpringGain,   0, 200);
    configInertiaGain  = constrain(data->InertiaGain,  0, 200);
    configCenterGain   = constrain(data->CenterGain,   0, 200);
    configStopGain     = constrain(data->StopGain,     0, 200);

    if (data->Calibrate) brWheelFFB.calibrate(); // stub no ESP32
  }
}

// ============================================================================
//  ESTADO GLOBAL DO VOLANTE
// ============================================================================

// Encoder / posição
static volatile float gEncPos_f  = 0.0f;
static float          gPosOffset = 0.0f;
static bool           gHaveEnc   = false;

// Auto-center
static bool     gAutoCenter_done     = false;
static uint32_t gAutoCenter_stableMs = 0;
static int16_t  gAutoCenter_lastPos  = 0;
static int32_t  gAutoCenter_sum      = 0;
static uint32_t gAutoCenter_count    = 0;

// Torque
static int16_t gTorqueOut = 0;

// Reset de posição (botão Center WheelControl)
static bool gResetPosition = false;

// Frame STM32
static StmEncFrame gLastFrame;
static uint8_t     gRxBuf[sizeof(StmEncFrame) * 2];
static uint8_t     gRxIdx = 0;

// Pedais
typedef struct { int16_t val; int16_t min; int16_t max; } AxisState;
typedef struct { int32_t val; int16_t min; int16_t max; } AxisState32;

static AxisState   gAccel  = {0, ACCEL_MIN_DEFAULT,  ACCEL_MAX_DEFAULT};
static AxisState32 gBrake  = {0, BRAKE_MIN_DEFAULT,  BRAKE_MAX_DEFAULT};
static AxisState   gClutch = {0, CLUTCH_MIN_DEFAULT, CLUTCH_MAX_DEFAULT};
static AxisState   gHbrake = {0, 0, 4095};

// Timer
static uint32_t gLastRefresh = 0;

// ============================================================================
//  SERIAL STM32
// ============================================================================

static inline void stmSendCmd(int16_t steer, int16_t speed) {
  StmCmdFrame f;
  f.start    = 0xABCD;
  f.steer    = steer;
  f.speed    = speed;
  f.checksum = f.start ^ (uint16_t)steer ^ (uint16_t)speed;
  STM_UART.write((uint8_t*)&f, sizeof(f));
}

// Keepalive para blocos que usam delay() (calibrate stub)
void stmSendCmdKeepalive() { stmSendCmd(0, 0); }

static void stmEncPoll() {
  while (STM_UART.available()) {
    uint8_t b = STM_UART.read();
    gRxBuf[gRxIdx++] = b;
    if (gRxIdx >= sizeof(StmEncFrame)) gRxIdx = 0;

    if (gRxIdx >= sizeof(StmEncFrame)) {
      StmEncFrame *f = (StmEncFrame*)gRxBuf;
      if (f->start == 0xABCD) {
        uint16_t cs = f->start ^ (uint16_t)f->cmd1     ^ (uint16_t)f->cmd2
                    ^ (uint16_t)f->speedR_meas          ^ (uint16_t)f->speedL_meas
                    ^ (uint16_t)f->batVoltage           ^ (uint16_t)f->boardTemp
                    ^ (uint16_t)f->cmdLed;
        if (cs == f->checksum) {
          memcpy(&gLastFrame, f, sizeof(StmEncFrame));
          gHaveEnc = true;

          // speedL_meas = -enc_pos (negado no GD32 main.c)
          float newPos = (float)(-(int32_t)f->speedL_meas) - gPosOffset;
          gEncPos_f = constrain(newPos, (float)-ROTATION_MAX, (float)ROTATION_MAX);
          gRxIdx = 0;

          // Auto-center: aguarda posição estável por AUTO_CENTER_SETTLE_MS
          if (!gAutoCenter_done) {
            int16_t curPos = (int16_t)f->speedL_meas;
            if (abs((int32_t)curPos - (int32_t)gAutoCenter_lastPos) > AUTO_CENTER_DEADBAND) {
              gAutoCenter_stableMs = millis();
              gAutoCenter_lastPos  = curPos;
              gAutoCenter_sum      = 0;
              gAutoCenter_count    = 0;
            } else {
              gAutoCenter_sum += curPos;
              gAutoCenter_count++;
              if ((millis() - gAutoCenter_stableMs) >= AUTO_CENTER_SETTLE_MS) {
                float avgPos = (gAutoCenter_count > 0)
                             ? (float)gAutoCenter_sum / (float)gAutoCenter_count
                             : (float)curPos;
                gPosOffset       = -avgPos;
                gEncPos_f        = 0.0f;
                gAutoCenter_done = true;
                gTorqueOut       = 0;
                stmSendCmd(0, 0);
                CONFIG_SERIAL.println("AUTO-CENTER OK");
              }
            }
          }
        } else {
          memmove(gRxBuf, gRxBuf + 1, sizeof(gRxBuf) - 1);
          gRxIdx = sizeof(StmEncFrame) - 1;
        }
      } else {
        memmove(gRxBuf, gRxBuf + 1, sizeof(gRxBuf) - 1);
        gRxIdx = sizeof(StmEncFrame) - 1;
      }
    }
  }
}

// ============================================================================
//  SLEW LIMITER
// ============================================================================
static int16_t slewLimit(int16_t target, int16_t current, int16_t slew) {
  int16_t diff = target - current;
  if (diff >  slew) diff =  slew;
  if (diff < -slew) diff = -slew;
  return current + diff;
}

// ============================================================================
//  PEDAIS
// ============================================================================
static void readPedals() {
  int rawAccel = analogRead(ACCEL_PIN);
  int rawBrake = analogRead(BRAKE_PIN);

  rawAccel = constrain(rawAccel, gAccel.min, gAccel.max);
  rawBrake = constrain(rawBrake, gBrake.min, gBrake.max);

  // ADC 12-bit → 0..4095 direto (campo HID Z=12bit, Y=16bit)
  gAccel.val = (int16_t)map(rawAccel, gAccel.min, gAccel.max, 0, 4095);
  gBrake.val = (int32_t)map(rawBrake, gBrake.min, gBrake.max, 0, 4095);

  // Descomentar quando embreagem estiver conectada:
  // gClutch.val = map(analogRead(CLUTCH_PIN), gClutch.min, gClutch.max, 0, 4095);
  gClutch.val = 0;
  gHbrake.val = 0;
}

// ============================================================================
//  BOTÕES
// ============================================================================
static uint32_t readButtons() {
  uint32_t buttons = 0;

  if (!digitalRead(BUTTON_0_PIN)) bitSet(buttons, 4);
  if (!digitalRead(BUTTON_1_PIN)) bitSet(buttons, 5);
  if (!digitalRead(BUTTON_2_PIN)) bitSet(buttons, 6);
  if (!digitalRead(BUTTON_3_PIN)) bitSet(buttons, 7);

  if (!digitalRead(SHIFTER_UP_PIN)) bitSet(buttons, SHIFTER_UP_BIT);
  if (!digitalRead(SHIFTER_DN_PIN)) bitSet(buttons, SHIFTER_DN_BIT);

  // Freio de mão digital (descomentar + #define HBRAKE_BTN_PIN):
  // if (!digitalRead(HBRAKE_BTN_PIN)) bitSet(buttons, HBRAKE_BTN_BIT);

  return buttons;
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  CONFIG_SERIAL.begin(CONFIG_SERIAL_BAUD);

  // ADC 12-bit, 0-3.3V
  analogReadResolution(PEDAL_ADC_BITS);
  analogSetAttenuation(ADC_11db);

  // Pinos de entrada
  pinMode(SHIFTER_UP_PIN, INPUT_PULLUP);
  pinMode(SHIFTER_DN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_0_PIN,   INPUT_PULLUP);
  pinMode(BUTTON_1_PIN,   INPUT_PULLUP);
  pinMode(BUTTON_2_PIN,   INPUT_PULLUP);
  pinMode(BUTTON_3_PIN,   INPUT_PULLUP);
  // pinMode(HBRAKE_BTN_PIN, INPUT_PULLUP);

  // UART STM32/GD32
  STM_UART.begin(STM_UART_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);

  // Calcula limites de rotação
  ROTATION_MAX = (int32_t)CPR * ROTATION_DEG / 360;
  ROTATION_MID = ROTATION_MAX >> 1;

  // FFB
  effstate = 0b00000001;   // autospring ativo
  FfbInit();               // inicializa driver + limpa efeitos (sem callback USB AVR)

  // USB HID
  WheelFFB.begin(HID);
  HID.begin();
  USB.begin();

  gLastRefresh = micros();
  CONFIG_SERIAL.println(F("brWheel FFB ESP32-S3 OK"));
}

// ============================================================================
//  LOOP PRINCIPAL (500Hz)
// ============================================================================
void loop() {
  uint32_t now = micros();
  if ((now - gLastRefresh) < CONTROL_PERIOD_US) return;
  gLastRefresh = now;

  // 1) Recebe feedback de posição do GD32
  stmEncPoll();

  // 2) Reset de posição (botão Center WheelControl)
  if (gResetPosition) {
    gResetPosition = false;
    gPosOffset    += gEncPos_f;
    gEncPos_f      = 0.0f;
    gTorqueOut     = 0;
    stmSendCmd(0, 0);
  }

  // 3) Calcula torque FFB via motor completo (ffb_pro.ino)
  //    CalcTorqueCommands retorna em [-MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE] = [-1000, 1000]
  s32v axis;
  axis.x = (s32)gEncPos_f;
  s32v ffbs = gFFB.CalcTorqueCommands(&axis);

  // Avança timer FFB (igual ao Pro Micro: incrementa em cada ciclo de 2ms)
  t0 = millis();

  // 4) Slew limiter + clip final
  int16_t t16 = (int16_t)constrain((int32_t)ffbs.x, -TORQUE_MAX, TORQUE_MAX);
  t16         = slewLimit(t16, gTorqueOut, TORQUE_SLEW);
  gTorqueOut  = t16;

  // 5) Envia torque ao GD32
  stmSendCmd(0, gTorqueOut);

  // 6) Lê pedais e botões
  readPedals();
  uint32_t buttons = readButtons();

  // 7) Mapeia posição → range HID X (0..65535, centro=32768)
  int32_t hidPos = 0;
  if (ROTATION_MAX > 0)
    hidPos = (int32_t)((float)axis.x / (float)ROTATION_MAX * (float)MID_REPORT_X);
  hidPos = constrain(hidPos, -(int32_t)MID_REPORT_X - 1, (int32_t)MID_REPORT_X);
  uint16_t hidX  = (uint16_t)(hidPos + MID_REPORT_X + 1);

  // Y = freio 16-bit (12-bit → 16-bit via shift ×16)
  uint16_t hidY  = (uint16_t)((gBrake.val  << 4) & 0xFFFF);
  uint16_t hidZ  = (uint16_t)(gAccel.val  & 0x0FFF);  // acelerador 12-bit
  uint16_t hidRX = (uint16_t)(gClutch.val & 0x0FFF);  // embreagem 12-bit
  uint16_t hidRY = (uint16_t)(gHbrake.val & 0x0FFF);  // freio mão 12-bit

  // 8) Envia report HID
  WheelFFB.SendJoystickReport(hidX, hidY, hidZ, hidRX, hidRY, buttons);

  // 9) Debug 2×/segundo
  static uint32_t lastDbgMs = 0;
  uint32_t nowMs = millis();
  if ((nowMs - lastDbgMs) >= 500) {
    lastDbgMs = nowMs;
    CONFIG_SERIAL.print(F("link="));  CONFIG_SERIAL.print(gHaveEnc ? 1 : 0);
    CONFIG_SERIAL.print(F(" pos="));  CONFIG_SERIAL.print(axis.x);
    CONFIG_SERIAL.print(F(" torq=")); CONFIG_SERIAL.print(gTorqueOut);
    CONFIG_SERIAL.print(F(" auto=")); CONFIG_SERIAL.print(gFFB.mAutoCenter ? "spring" : "FFB");
    CONFIG_SERIAL.print(F(" acc="));  CONFIG_SERIAL.print(gAccel.val);
    CONFIG_SERIAL.print(F(" brk="));  CONFIG_SERIAL.print(gBrake.val);
    if (gLastFrame.batVoltage > 0) {
      CONFIG_SERIAL.print(F(" bat="));
      CONFIG_SERIAL.print(gLastFrame.batVoltage / 100.0f, 1);
      CONFIG_SERIAL.print(F("V"));
    }
    CONFIG_SERIAL.println();
  }
}

/*
 * brWheel FFB — ESP32-S3 Zero / Mini
 * Port completo do brWheel_HID (Pro Micro) para ESP32-S3.
 *
 * Funcionalidades:
 *   - USB HID idêntico ao Pro Micro (compatível com WheelControl, BeamNG, AC, iRacing)
 *   - Force Feedback completo via gFFB.CalcTorqueCommands() (ffb_pro.ino original)
 *   - Serial 500kbps → GD32/STM32 (hoverboard-firmware-hack-FOC-USART3)
 *   - Pedais analógicos ADC 12-bit | Load Cell HX711 opcional | Câmbio sequencial | Botões
 *   - configCDC(): interface serial completa compatível com WheelControl (U/V/B/G/E/Y/F/C/S/R/A)
 *   - Preferences: salva ganhos, rotação e LC_scaling em flash (equivalente ao EEPROM AVR)
 *   - applyDeadband(): elimina ruído < 5 unidades antes de enviar ao GD32
 *   - Auto-center via posição estável do GD32 (sem encoder local)
 *
 * Requer: arduino-esp32 >= 3.0
 *   Board: "ESP32S3 Dev Module"
 *   USB Mode: "USB-OTG (TinyUSB)"
 *   USB CDC On Boot: Disabled
 *   CPU Frequency: 240MHz
 *
 * Para load cell:
 *   Descomente #define USE_LOAD_CELL em Config_ESP32S3.h
 *   Instale biblioteca "HX711_ADC" por Olav Kallhovd (Gerenciador de Bibliotecas)
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
#include <Preferences.h>
#include "Config_ESP32S3.h"
#include "HID_Wheel_ESP32.h"
#include "StmFrames.h"
#include "ffb.h"
#include "ffb_pro.h"

#ifdef USE_LOAD_CELL
  #include <HX711_ADC.h>
#endif

// ============================================================================
//  INSTÂNCIAS GLOBAIS
// ============================================================================
USBHID      HID;
WheelFFBDevice WheelFFB;
Preferences prefs;

#ifdef USE_LOAD_CELL
  HX711_ADC LoadCell(LC_DOUT_PIN, LC_SCK_PIN);
#endif

// ============================================================================
//  GLOBAIS COMPARTILHADOS COM ffb_pro.ino / ffb.ino
// ============================================================================
cFFB  gFFB;
BRFFB brWheelFFB;

u8  effstate  = 0b00000001;
u8  pwmstate  = 0b00001100;

u8  configGeneralGain  = DEFAULT_GENERAL_GAIN;
u8  configSpringGain   = DEFAULT_SPRING_GAIN;
u8  configDamperGain   = DEFAULT_DAMPER_GAIN;
u8  configFrictionGain = DEFAULT_FRICTION_GAIN;
u8  configInertiaGain  = DEFAULT_INERTIA_GAIN;
u8  configConstantGain = DEFAULT_CONSTANT_GAIN;
u8  configPeriodicGain = DEFAULT_PERIODIC_GAIN;
u8  configCenterGain   = DEFAULT_CENTER_GAIN;
u8  configStopGain     = DEFAULT_STOP_GAIN;

int32_t  CPR          = ENCODER_CPR_DEFAULT;
int16_t  ROTATION_DEG = ROTATION_DEG_DEFAULT;
int32_t  ROTATION_MAX = 0;
int32_t  ROTATION_MID = 0;

uint16_t TOP                 = 1000;
uint16_t MM_MAX_MOTOR_TORQUE = 1000;
uint16_t MM_MIN_MOTOR_TORQUE = 0;
uint8_t  LC_scaling          = LC_SCALING_DEFAULT;

static int16_t TORQUE_MAX  = TORQUE_MAX_DEFAULT;
static int16_t TORQUE_SLEW = TORQUE_SLEW_DEFAULT;

// ============================================================================
//  ESTADO INTERNO
// ============================================================================
static volatile float gEncPos_f  = 0.0f;
static float          gPosOffset = 0.0f;
static bool           gHaveEnc   = false;

static bool     gAutoCenter_done     = false;
static uint32_t gAutoCenter_stableMs = 0;
static int16_t  gAutoCenter_lastPos  = 0;
static int32_t  gAutoCenter_sum      = 0;
static uint32_t gAutoCenter_count    = 0;

static int16_t gTorqueOut   = 0;
volatile bool  gResetPosition = false;

static StmEncFrame gLastFrame;
static uint8_t     gRxBuf[sizeof(StmEncFrame) * 2];
static uint8_t     gRxIdx = 0;

typedef struct { int16_t val; int16_t min; int16_t max; } AxisState;
typedef struct { int32_t val; int16_t min; int16_t max; } AxisState32;

static AxisState   gAccel  = {0, ACCEL_MIN_DEFAULT,  ACCEL_MAX_DEFAULT};
static AxisState32 gBrake  = {0, BRAKE_MIN_DEFAULT,  BRAKE_MAX_DEFAULT};
static AxisState   gClutch = {0, CLUTCH_MIN_DEFAULT, CLUTCH_MAX_DEFAULT};
static AxisState   gHbrake = {0, 0, 4095};

static uint32_t gLastRefresh = 0;

// ============================================================================
//  PREFERENCES — salva/carrega ganhos e parâmetros em flash
// ============================================================================
static void savePrefs() {
  prefs.begin("brwheel", false);
  prefs.putShort("rot",  ROTATION_DEG);
  prefs.putInt  ("cpr",  CPR);
  prefs.putUChar("gg",   configGeneralGain);
  prefs.putUChar("sg",   configSpringGain);
  prefs.putUChar("dg",   configDamperGain);
  prefs.putUChar("fg",   configFrictionGain);
  prefs.putUChar("ig",   configInertiaGain);
  prefs.putUChar("cng",  configConstantGain);
  prefs.putUChar("pg",   configPeriodicGain);
  prefs.putUChar("ceg",  configCenterGain);
  prefs.putUChar("stg",  configStopGain);
  prefs.putUChar("eff",  effstate);
  prefs.putUChar("lcs",  LC_scaling);
  prefs.putShort("brmi", gBrake.min);
  prefs.putShort("brma", gBrake.max);
  prefs.putShort("acmi", gAccel.min);
  prefs.putShort("acma", gAccel.max);
  prefs.putShort("clmi", gClutch.min);
  prefs.putShort("clma", gClutch.max);
  prefs.end();
  CONFIG_SERIAL.println(1);
}

static void loadPrefs() {
  prefs.begin("brwheel", true);
  ROTATION_DEG      = prefs.getShort("rot",  ROTATION_DEG_DEFAULT);
  CPR               = prefs.getInt  ("cpr",  ENCODER_CPR_DEFAULT);
  configGeneralGain = prefs.getUChar("gg",   DEFAULT_GENERAL_GAIN);
  configSpringGain  = prefs.getUChar("sg",   DEFAULT_SPRING_GAIN);
  configDamperGain  = prefs.getUChar("dg",   DEFAULT_DAMPER_GAIN);
  configFrictionGain= prefs.getUChar("fg",   DEFAULT_FRICTION_GAIN);
  configInertiaGain = prefs.getUChar("ig",   DEFAULT_INERTIA_GAIN);
  configConstantGain= prefs.getUChar("cng",  DEFAULT_CONSTANT_GAIN);
  configPeriodicGain= prefs.getUChar("pg",   DEFAULT_PERIODIC_GAIN);
  configCenterGain  = prefs.getUChar("ceg",  DEFAULT_CENTER_GAIN);
  configStopGain    = prefs.getUChar("stg",  DEFAULT_STOP_GAIN);
  effstate          = prefs.getUChar("eff",  0b00000001);
  LC_scaling        = prefs.getUChar("lcs",  LC_SCALING_DEFAULT);
  gBrake.min        = prefs.getShort("brmi", BRAKE_MIN_DEFAULT);
  gBrake.max        = prefs.getShort("brma", BRAKE_MAX_DEFAULT);
  gAccel.min        = prefs.getShort("acmi", ACCEL_MIN_DEFAULT);
  gAccel.max        = prefs.getShort("acma", ACCEL_MAX_DEFAULT);
  gClutch.min       = prefs.getShort("clmi", CLUTCH_MIN_DEFAULT);
  gClutch.max       = prefs.getShort("clma", CLUTCH_MAX_DEFAULT);
  prefs.end();
}

// ============================================================================
//  UART STM32 / GD32
// ============================================================================
static inline void stmSendCmd(int16_t steer, int16_t speed) {
  StmCmdFrame f;
  f.start    = 0xABCD;
  f.steer    = steer;
  f.speed    = speed;
  f.checksum = f.start ^ (uint16_t)steer ^ (uint16_t)speed;
  STM_UART.write((uint8_t*)&f, sizeof(f));
}

void stmSendCmdKeepalive() { stmSendCmd(0, 0); }

static void stmEncPoll() {
  while (STM_UART.available()) {
    uint8_t b = STM_UART.read();
    gRxBuf[gRxIdx++] = b;
    if (gRxIdx >= sizeof(gRxBuf)) gRxIdx = 0;

    if (gRxIdx >= sizeof(StmEncFrame)) {
      StmEncFrame *f = (StmEncFrame*)gRxBuf;
      if (f->start == 0xABCD) {
        uint16_t cs = f->start ^ (uint16_t)f->cmd1        ^ (uint16_t)f->cmd2
                    ^ (uint16_t)f->speedR_meas             ^ (uint16_t)f->speedL_meas
                    ^ (uint16_t)f->batVoltage              ^ (uint16_t)f->boardTemp
                    ^ (uint16_t)f->cmdLed;
        if (cs == f->checksum) {
          memcpy(&gLastFrame, f, sizeof(StmEncFrame));
          gHaveEnc   = true;

          float newPos = (float)(-(int32_t)f->speedL_meas) - gPosOffset;
          gEncPos_f   = constrain(newPos, -(float)ROTATION_MAX, (float)ROTATION_MAX);
          gRxIdx = 0;

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
                float avg = (gAutoCenter_count > 0)
                          ? (float)gAutoCenter_sum / (float)gAutoCenter_count
                          : (float)curPos;
                gPosOffset       = -avg;
                gEncPos_f        = 0.0f;
                gAutoCenter_done = true;
                gTorqueOut       = 0;
                stmSendCmd(0, 0);
                CONFIG_SERIAL.println(F("AUTO-CENTER OK"));
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
//  TORQUE — slew limiter + deadband
// ============================================================================
static int16_t slewLimit(int16_t target, int16_t current, int16_t slew) {
  int16_t d = target - current;
  if (d >  slew) d =  slew;
  if (d < -slew) d = -slew;
  return current + d;
}

static int16_t applyDeadband(int16_t x, int16_t db) {
  if (x > 0 && x <  db) return 0;
  if (x < 0 && x > -db) return 0;
  return x;
}

// ============================================================================
//  LOAD CELL
// ============================================================================
#ifdef USE_LOAD_CELL
static void initLoadCell() {
  LoadCell.begin();
  LoadCell.setGain(LC_GAIN);
  LoadCell.start(2000);  // 2s de estabilização
  LoadCell.setCalFactor(0.25f * (float)LC_scaling);
  CONFIG_SERIAL.println(F("LoadCell OK"));
}

// Lê load cell; retorna valor 0–4095 para o eixo Y HID
static int32_t readLoadCell() {
  static uint32_t lastUpdate = 0;
  static int32_t  lastVal    = 0;
  if (LoadCell.update() || (millis() - lastUpdate > 5)) {
    float raw = LoadCell.getData();   // valor em gramas/newtons (dependente do calFactor)
    int32_t val = (int32_t)constrain(raw, 0.0f, 4095.0f);
    lastVal    = val;
    lastUpdate = millis();
  }
  return lastVal;
}
#endif

// ============================================================================
//  PEDAIS
// ============================================================================
static void readPedals() {
  // Acelerador — sempre analógico
  int rawAccel = constrain(analogRead(ACCEL_PIN), gAccel.min, gAccel.max);
  gAccel.val   = (int16_t)map(rawAccel, gAccel.min, gAccel.max, 0, 4095);

  // Freio — load cell ou potenciômetro
#ifdef USE_LOAD_CELL
  gBrake.val = readLoadCell();
#else
  int rawBrake = constrain(analogRead(BRAKE_PIN), gBrake.min, gBrake.max);
  gBrake.val   = (int32_t)map(rawBrake, gBrake.min, gBrake.max, 0, 4095);
#endif

  // Embreagem (descomentar quando conectada)
  // int rawClutch = constrain(analogRead(CLUTCH_PIN), gClutch.min, gClutch.max);
  // gClutch.val   = (int16_t)map(rawClutch, gClutch.min, gClutch.max, 0, 4095);
  gClutch.val = 0;
  gHbrake.val = 0;
}

// ============================================================================
//  BOTÕES
// ============================================================================
static uint32_t readButtons() {
  uint32_t btn = 0;
  if (!digitalRead(BUTTON_0_PIN)) bitSet(btn, 4);
  if (!digitalRead(BUTTON_1_PIN)) bitSet(btn, 5);
  if (!digitalRead(BUTTON_2_PIN)) bitSet(btn, 6);
  if (!digitalRead(BUTTON_3_PIN)) bitSet(btn, 7);
  if (!digitalRead(SHIFTER_UP_PIN)) bitSet(btn, SHIFTER_UP_BIT);
  if (!digitalRead(SHIFTER_DN_PIN)) bitSet(btn, SHIFTER_DN_BIT);
  // if (!digitalRead(HBRAKE_BTN_PIN)) bitSet(btn, HBRAKE_BTN_BIT);
  return btn;
}

// ============================================================================
//  configHID — report 0xF1 do WheelControl (USB HID)
// ============================================================================
void configHID(USB_ConfigReport* data) {
  if (data->Info == 1) {
    int16_t temp = constrain(data->Rotation, 30, 1800);
    if (temp != ROTATION_DEG) {
      ROTATION_DEG = temp;
      ROTATION_MAX = (int32_t)CPR * ROTATION_DEG / 360;
      ROTATION_MID = ROTATION_MAX >> 1;
    }
    configGeneralGain  = constrain(data->GeneralGain,  0, 200);
    configConstantGain = constrain(data->ConstantGain, 0, 200);
    configDamperGain   = constrain(data->DamperGain,   0, 200);
    configFrictionGain = constrain(data->FrictionGain, 0, 200);
    configPeriodicGain = constrain(data->PeriodicGain, 0, 200);
    configSpringGain   = constrain(data->SpringGain,   0, 200);
    configInertiaGain  = constrain(data->InertiaGain,  0, 200);
    configCenterGain   = constrain(data->CenterGain,   0, 200);
    configStopGain     = constrain(data->StopGain,     0, 200);
    if (data->Calibrate) brWheelFFB.calibrate();
  }
}

// ============================================================================
//  configCDC — interface serial ASCII compatível com WheelControl
//
//  Comandos principais:
//    U        → envia todos os parâmetros (WheelControl lê ao conectar)
//    V        → versão do firmware
//    S        → estado FFB (brWheelFFB.state)
//    R        → calibração (stub)
//    C        → set center (reset posição)
//    G<deg>   → set rotação (ex: G900)
//    E<n>     → effstate (efeitos desktop, decimal)
//    B<n>     → LC_scaling load cell (1–255)
//    FG<n>    → general gain
//    FC<n>    → constant gain
//    FD<n>    → damper gain
//    FF<n>    → friction gain
//    FS<n>    → periodic gain
//    FM<n>    → spring gain
//    FI<n>    → inertia gain
//    FA<n>    → center gain
//    FB<n>    → stop gain
//    YA<n>    → brake min    | YB<n> → brake max
//    YC<n>    → accel min    | YD<n> → accel max
//    YE<n>    → clutch min   | YF<n> → clutch max
//    YG<n>    → hbrake min   | YH<n> → hbrake max
//    YR       → lê calibração de pedais
//    A        → salva tudo em flash (Preferences)
// ============================================================================
static uint8_t toUpper(uint8_t c) {
  return (c >= 'a' && c <= 'z') ? (c + 'A' - 'a') : c;
}

void configCDC() {
  if (!CONFIG_SERIAL.available()) return;
  uint8_t c = toUpper(CONFIG_SERIAL.read());
  int32_t temp;
  uint8_t ffb_temp;

  switch (c) {

    case 'U':  // todos os parâmetros (WheelControl lê ao conectar)
      CONFIG_SERIAL.print(ROTATION_DEG);       CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(configGeneralGain);  CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(configDamperGain);   CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(configFrictionGain); CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(configConstantGain); CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(configPeriodicGain); CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(configSpringGain);   CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(configInertiaGain);  CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(configCenterGain);   CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(configStopGain);     CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(MM_MIN_MOTOR_TORQUE);CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(LC_scaling);         CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(effstate, DEC);      CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(MM_MAX_MOTOR_TORQUE);CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.print(CPR);                CONFIG_SERIAL.print(' ');
      CONFIG_SERIAL.println(pwmstate, DEC);
      break;

    case 'V':  // versão firmware
      CONFIG_SERIAL.print(F("fw-v"));
      CONFIG_SERIAL.print(FIRMWARE_VERSION, DEC);
      CONFIG_SERIAL.print(F("l"));   // 'l' = USE_LOAD_CELL (informativo)
      CONFIG_SERIAL.println(F("m")); // 'm' = USE_PROMICRO (compat.)
      break;

    case 'S':  // estado FFB
      CONFIG_SERIAL.println(brWheelFFB.state, DEC);
      break;

    case 'R':  // calibração (stub — alinhamento feito pelo GD32)
      brWheelFFB.calibrate();
      break;

    case 'C':  // set center — reset posição atual para zero
      gResetPosition = true;
      CONFIG_SERIAL.println(1);
      break;

    case 'G': {  // set rotação  ex: G900
      temp = CONFIG_SERIAL.parseInt();
      temp = constrain(temp, 30, 1800);
      ROTATION_DEG = (int16_t)temp;
      ROTATION_MAX = (int32_t)CPR * ROTATION_DEG / 360;
      ROTATION_MID = ROTATION_MAX >> 1;
      CONFIG_SERIAL.println(1);
      break;
    }

    case 'E': {  // effstate (efeitos desktop)
      ffb_temp = (uint8_t)constrain(CONFIG_SERIAL.parseInt(), 0, 255);
      effstate = ffb_temp;
      CONFIG_SERIAL.println(effstate, BIN);
      break;
    }

    case 'B': {  // LC_scaling — sensibilidade load cell
      ffb_temp = (uint8_t)constrain(CONFIG_SERIAL.parseInt(), 1, 255);
      LC_scaling = ffb_temp;
#ifdef USE_LOAD_CELL
      LoadCell.setCalFactor(0.25f * (float)LC_scaling);
#endif
      CONFIG_SERIAL.println(1);
      break;
    }

    case 'W':  // PWM state (não aplicável ao DD serial; compat.)
      CONFIG_SERIAL.println(0);
      break;

    case 'A':  // salva tudo em flash
      savePrefs();
      break;

    case 'Y': {  // calibração manual de pedais
      uint8_t sub = toUpper(CONFIG_SERIAL.read());
      switch (sub) {
        case 'A': temp=CONFIG_SERIAL.parseInt(); gBrake.min =(int16_t)constrain(temp,0,4095); CONFIG_SERIAL.println(1); break;
        case 'B': temp=CONFIG_SERIAL.parseInt(); gBrake.max =(int16_t)constrain(temp,0,4095); CONFIG_SERIAL.println(1); break;
        case 'C': temp=CONFIG_SERIAL.parseInt(); gAccel.min =(int16_t)constrain(temp,0,4095); CONFIG_SERIAL.println(1); break;
        case 'D': temp=CONFIG_SERIAL.parseInt(); gAccel.max =(int16_t)constrain(temp,0,4095); CONFIG_SERIAL.println(1); break;
        case 'E': temp=CONFIG_SERIAL.parseInt(); gClutch.min=(int16_t)constrain(temp,0,4095); CONFIG_SERIAL.println(1); break;
        case 'F': temp=CONFIG_SERIAL.parseInt(); gClutch.max=(int16_t)constrain(temp,0,4095); CONFIG_SERIAL.println(1); break;
        case 'G': temp=CONFIG_SERIAL.parseInt(); gHbrake.min=(int16_t)constrain(temp,0,4095); CONFIG_SERIAL.println(1); break;
        case 'H': temp=CONFIG_SERIAL.parseInt(); gHbrake.max=(int16_t)constrain(temp,0,4095); CONFIG_SERIAL.println(1); break;
        case 'R':
          CONFIG_SERIAL.print(gBrake.min);  CONFIG_SERIAL.print(' ');
          CONFIG_SERIAL.print(gBrake.max);  CONFIG_SERIAL.print(' ');
          CONFIG_SERIAL.print(gAccel.min);  CONFIG_SERIAL.print(' ');
          CONFIG_SERIAL.print(gAccel.max);  CONFIG_SERIAL.print(' ');
          CONFIG_SERIAL.print(gClutch.min); CONFIG_SERIAL.print(' ');
          CONFIG_SERIAL.print(gClutch.max); CONFIG_SERIAL.print(' ');
          CONFIG_SERIAL.print(gHbrake.min); CONFIG_SERIAL.print(' ');
          CONFIG_SERIAL.println(gHbrake.max);
          break;
        default:  CONFIG_SERIAL.println(0); break;
      }
      break;
    }

    case 'F': {  // ganhos FFB individuais
      uint8_t sub = toUpper(CONFIG_SERIAL.read());
      ffb_temp = (uint8_t)constrain(CONFIG_SERIAL.parseInt(), 0, 255);
      switch (sub) {
        case 'G': configGeneralGain  = ffb_temp; CONFIG_SERIAL.println(1); break;
        case 'C': configConstantGain = ffb_temp; CONFIG_SERIAL.println(1); break;
        case 'D': configDamperGain   = ffb_temp; CONFIG_SERIAL.println(1); break;
        case 'F': configFrictionGain = ffb_temp; CONFIG_SERIAL.println(1); break;
        case 'S': configPeriodicGain = ffb_temp; CONFIG_SERIAL.println(1); break;
        case 'M': configSpringGain   = ffb_temp; CONFIG_SERIAL.println(1); break;
        case 'I': configInertiaGain  = ffb_temp; CONFIG_SERIAL.println(1); break;
        case 'A': configCenterGain   = ffb_temp; CONFIG_SERIAL.println(1); break;
        case 'B': configStopGain     = ffb_temp; CONFIG_SERIAL.println(1); break;
        default:  CONFIG_SERIAL.println(0); break;
      }
      break;
    }

    default:
      break;
  }
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  CONFIG_SERIAL.begin(CONFIG_SERIAL_BAUD);
  delay(500);

  // Carrega parâmetros salvos em flash
  loadPrefs();

  // ADC 12-bit, 0–3.3V
  analogReadResolution(PEDAL_ADC_BITS);
  analogSetAttenuation(ADC_11db);

  // GPIOs
  pinMode(SHIFTER_UP_PIN, INPUT_PULLUP);
  pinMode(SHIFTER_DN_PIN, INPUT_PULLUP);
  // pinMode(HBRAKE_BTN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_0_PIN, INPUT_PULLUP);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);

  // Load cell
#ifdef USE_LOAD_CELL
  initLoadCell();
#endif

  // UART GD32/STM32
  STM_UART.begin(STM_UART_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);

  // Parâmetros de rotação
  ROTATION_MAX = (int32_t)CPR * ROTATION_DEG / 360;
  ROTATION_MID = ROTATION_MAX >> 1;

  // FFB
  FfbInit();

  // USB HID
  WheelFFB.begin(HID);
  HID.begin();
  USB.begin();

  gLastRefresh = micros();
  CONFIG_SERIAL.println(F("brWheel FFB ESP32-S3 OK"));
  CONFIG_SERIAL.print(F("rot="));  CONFIG_SERIAL.print(ROTATION_DEG);
  CONFIG_SERIAL.print(F(" cpr=")); CONFIG_SERIAL.print(CPR);
#ifdef USE_LOAD_CELL
  CONFIG_SERIAL.print(F(" lc=1 lcs=")); CONFIG_SERIAL.print(LC_scaling);
#else
  CONFIG_SERIAL.print(F(" lc=0"));
#endif
  CONFIG_SERIAL.println();
}

// ============================================================================
//  LOOP PRINCIPAL (500Hz)
// ============================================================================
void loop() {
  uint32_t now = micros();
  if ((now - gLastRefresh) < CONTROL_PERIOD_US) {
    // Processa serial fora do período de controle
    configCDC();
    return;
  }
  gLastRefresh = now;

  // 1) Feedback de posição do GD32
  stmEncPoll();

  // 2) Reset de posição (botão Center WheelControl)
  if (gResetPosition) {
    gResetPosition = false;
    gPosOffset    += gEncPos_f;
    gEncPos_f      = 0.0f;
    gTorqueOut     = 0;
    stmSendCmd(0, 0);
  }

  // 3) Calcula torque FFB
  s32v axis;
  axis.x    = (s32)gEncPos_f;
  t0        = millis();           // avança timer interno do ffb_pro.ino
  s32v ffbs = gFFB.CalcTorqueCommands(&axis);

  // 4) Pipeline de torque:
  //    clip → deadband → slew → envio GD32
  int16_t t16 = (int16_t)constrain((int32_t)ffbs.x, -1000L, 1000L);
  t16         = applyDeadband(t16, TORQUE_DEADBAND);
  t16         = (int16_t)constrain((int32_t)t16, (int32_t)-TORQUE_MAX, (int32_t)TORQUE_MAX);
  t16         = slewLimit(t16, gTorqueOut, TORQUE_SLEW);
  gTorqueOut  = t16;
  stmSendCmd(0, gTorqueOut);

  // 5) Pedais e botões
  readPedals();
  uint32_t buttons = readButtons();

  // 6) Monta report HID
  int32_t hidPos = 0;
  if (ROTATION_MAX > 0)
    hidPos = (int32_t)((float)axis.x / (float)ROTATION_MAX * (float)MID_REPORT_X);
  hidPos = constrain(hidPos, -(int32_t)MID_REPORT_X - 1, (int32_t)MID_REPORT_X);

  uint16_t hidX  = (uint16_t)(hidPos + MID_REPORT_X + 1);
  uint16_t hidY  = (uint16_t)((gBrake.val  << 4) & 0xFFFF);  // freio 16-bit
  uint16_t hidZ  = (uint16_t)(gAccel.val  & 0x0FFF);
  uint16_t hidRX = (uint16_t)(gClutch.val & 0x0FFF);
  uint16_t hidRY = (uint16_t)(gHbrake.val & 0x0FFF);

  WheelFFB.SendJoystickReport(hidX, hidY, hidZ, hidRX, hidRY, buttons);

  // 7) Debug 2×/segundo
  static uint32_t lastDbgMs = 0;
  uint32_t nowMs = millis();
  if ((nowMs - lastDbgMs) >= 500) {
    lastDbgMs = nowMs;
    CONFIG_SERIAL.print(F("link="));  CONFIG_SERIAL.print(gHaveEnc ? 1 : 0);
    CONFIG_SERIAL.print(F(" pos="));  CONFIG_SERIAL.print(axis.x);
    CONFIG_SERIAL.print(F(" torq=")); CONFIG_SERIAL.print(gTorqueOut);
    CONFIG_SERIAL.print(F(" acc="));  CONFIG_SERIAL.print(gAccel.val);
    CONFIG_SERIAL.print(F(" brk="));  CONFIG_SERIAL.print(gBrake.val);
    CONFIG_SERIAL.print(F(" bat="));  CONFIG_SERIAL.print(gLastFrame.batVoltage / 100.0f, 1);
    CONFIG_SERIAL.print(F("V tmp=")); CONFIG_SERIAL.print(gLastFrame.boardTemp  / 10.0f,  1);
    CONFIG_SERIAL.print(F("C spdR="));CONFIG_SERIAL.print(gLastFrame.speedR_meas);
#ifdef USE_LOAD_CELL
    CONFIG_SERIAL.print(F(" lcs="));  CONFIG_SERIAL.print(LC_scaling);
#endif
    CONFIG_SERIAL.println();
  }
}

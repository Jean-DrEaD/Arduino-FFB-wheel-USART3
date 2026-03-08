#pragma once
// =============================================================================
//  Config_ESP32S3.h — brWheel FFB ESP32-S3
//  Pinos, parâmetros e globais compartilhados com ffb_pro.ino / ffb.ino
// =============================================================================

// ---- Tipos (espelham Config.h do Pro Micro) ---------------------------------
typedef uint8_t   u8;
typedef int8_t    s8;
typedef uint16_t  u16;
typedef int16_t   s16;
typedef uint32_t  u32;
typedef int32_t   s32;
typedef float     f32;
typedef bool      b8;

typedef struct { int16_t val; int16_t min; int16_t max; } s16a;
typedef struct { int32_t val; int32_t min; int32_t max; } s32a;
typedef struct { s32 x; s32 y; } s32v;

// ---- Versão de firmware -----------------------------------------------------
#define FIRMWARE_VERSION  130   // 1.3.0

// ---- USB ConfigReport (espelha ConfigHID.h do Pro Micro) --------------------
typedef struct { uint8_t dummy; } fwOpt;
typedef struct {
  uint8_t  ReportId;
  uint16_t Rotation;
  fwOpt    fwOption;
  int32_t  Offset;
  int32_t  EncCPR;
  uint8_t  GeneralGain;
  uint8_t  ConstantGain;
  uint8_t  DamperGain;
  uint8_t  FrictionGain;
  uint8_t  PeriodicGain;
  uint8_t  SpringGain;
  uint8_t  InertiaGain;
  uint8_t  CenterGain;
  uint8_t  StopGain;
  uint16_t MaxForce;
  uint16_t MinForce;
  uint8_t  PWMstate;
  uint8_t  EFFstate;
  uint8_t  LCscale;
  boolean  Calibrate;
  uint8_t  Info;
  uint16_t Version;
} USB_ConfigReport;

// ---- Globais compartilhados com ffb_pro.ino / ffb.ino -----------------------
extern u8       effstate;
extern u8       configGeneralGain;
extern u8       configSpringGain;
extern u8       configDamperGain;
extern u8       configFrictionGain;
extern u8       configInertiaGain;
extern u8       configConstantGain;
extern u8       configPeriodicGain;
extern u8       configCenterGain;
extern u8       configStopGain;
extern int32_t  CPR;
extern int16_t  ROTATION_DEG;
extern int32_t  ROTATION_MAX;
extern int32_t  ROTATION_MID;
extern uint16_t MM_MAX_MOTOR_TORQUE;
extern uint16_t MM_MIN_MOTOR_TORQUE;
extern uint16_t TOP;
extern uint8_t  LC_scaling;

// ---- UART para STM32/GD32 ---------------------------------------------------
#define STM_UART          Serial1
#define STM_UART_BAUD     500000
#define STM_TX_PIN        17    // GPIO17 → PB11 do GD32 (USART3 RX)
#define STM_RX_PIN        18    // GPIO18 → PB10 do GD32 (USART3 TX)

// ---- Debug ------------------------------------------------------------------
#define CONFIG_SERIAL       Serial
#define DEBUG_SERIAL        Serial
#define CONFIG_SERIAL_BAUD  115200

// ---- Pedais (ADC1, 12-bit, máx 3.3V) ----------------------------------------
#define ACCEL_PIN        1     // GPIO1  — acelerador
#define BRAKE_PIN        2     // GPIO2  — freio analógico (ignorado com USE_LOAD_CELL)
#define CLUTCH_PIN       3     // GPIO3  — embreagem (opcional)
#define HBRAKE_AXIS_PIN  4     // GPIO4  — freio de mão analógico (opcional)
#define PEDAL_ADC_BITS   12
#define PEDAL_ADC_MAX    4095

#define ACCEL_MIN_DEFAULT    100
#define ACCEL_MAX_DEFAULT   3900
#define BRAKE_MIN_DEFAULT    100
#define BRAKE_MAX_DEFAULT   3900
#define CLUTCH_MIN_DEFAULT   100
#define CLUTCH_MAX_DEFAULT  3900

// ---- Load Cell (HX711) -------------------------------------------------------
// Descomente USE_LOAD_CELL para usar load cell no lugar do potenciômetro de freio.
// Biblioteca: "HX711_ADC" por Olav Kallhovd (Gerenciador de Bibliotecas Arduino).
//
// Fiação:
//   HX711 DOUT → LC_DOUT_PIN (GPIO11)
//   HX711 SCK  → LC_SCK_PIN  (GPIO12)
//   HX711 VCC  → 3.3V ou 5V (verifique o módulo)
//   HX711 GND  → GND
//   Célula de carga → HX711 E+/E-/A+/A-
//
// Calibração:
//   LC_scaling (1–255) ajusta a sensibilidade via comando serial 'B'.
//   calFactor = 0.25 × LC_scaling  →  padrão 128 → calFactor = 32.0
//   Aumentar LC_scaling = pedal mais "leve" (mais força ADC por Newton).
//   Ajuste até o freio atingir máximo com a força desejada.
//#define USE_LOAD_CELL

#define LC_DOUT_PIN         11   // GPIO11 — HX711 data out
#define LC_SCK_PIN          12   // GPIO12 — HX711 clock
#define LC_GAIN            128   // 128 ou 64 (canal A) | 32 (canal B)
#define LC_SCALING_DEFAULT 128   // calFactor = 0.25 × 128 = 32.0

// ---- Câmbio sequencial (INPUT_PULLUP, NA) ------------------------------------
#define SHIFTER_UP_PIN  38    // GPIO38 — gear up
#define SHIFTER_DN_PIN  39    // GPIO39 — gear down
#define HBRAKE_BTN_PIN  40    // GPIO40 — freio de mão digital (descomentar abaixo)

// ---- Botões extras do volante -----------------------------------------------
#define BUTTON_0_PIN    5
#define BUTTON_1_PIN    6
#define BUTTON_2_PIN    7
#define BUTTON_3_PIN    8

// Bits no campo buttons HID
#define SHIFTER_UP_BIT  12
#define SHIFTER_DN_BIT  13
#define HBRAKE_BTN_BIT  14

// ---- Loop de controle -------------------------------------------------------
#define CONTROL_PERIOD_US  2000   // 2ms = 500Hz

// ---- Torque -----------------------------------------------------------------
#define TORQUE_MAX_DEFAULT   700
#define TORQUE_SLEW_DEFAULT  150
#define TORQUE_DEADBAND        5   // ignora ruído < 5 unidades antes de enviar ao GD32

// ---- Encoder / Volante ------------------------------------------------------
#define ENCODER_CPR_DEFAULT   4096
#define ROTATION_DEG_DEFAULT   900

// ---- Auto-center ------------------------------------------------------------
#define AUTO_CENTER_SETTLE_MS  4200
#define AUTO_CENTER_DEADBAND      3

// ---- Ganhos FFB padrão ------------------------------------------------------
#define DEFAULT_GENERAL_GAIN    100
#define DEFAULT_SPRING_GAIN      50
#define DEFAULT_DAMPER_GAIN     100
#define DEFAULT_FRICTION_GAIN   100
#define DEFAULT_INERTIA_GAIN    100
#define DEFAULT_CONSTANT_GAIN   100
#define DEFAULT_PERIODIC_GAIN   100
#define DEFAULT_CENTER_GAIN      50
#define DEFAULT_STOP_GAIN       100

// ---- Macros de calibração (no-op no ESP32) ----------------------------------
#define cal_print(s)
#define cal_println(s)

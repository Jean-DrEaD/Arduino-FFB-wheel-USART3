#pragma once
// =============================================================================
//  brWheel FFB — ESP32-S3 Zero / Mini
//  Config_ESP32S3.h — substitui Config.h e ConfigHID.h do Pro Micro
//
//  Requer: arduino-esp32 >= 3.0
//  Board: "ESP32S3 Dev Module" | USB Mode: "USB-OTG (TinyUSB)" | USB CDC On Boot: Disabled
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
typedef struct { int32_t val; int16_t min; int16_t max; } s32a;
typedef struct { s32 x; s32 y; } s32v;

// ---- Struct USB_ConfigReport (espelha ConfigHID.h) --------------------------
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

// ---- Globais compartilhados com ffb.h / ffb_pro.ino -------------------------
// Declarados aqui, definidos em brWheel_ESP32S3.ino
extern u8      effstate;           // bits: 0=autospring 1=damper 2=inertia 3=friction 4=monitor
extern u8      configGeneralGain;
extern u8      configSpringGain;
extern u8      configDamperGain;
extern u8      configFrictionGain;
extern u8      configInertiaGain;
extern u8      configConstantGain;
extern u8      configPeriodicGain;
extern u8      configCenterGain;
extern u8      configStopGain;
extern int32_t CPR;                // encoder counts per revolution
extern int16_t ROTATION_DEG;       // graus de rotação total (ex: 900)
extern int32_t ROTATION_MAX;       // CPR * ROTATION_DEG / 360
extern int32_t ROTATION_MID;       // ROTATION_MAX / 2
extern uint16_t MM_MAX_MOTOR_TORQUE;
extern uint16_t MM_MIN_MOTOR_TORQUE;
extern uint16_t TOP;               // escala PWM / torque top = 1000 para DD serial

// ---- UART para STM32/GD32 ---------------------------------------------------
#define STM_UART          Serial1
#define STM_UART_BAUD     500000
#define STM_TX_PIN        17    // GPIO17 → RX do GD32 (USART3 PB11)
#define STM_RX_PIN        18    // GPIO18 → TX do GD32 (USART3 PB10)

// ---- Debug ------------------------------------------------------------------
#define CONFIG_SERIAL      Serial
#define DEBUG_SERIAL       Serial
#define CONFIG_SERIAL_BAUD 115200

// ---- Pedais (ADC1, 12-bit, max 3.3V) ----------------------------------------
#define ACCEL_PIN        1     // GPIO1  — acelerador
#define BRAKE_PIN        2     // GPIO2  — freio
#define CLUTCH_PIN       3     // GPIO3  — embreagem (opcional)
#define HBRAKE_AXIS_PIN  4     // GPIO4  — freio de mão analógico (opcional)
#define PEDAL_ADC_BITS   12
#define PEDAL_ADC_MAX    4095

#define ACCEL_MIN_DEFAULT   100
#define ACCEL_MAX_DEFAULT   3900
#define BRAKE_MIN_DEFAULT   100
#define BRAKE_MAX_DEFAULT   3900
#define CLUTCH_MIN_DEFAULT  100
#define CLUTCH_MAX_DEFAULT  3900

// ---- Câmbio sequencial (INPUT_PULLUP, NA) -----------------------------------
#define SHIFTER_UP_PIN  38
#define SHIFTER_DN_PIN  39
#define HBRAKE_BTN_PIN  40     // digital (descomentar em .ino)

// ---- Botões extras ----------------------------------------------------------
#define BUTTON_0_PIN    5
#define BUTTON_1_PIN    6
#define BUTTON_2_PIN    7
#define BUTTON_3_PIN    8

// Mapa de bits no campo buttons HID (32 bits)
// bit 0..3: hat switch | bit 4..11: botões volante | bit 12..14: shifter/hbrake
#define SHIFTER_UP_BIT  12
#define SHIFTER_DN_BIT  13
#define HBRAKE_BTN_BIT  14

// ---- Loop de controle -------------------------------------------------------
#define CONTROL_PERIOD_US 2000   // 2ms = 500Hz

// ---- FFB / Torque -----------------------------------------------------------
#define TORQUE_MAX_DEFAULT  700   // clip final enviado ao GD32 (±1000 escala STM)
#define TORQUE_SLEW_DEFAULT 150   // variação máxima por ciclo 2ms

// ---- Encoder / Volante (defaults, overwrite via Serial/WheelControl) --------
#define ENCODER_CPR_DEFAULT   4096   // MT6701 @ 1024PPR × 4
#define ROTATION_DEG_DEFAULT  900    // ±450°

// ---- Auto-center (via alinhamento GD32, não calibração mecânica) ------------
#define AUTO_CENTER_SETTLE_MS  4200
#define AUTO_CENTER_DEADBAND   3

// ---- Ganhos FFB defaults ----------------------------------------------------
#define DEFAULT_GENERAL_GAIN    100
#define DEFAULT_SPRING_GAIN      50
#define DEFAULT_DAMPER_GAIN     100
#define DEFAULT_FRICTION_GAIN   100
#define DEFAULT_INERTIA_GAIN    100
#define DEFAULT_CONSTANT_GAIN   100
#define DEFAULT_PERIODIC_GAIN   100
#define DEFAULT_CENTER_GAIN      50
#define DEFAULT_STOP_GAIN       100

// ---- Macros de debug de calibração (no-op no ESP32) -------------------------
#define cal_print(s)
#define cal_println(s)

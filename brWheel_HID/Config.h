#ifndef _CONFIG_H_
#define _CONFIG_H_


//------------------------------------- Firmware options -------------------------------------------------

#define USE_CONFIGCDC      // milos, use virtual RS232 serial port for configuring firmware settings
//#define USE_CONFIGHID      // milos, use native USB HID protocol for configuring firmware settings (not implemented fully yet)
//#define USE_VNH5019        // Pololu dual 24V DC motor drive (partially inplemented)
//#define USE_SM_RS485       // Granite devices simple motion protocol (not implemented yet)
//#define USE_LCD            // milos, LCD via i2C (not implemented yet)
//#define USE_ADS1015        // milos, uncomment for 12bit pedals, when commented out it's 10bit from arduino inputs (can not be used with AVG_INPUTS)
//#define USE_DSP56ADC16S    // 16 bits Stereo ADC (milos, not implemented fully yet)
//#define USE_QUADRATURE_ENCODER // milos, optical quadrature encoder (you may comment it out)
//#define USE_TWOFFBAXIS     // milos, uncomment to enable 2nd FFB axis and PWM/DAC output
//#define USE_AS5600         // milos, magnetic encoder via i2C instead of optical encoder
//#define USE_TCA9548        // milos, i2C multiplexer chip
//#define USE_ZINDEX         // milos, Z-index encoder channel
//#define USE_LOAD_CELL      // Load cell shield
//#define USE_SHIFT_REGISTER // shift registers for buttons
//#define USE_DUAL_SHIFT_REGISTER
//#define USE_SN74ALS166N
//#define USE_XY_SHIFTER
//#define USE_HATSWITCH
//#define USE_BTNMATRIX
//#define AVG_INPUTS
//#define USE_AUTOCALIB
//#define USE_SPLITAXIS
//#define USE_CENTERBTN
//#define USE_EXTRABTN
//#define USE_MCP4725
//#define USE_ANALOGFFBAXIS

#define USE_PROMICRO       // <<< CORREÇÃO: você está usando Pro Micro
#define USE_EEPROM         // milos, enable loading/saving settings from EEPROM

#define CALIBRATE_AT_INIT 0 // milos, was 1

//------------------------------------- Pins -------------------------------------------------------------

// milos, added - ffb clip LED indicator
#define FFBCLIP_LED_PIN 13 // only for leonardo/micro

#ifdef USE_PROMICRO
#ifndef USE_MCP4725
#ifndef USE_ADS1015
#ifndef USE_AS5600
#define FFBCLIP_LED_PIN 3 // for ProMicro if no above i2C devices
#endif // end of as5600
#endif // end of ads1015
#endif // end of mcp4725
#endif // end of promicro

#define ACCEL_PIN     A0
#ifdef USE_LOAD_CELL
#define CLUTCH_PIN    A1
#define HBRAKE_PIN    A2
#else
#define BRAKE_PIN     A1
#define CLUTCH_PIN    A2
#define HBRAKE_PIN    A3
#endif

// milos, added - for alternate button0 options
#ifdef USE_LOAD_CELL
#define BUTTON0 A3
#define B0PORTBIT 4
#else
#define BUTTON0 4
#define B0PORTBIT 4
#define BUTTON7 5
#define B7PORTBIT 6
#endif

#ifdef USE_SHIFT_REGISTER
#define SHIFTREG_PL 8
#define SHIFTREG_CLK 7
#define SHIFTREG_DATA_SW 6
#define SHIFTS_NUM 33
#ifdef USE_SN74ALS166N
#ifndef USE_XY_SHIFTER
#undef SHIFTS_NUM
#define SHIFTS_NUM 49
#endif
#endif
#else
#define BUTTON4 6
#define B4PORTBIT 7
#define BUTTON5 7
#define B5PORTBIT 6
#define BUTTON6 8
#define B6PORTBIT 4
#endif

#ifndef USE_PROMICRO
#ifndef USE_XY_SHIFTER
#define BUTTON1 A4
#define B1PORTBIT 1
#define BUTTON2 A5
#define B2PORTBIT 0
#else
#define SHIFTER_X_PIN A4
#define SHIFTER_Y_PIN A5
#ifdef USE_HATSWITCH
#undef BUTTON0
#undef B0PORTBIT
#undef BUTTON4
#undef B4PORTBIT
#define BUTTON0 5
#define B0PORTBIT 6
#define BUTTON4 4
#define B4PORTBIT 4
#endif
#define BUTTON1 6
#define B1PORTBIT 7
#define BUTTON2 7
#define B2PORTBIT 6
#endif
#define BUTTON3 12
#define B3PORTBIT 6
#else // Pro Micro
#ifdef USE_HATSWITCH
#undef BUTTON0
#undef B0PORTBIT
#undef BUTTON4
#undef B4PORTBIT
#undef BUTTON7
#undef B7PORTBIT
#define BUTTON0 5
#define B0PORTBIT 6
#define BUTTON4 4
#define B4PORTBIT 4
#define BUTTON7 6
#define B7PORTBIT 7
#endif
#define BUTTON1 14
#define B1PORTBIT 3
#define BUTTON2 15
#define B2PORTBIT 1
#ifndef USE_CENTERBTN
#define BUTTON3 2
#define B3PORTBIT 1
#else
#define BUTTON3 3
#define B3PORTBIT 0
#endif
#endif // end of promicro

#ifdef USE_EXTRABTN
#define BUTTON8 A2
#define B8PORTBIT 5
#define BUTTON9 A3
#define B9PORTBIT 4
#endif

#ifdef USE_DSP56ADC16S
#define ADC_PIN_CLKIN   5
#define ADC_PIN_FSO     3
#define ADC_PIN_SDO     13
#define ADC_PIN_SCO     11
#define PIN_MISO        12
#endif

#define PWM_PIN_L     9
#define PWM_PIN_R     10
#ifdef USE_TWOFFBAXIS
#define PWM_PIN_U     11
#define PWM_PIN_D     5
#endif

#ifndef USE_PROMICRO
#define DIR_PIN       11
#else
#define DIR_PIN       16
#endif

#define ACCEL_INPUT 0
#ifdef USE_LOAD_CELL
#define CLUTCH_INPUT 1
#define HBRAKE_INPUT 2
#ifdef USE_XY_SHIFTER
#define SHIFTER_X_INPUT 3
#define SHIFTER_Y_INPUT 4
#endif
#else
#define BRAKE_INPUT 1
#define CLUTCH_INPUT 2
#define HBRAKE_INPUT 3
#ifdef USE_XY_SHIFTER
#define SHIFTER_X_INPUT 4
#define SHIFTER_Y_INPUT 5
#endif
#endif

uint8_t LC_scaling;

//------------------------------------- EEPROM Config -----------------------------------------------------

#define PARAM_ADDR_FW_VERSION    0x00
#define PARAM_ADDR_ENC_OFFSET    0x02
#define PARAM_ADDR_ROTATION_DEG  0x06
#define PARAM_ADDR_GEN_GAIN      0x08
#define PARAM_ADDR_DMP_GAIN      0x09
#define PARAM_ADDR_FRC_GAIN      0x0A
#define PARAM_ADDR_CNT_GAIN      0x0B
#define PARAM_ADDR_PER_GAIN      0x0C
#define PARAM_ADDR_SPR_GAIN      0x0D
#define PARAM_ADDR_INR_GAIN      0x0E
#define PARAM_ADDR_CTS_GAIN      0x0F
#define PARAM_ADDR_STP_GAIN      0x10
#define PARAM_ADDR_MIN_TORQ      0x11
#define PARAM_ADDR_MAX_TORQ      0x13
#define PARAM_ADDR_MAX_DAC       0x15
#define PARAM_ADDR_BRK_PRES      0x17
#define PARAM_ADDR_DSK_EFFC      0x18
#define PARAM_ADDR_ENC_CPR       0x19
#define PARAM_ADDR_PWM_SET       0x1D
#define PARAM_ADDR_SHFT_X0       0x1E
#define PARAM_ADDR_SHFT_X1       0x20
#define PARAM_ADDR_SHFT_X2       0x22
#define PARAM_ADDR_SHFT_Y0       0x24
#define PARAM_ADDR_SHFT_Y1       0x26
#define PARAM_ADDR_SHFT_CFG      0x28
#define PARAM_ADDR_ACEL_LO       0x2A
#define PARAM_ADDR_ACEL_HI       0x2C
#define PARAM_ADDR_BRAK_LO       0x2E
#define PARAM_ADDR_BRAK_HI       0x30
#define PARAM_ADDR_CLUT_LO       0x32
#define PARAM_ADDR_CLUT_HI       0x34
#define PARAM_ADDR_HBRK_LO       0x36
#define PARAM_ADDR_HBRK_HI       0x38

#define FIRMWARE_VERSION         0xFA

#define GetParam(m_offset,m_data) getParam((m_offset),(u8*)&(m_data),sizeof(m_data))
#define SetParam(m_offset,m_data) setParam((m_offset),(u8*)&(m_data),sizeof(m_data))

//------------------------------------- Main Config -----------------------------------------------------

#define CONTROL_PERIOD        2000
#define CONFIG_SERIAL_PERIOD  10000

//------------------------------------- FFB/Firmware config -----------------------------------------------------

typedef struct fwOpt {
  boolean a = false;
  boolean b = false;
  boolean c = false;
  boolean d = false;
  boolean e = false;
  boolean f = false;
  boolean g = false;
  boolean h = false;
  boolean i = false;
  boolean k = false;
  boolean l = false;
  boolean m = false;
  boolean n = false;
  boolean p = false;
  boolean r = false;
  boolean s = false;
  boolean t = false;
  boolean u = false;
  boolean w = false;
  boolean x = false;
  boolean z = false;
} fwOpt;

void update(fwOpt *option) {
#ifdef USE_AUTOCALIB
  option->a = true;
#endif
#ifdef USE_TWOFFBAXIS
  option->b = true;
#endif
#ifdef USE_CENTERBTN
  option->c = true;
#endif
#ifndef USE_QUADRATURE_ENCODER
  option->d = true;
#endif
#ifdef USE_EXTRABTN
  option->e = true;
#endif
#ifdef USE_XY_SHIFTER
  option->f = true;
#endif
#ifdef USE_MCP4725
  option->g = true;
#endif
#ifdef USE_HATSWITCH
  option->h = true;
#endif
#ifdef AVG_INPUTS
  option->i = true;
#endif
#ifdef USE_SPLITAXIS
  option->k = true;
#endif
#ifdef USE_LOAD_CELL
  option->l = true;
#endif
#ifdef USE_PROMICRO
  option->m = true;
#endif
#ifndef USE_EEPROM
  option->p = true;
#endif
#ifdef USE_SHIFT_REGISTER
  option->n = true;
#endif
#ifdef USE_SN74ALS166N
  option->r = true;
#endif
#ifdef USE_ADS1015
  option->s = true;
#endif
#ifdef USE_BTNMATRIX
  option->t = true;
#endif
#ifdef USE_TCA9548
  option->u = true;
#endif
#ifdef USE_AS5600
  option->w = true;
#endif
#ifdef USE_ANALOGFFBAXIS
  option->x = true;
#endif
#ifdef USE_ZINDEX
  option->z = true;
#endif
}

// milos, loaded from EEPROM
u8 effstate;
u8 pwmstate;

u8 configGeneralGain;
u8 configDamperGain;
u8 configFrictionGain;
u8 configConstantGain;
u8 configPeriodicGain;
u8 configSpringGain;
u8 configInertiaGain;
u8 configCenterGain;
u8 configStopGain;

uint16_t PWMtops [13] = {
  400,800,1023,2047,4095,5000,10000,16383,20000,32767,30000,40000,65535
};

int16_t ROTATION_DEG;
int32_t CPR;
int32_t ROTATION_MAX;
int32_t ROTATION_MID;
uint16_t MM_MIN_MOTOR_TORQUE;
uint16_t MM_MAX_MOTOR_TORQUE;
uint16_t MAX_DAC;
uint16_t TOP;

uint16_t calcTOP(byte b) {
#ifndef USE_MCP4725
  byte j = 0;
  for (uint8_t i = 0; i < 4; i++) bitWrite(j, i, bitRead(b, i + 2));
  if (bitRead(b, 0)) return (PWMtops[j] >> 1);
  return PWMtops[j];
#else
  return (MAX_DAC);
#endif
}

typedef struct s32v {
  s32 x;
#ifdef USE_TWOFFBAXIS
  s32 y;
#endif
} s32v;

f32 FFB_bal;
f32 L_bal;
f32 R_bal;
f32 minTorquePP;

f32 RCM_min = 1.0;
f32 RCM_zer = 1.5;
f32 RCM_max = 2.0;

f32 RCMscaler(byte value) {
  if (bitRead(value, 0)) return 1000.0;
  return 2000.0;
}

boolean zIndexFound = false;

typedef struct s16a {
  int16_t val;
  int16_t min;
  int16_t max;
} s16a;

typedef struct s32a {
  int32_t val;
  int16_t min;
  int16_t max;
} s32a;

const uint8_t avgSamples = 4;

#ifdef AVG_INPUTS
const uint16_t maxCal = 4095;
#else
#ifdef USE_ADS1015
const uint16_t maxCal = 2047;
#else
const uint16_t maxCal = 1023;
#endif
#endif

#ifdef USE_TCA9548
#include <Wire.h>
#define baseTCA0 0x70
void TcaChannelSel(uint8_t addr, uint8_t ch) {
  Wire.beginTransmission(addr);
  Wire.write(1 << ch);
  Wire.endTransmission();
}
#endif

#endif // _CONFIG_H_


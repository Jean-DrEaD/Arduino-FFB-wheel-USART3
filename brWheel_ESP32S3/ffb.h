/*
  Force Feedback Joystick
  USB HID descriptors for a force feedback joystick.

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  Copyright 2018-2025  Milos Rankovic (ranenbg [at] gmail [dot] com)
  MIT License.
*/

#ifndef _FFB_
#define _FFB_

#include <arduino.h>

// ESP32-S3: usa Config_ESP32S3.h; AVR/Pro Micro: usa Config.h
#ifdef ESP32
  #include "Config_ESP32S3.h"
#else
  #include "Config.h"
#endif

/* Type Defines: */

// Maximum number of parallel effects in memory
#define MAX_EFFECTS 11 //milos, changed from 20

// milos, this will increment in each cycle by 2ms, 500Hz FFB effects calculation
u32 t0 = 0; //milos, added
u32 effectTime[MAX_EFFECTS]; //milos, added
bool t0_updated = false; //milos, added

// ---- Input

typedef struct
{
  uint8_t	reportId;	// =2
  uint8_t	status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
  uint8_t	effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)
} USB_FFBReport_PIDStatus_Input_Data_t;

// ---- Output

typedef struct
{ // FFB: Set Effect Output Report
  uint8_t	reportId;	// =1
  uint8_t	effectBlockIndex;	// 1..40
  uint8_t	effectType;	// 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28) //milos, total 11, 28 is removed (custom force)
  uint16_t duration; // 0..65535, exp -3, s
  uint16_t triggerRepeatInterval; // 0..65535, exp -3, s // milos, do not comment out this (ffb stops working)
  int16_t gain; // 0..32767  (physical 0..32767) //milos, was 0(0)..(255)10000, uint8_t
  uint8_t	triggerButton;	// button ID (0..8) // milos, do not comment out this (ffb stops working)
  uint8_t	enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
  uint16_t direction; // angle (0=0 .. 32767=35999, exp -2, deg) //milos, 16bit
  uint16_t startDelay;	// 0..65535, exp -3, s //milos, uncommented
} USB_FFBReport_SetEffect_Output_Data_t;

typedef struct
{ // FFB: Set Envelope Output Report
  uint8_t	reportId;	// =2
  uint8_t	effectBlockIndex;	// 1..40
  uint8_t attackLevel; // 0..255  (physical 0..32767) //milos, was 10000
  uint8_t	fadeLevel; // 0..255  (physical 0..32767) //milos, was 10000
  uint16_t attackTime;	// 0..32767  (physical 0..32767), exp -3, s
  uint16_t fadeTime;	// 0..32767  (physical 0..32767), exp -3, s
} USB_FFBReport_SetEnvelope_Output_Data_t;

typedef struct
{ // FFB: Set Condition Output Report
  uint8_t	reportId;	// =3
  uint8_t	effectBlockIndex;	// 1..40
  uint8_t	parameterBlockOffset;	// bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
  int16_t cpOffset;	// -32768..32767
  int16_t	positiveCoefficient;	// -32767..32767
  uint8_t deadBand;	// 0..255
} USB_FFBReport_SetCondition_Output_Data_t;

typedef struct
{ // FFB: Set Periodic Output Report
  uint8_t	reportId;	// =4
  uint8_t	effectBlockIndex;	// 1..40
  int16_t magnitude; // 0..32767
  int16_t offset; // -32768..32767
  uint8_t phase;  // 0..255 (physical 0..359 deg)
  uint16_t period;	// 0..65535, exp -3, s
} USB_FFBReport_SetPeriodic_Output_Data_t;

typedef struct
{ // FFB: Set ConstantForce Output Report
  uint8_t	reportId;	// =5
  uint8_t	effectBlockIndex;	// 1..40
  int16_t magnitude;	// -32767..32737
} USB_FFBReport_SetConstantForce_Output_Data_t;

typedef struct
{ // FFB: Set RampForce Output Report
  uint8_t	reportId;	// =6
  uint8_t	effectBlockIndex;	// 1..40
  int8_t rampStart; // -127..127
  int8_t rampEnd; // -127..127
} USB_FFBReport_SetRampForce_Output_Data_t;

typedef struct
{ // FFB: Set EffectOperation Output Report
  uint8_t	reportId;	// =10
  uint8_t effectBlockIndex;	// 1..40
  uint8_t operation; // 1=Start, 2=StartSolo, 3=Stop
  uint8_t	loopCount; //0..255
} USB_FFBReport_EffectOperation_Output_Data_t;

typedef struct
{ // FFB: Block Free Output Report
  uint8_t	reportId;	// =11
  uint8_t effectBlockIndex;	// 1..40
} USB_FFBReport_BlockFree_Output_Data_t;

typedef struct
{ // FFB: Device Control Output Report
  uint8_t	reportId;	// =12
  uint8_t control;
} USB_FFBReport_DeviceControl_Output_Data_t;

typedef struct
{ // FFB: DeviceGain Output Report
  uint8_t	reportId;	// =13
  uint8_t deviceGain; //0..255
} USB_FFBReport_DeviceGain_Output_Data_t;

// ---- Features

typedef struct
{ // FFB: Create New Effect Feature Report
  uint8_t reportId;	// =1
  uint8_t	effectType;	// Enum (1..12)
  uint16_t byteCount;	// 0..511
} USB_FFBReport_CreateNewEffect_Feature_Data_t;

typedef struct
{ // FFB: PID Block Load Feature Report
  uint8_t	reportId;	// =2
  uint8_t effectBlockIndex;	// 1..40
  uint8_t	loadStatus;	// 1=Success,2=Full,3=Error
  uint16_t ramPoolAvailable;
} USB_FFBReport_PIDBlockLoad_Feature_Data_t;

typedef struct
{ // FFB: PID Pool Feature Report
  uint8_t	reportId;	// =3
  uint16_t ramPoolSize;
  uint8_t	maxSimultaneousEffects;
  uint8_t	memoryManagement;
} USB_FFBReport_PIDPool_Feature_Data_t;

extern const uint16_t OutReportSize[];

void FfbSetDriver(uint8_t id);
void FfbInit(void);  // ESP32: substituto de FfbSetDriver(0)
void FfbOnUsbData(uint8_t *data, uint16_t len);
void FfbOnCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData);
void FfbOnPIDPool(USB_FFBReport_PIDPool_Feature_Data_t *data);
void WaitMs(int ms);
void FfbSendData(const uint8_t *data, uint16_t len);
void FfbSendPackets(const uint8_t *data, uint16_t len);
uint8_t FfbDebugListEffects(uint8_t *index);

typedef struct
{
  uint8_t midi;
  uint8_t springs;
  uint8_t constants;
  uint8_t triangles;
  uint8_t sines;
  uint8_t effectId[MAX_EFFECTS];
} TDisabledEffectTypes;

extern volatile TDisabledEffectTypes gDisabledEffects;

void FfbEnableSprings(uint8_t inEnable);
void FfbEnableConstants(uint8_t inEnable);
void FfbEnableTriangles(uint8_t inEnable);
void FfbEnableSines(uint8_t inEnable);
void FfbEnableEffectId(uint8_t inId, uint8_t inEnable);

// Bit-masks for effect states
#define MEffectState_Free       0x00
#define MEffectState_Allocated  0x01
#define MEffectState_Playing    0x02

#define USB_DURATION_INFINITE	0xFFFF

#define USB_EFFECT_CONSTANT		0x01
#define USB_EFFECT_RAMP			0x02
#define USB_EFFECT_SQUARE 		0x03
#define USB_EFFECT_SINE 		0x04
#define USB_EFFECT_TRIANGLE		0x05
#define USB_EFFECT_SAWTOOTHDOWN	0x06
#define USB_EFFECT_SAWTOOTHUP	0x07
#define USB_EFFECT_SPRING		0x08
#define USB_EFFECT_DAMPER		0x09
#define USB_EFFECT_INERTIA		0x0A
#define USB_EFFECT_FRICTION		0x0B
#define USB_EFFECT_CUSTOM		0x0C
#define USB_EFFECT_PERIODIC		0x0D

typedef struct
{
  u8 state;
  u8 type;
  u8 parameterBlockOffset;
  u8 attackLevel, fadeLevel, deadBand, enableAxis;
  s8 rampStart, rampEnd;
  u16 gain, period, direction;
  u16 duration, fadeTime, attackTime, startDelay;
  s16 magnitude, positiveCoefficient;
  s16 offset;
  u8 phase;
#ifdef USE_TWOFFBAXIS
  u8 deadBand2;
  s16 magnitude2, offset2;
#endif
} TEffectState;

typedef struct
{
  void (*EnableInterrupts)(void);
  const uint8_t* (*GetSysExHeader)(uint8_t* hdr_len);
  void (*SetAutoCenter)(uint8_t enable);
  void (*StartEffect)(uint8_t eid);
  void (*StopEffect)(uint8_t eid);
  void (*FreeEffect)(uint8_t eid);
  void (*ModifyDuration)(uint8_t effectId, uint16_t duration);
  void (*CreateNewEffect)(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, volatile TEffectState* effect);
  void (*SetEnvelope)(USB_FFBReport_SetEnvelope_Output_Data_t* data, volatile TEffectState* effect);
  void (*SetCondition)(USB_FFBReport_SetCondition_Output_Data_t* data, volatile TEffectState* effect);
  void (*SetPeriodic)(USB_FFBReport_SetPeriodic_Output_Data_t* data, volatile TEffectState* effect);
  void (*SetConstantForce)(USB_FFBReport_SetConstantForce_Output_Data_t* data, volatile TEffectState* effect);
  void (*SetRampForce)(USB_FFBReport_SetRampForce_Output_Data_t* data, volatile TEffectState* effect);
  uint8_t (*SetEffect)(USB_FFBReport_SetEffect_Output_Data_t* data, volatile TEffectState* effect);
} FFB_Driver;

#endif // _FFB_

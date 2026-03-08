/*
  Force Feedback Wheel — ffb_pro.h
  Copyright 2012-2025: Tero Loimunema, Saku Kekkonen, Etienne Saint-Paul,
                       Fernando Igor, Milos Rankovic (ranenbg)
  ESP32 guard: Jean-DrEaD, 2026
*/

#ifndef _FFB_PRO_
#define _FFB_PRO_

#include <stdint.h>
#include "ffb.h"

const s32 SPD_THRESHOLD = 0;
const s32 ACL_THRESHOLD = 0; //milos, added
const s32 FRC_THRESHOLD = 1; //milos, added

#define ADC_NB_BITS  10
#define VAL_NB_BITS  16
#define MAX_VAL      ((1<<VAL_NB_BITS)-1)
#define MID_VAL      (1<<(VAL_NB_BITS-1))

#define MID_REPORT_X  (X_AXIS_PHYS_MAX >> 1) // milos, 32767
#define MID_REPORT_Y  (Y_AXIS_PHYS_MAX >> 1)
#define MID_REPORT_Z  (Z_AXIS_PHYS_MAX >> 1)

#define CALIBRATING_LEFT    0x0
#define CALIBRATING_RIGHT   0x1
#define CALIBRATING_INDEX   0x2
#define CALIBRATING_HOMING  0x3
#define CALIBRATION_ERROR   0x4
#define CALIBRATION_DONE    0xFF

const u8 NB_TAPS   = 10; //milos, was 9
const u8 NB_TAPS_A = 20; //milos, added

void FfbproSetAutoCenter(uint8_t enable);
void FfbproEnableInterrupts(void);
const uint8_t* FfbproGetSysExHeader(uint8_t* hdr_len);

void FfbproStartEffect(uint8_t id);
void FfbproStopEffect(uint8_t id);
void FfbproFreeEffect(uint8_t id);

void FfbproModifyDuration(uint8_t effectId, uint16_t duration, uint16_t stdelay);

void FfbproSetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetCondition(USB_FFBReport_SetCondition_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetRampForce(USB_FFBReport_SetRampForce_Output_Data_t* data, volatile TEffectState* effect);
uint8_t FfbproSetEffect(USB_FFBReport_SetEffect_Output_Data_t* data, volatile TEffectState* effect);
void FfbproCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, volatile TEffectState* effect);

class cSpeedObs {
  public:
    cSpeedObs() { Init(); }
    void Init();
    f32 Update(s32 new_pos);
    s32 mLastPos;
    f32 mLastSpeed;
    b8  mLastValueValid;
    f32 mLastSpeeds[NB_TAPS];
    u8  mCurrentLS;
};

class cAccelObs {
  public:
    cAccelObs() { Init(); }
    void Init();
    f32 Update(f32 new_spd);
    f32 mLastSpd;
    f32 mLastAccel;
    b8  mLastValueValid;
    f32 mLastAccels[NB_TAPS_A];
    u8  mCurrentLA;
};

class cFFB {
  public:
    cFFB();
    s32v CalcTorqueCommands(s32v *pos);
    cSpeedObs mSpeed;
    cAccelObs mAccel;
    b8 mAutoCenter;
};

class BRFFB {
  public:
    BRFFB();
    void calibrate();
    s32 offset;
    b8  state;
};

// Instâncias globais — definidas em brWheel_ESP32S3.ino
extern cFFB  gFFB;
extern BRFFB brWheelFFB;

#endif // _FFB_PRO_

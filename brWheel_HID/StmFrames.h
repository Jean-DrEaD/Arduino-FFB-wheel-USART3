#ifndef _STM_FRAMES_H_
#define _STM_FRAMES_H_

#include <stdint.h>

// Espelho EXATO do SerialFeedback (Src/main.c linha 159-169)
typedef struct __attribute__((packed)) {
  uint16_t start;        // 0xABCD
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;     // start^cmd1^cmd2^speedR^speedL^batVoltage^boardTemp^cmdLed
} StmEncFrame;           // 18 bytes

// Espelho EXATO do SerialCommand (Src/util.c linha 2182)
typedef struct __attribute__((packed)) {
  uint16_t start;        // 0xABCD
  int16_t  steer;        // -1000..1000
  int16_t  speed;        // -1000..1000
  uint16_t checksum;     // start ^ steer ^ speed
} StmCmdFrame;           // 8 bytes

#endif // _STM_FRAMES_H_


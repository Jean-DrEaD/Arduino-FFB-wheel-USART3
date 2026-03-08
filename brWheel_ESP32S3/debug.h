/*
  Force Feedback Joystick — debug.h
  Original: Copyright 2012 Tero Loimuneva (tloimu@gmail.com)
  ESP32 guard: Jean-DrEaD, 2026
*/
#ifndef _DEBUG_H_
#define _DEBUG_H_

#ifdef ESP32

// No-ops: mantém compatibilidade com chamadas em ffb.ino / ffb_pro.ino
// sem gerar código ou overhead
#define LogText(x)
#define LogTextLf(x)
#define LogTextP(x)
#define LogTextLfP(x)
#define LogBinary(d, l)
#define LogBinaryLf(d, l)
#define LogData(t, id, d, l)
#define LogDataLf(t, id, d, l)
#define DoDebug(t) false
#define FlushDebugBuffer()
#define WaitMs(ms) delay(ms)

// DEBUG_FFB — descomentar para imprimir torque FFB em Serial (500Hz!)
//#define DEBUG_FFB

#else  // AVR / Pro Micro -------------------------------------------------------

extern volatile u8 gDebugMode;
b8 DoDebug(const u8 type);

#define DEBUG_ENABLE_UART
#define DEBUG_BUFFER_SIZE 512

void LogSendByte(u8 data);
void LogText(const char *text);
void LogTextLf(const char *text);
void LogTextP(const char *text);
void LogTextLfP(const char *text);
void LogBinary(const void *data, uint16_t len);
void LogBinaryLf(const void *data, uint16_t len);
void LogData(const char *text, u8 reportId, const void *data, uint16_t len);
void LogDataLf(const char *text, u8 reportId, const void *data, uint16_t len);
void LogReport(const char *text, const uint16_t *reportSizeArray, u8 *data, uint16_t len);
void FlushDebugBuffer(void);

#ifdef DEBUG_CALIBRATION
#define cal_print(str)   DEBUG_SERIAL.print(str)
#define cal_println(str) DEBUG_SERIAL.println(str)
#else
#define cal_print(str)
#define cal_println(str)
#endif

#endif  // ESP32

#endif  // _DEBUG_H_

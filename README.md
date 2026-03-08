# Arduino-FFB-wheel — USART3 Fork

Fork do [Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel) adaptado para comunicação serial USART3 com motores STM32/GD32 (hoverboard-firmware-hack-FOC). Substitui a saída PWM/DAC do original por protocolo serial bidirecional, delegando o controle FOC de corrente ao STM32/GD32 e usando o encoder do próprio STM32/GD32 como feedback de posição.

**Firmware do motor:** [hoverboard-firmware-hack-FOC-USART3](https://github.com/Jean-DrEaD/hoverboard-firmware-hack-FOC-USART3)
— variante PlatformIO: `ONE_AXIS_USART_VARIANT` (STM32) ou `GD32F103RC_stlink` / `GD32F103RC_uart` (GD32)

---

## Branches

| Branch | Plataforma | Pasta do sketch |
|--------|-----------|----------------|
| **`main`** — você está aqui | Pro Micro (ATmega32U4) | `brWheel_HID/` |
| `esp32-s3` | ESP32-S3 Zero / Mini | `brWheel_ESP32S3/` |

---

## Branch `main` — Pro Micro

## Pinout / Ligação

![Pinout](pinout.png)

| Sinal | STM32/GD32 (placa) | Arduino Pro Micro |
|---|---|---|
| USART3 TX (feedback) | Right Sideboard — PB10 / TX | RX1 (pin 0) |
| USART3 RX (comando) | Right Sideboard — PB11 / RX | TX1 (pin 1) via **R1 1kΩ + R2 2kΩ** |
| GND | Right Sideboard — GND | GND |
| Encoder A | Left Hall — PB6 | — |
| Encoder B | Left Hall — PB7 | — |
| Encoder VCC | Left Hall — 5 V | — |
| SWD Flash | PA14 (SWCLK) / PA13 (SWDIO) | ST-Link |

> ⚠️ **Divisor de tensão TX Arduino → RX STM32/GD32:** o Pro Micro opera a 5 V e o STM32/GD32 aceita 3,3 V. Use R1 = 1 kΩ em série com o TX e R2 = 2 kΩ entre o fio e GND.

### Hardware

| Função | Pino | Obs |
|--------|------|-----|
| UART TX → STM32/GD32 RX | TX1 (Serial1) | PB11 do STM32/GD32 (USART3 RX), 500kbps |
| UART RX ← STM32/GD32 TX | RX1 (Serial1) | PB10 do STM32/GD32 (USART3 TX) |
| Acelerador | A0 | Pot 3 fios, 5V max |
| Freio | A1 | Pot analógico ou load cell HX711 |
| Embreagem | A2 | Opcional |
| Gear UP | D9 | INPUT_PULLUP, switch NA → GND |
| Gear DN | D10 | INPUT_PULLUP, switch NA → GND |
| HX711 DOUT | D4 | Somente com USE_LOAD_CELL |
| HX711 SCK | D5 | Somente com USE_LOAD_CELL |

### Configuração (`brWheel_HID/Config.h`)

```c
#define USE_PROMICRO
#define USE_EEPROM
#define USE_SEQ_SHIFTER
#define SHIFTER_UP_PIN  9
#define SHIFTER_DN_PIN  10
//#define USE_LOAD_CELL   // descomente para load cell HX711 no freio
```

### Arduino IDE

```
Board: Arduino Leonardo | SparkFun Pro Micro | 5V / 16MHz
```

---

## Protocolo serial GD32 ↔ Arduino

Baud: **500 000 bps**, 8N1.

---

## Paridade de features com branch esp32-s3

| Feature | Pro Micro | ESP32-S3 |
|---------|-----------|----------|
| FFB engine completo (ffb_pro.ino) | ✅ | ✅ |
| HID compatível WheelControl | ✅ | ✅ |
| configCDC serial (U/V/S/R/C/G/E/B/Y/F/A) | ✅ | ✅ |
| applyDeadband + slewLimit torque | ✅ | ✅ |
| Auto-center por posição estável (4200ms) | ✅ | ✅ |
| Link timeout (200ms) + reset auto-center | ✅ | ✅ |
| Pedais analógicos (accel + brake) | ✅ | ✅ |
| Load cell HX711 no freio | ✅ USE_LOAD_CELL | ✅ USE_LOAD_CELL |
| Câmbio sequencial | ✅ D9/D10 | ✅ GPIO38/39 |
| Botões extras | ✅ D4–D7 | ✅ GPIO5–8 |
| USE_FIXED_TORQUE_TEST (debug) | ✅ | ✅ |
| Persistência de parâmetros | EEPROM (512B AVR) | Preferences (flash NVS) |
| ADC resolução | 10-bit → shift ×4 → 12-bit HID | 12-bit nativo |

---

## Bugs corrigidos (vs upstream ranenbg)

1. **Runaway pós-alinhamento** — sinal de torque invertido em `stmSendCmd()`
2. **Jump de posição fase 2→3** — `seed_count` usava contador open-loop em vez de TIM4 CNT real
3. **Auto-center reset ao cruzar pos=0** — removida guarda espúria `speedL_meas == 0`
4. **Queda de tensão antes do center** — `TORQUE_MAX` 950→700, `TORQUE_SLEW` 250→150
5. **`finalize_y_alignment()`** — resets overcurrent/watchdog/serial timeout ausentes
6. **`handle_y_high_power_phase()`** — zero-torque ausente antes de `ApplyDirection`

---

## Efeitos FFB suportados

Constant · Spring · Damper · Inertia · Friction · Sine · Square · Triangle · SawtoothUp · SawtoothDown · Ramp · Envelope · Desktop autocenter / damper / inertia / friction

Ganhos e rotação configuráveis via **WheelControl** (CDC serial, report `0xF1`).

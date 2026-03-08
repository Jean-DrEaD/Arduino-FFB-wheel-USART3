# Arduino-FFB-wheel â€” USART3 Fork

Fork do [Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel) com suporte a comunicaÃ§Ã£o serial USART3 para motores GD32/STM32 (hoverboard-firmware-hack-FOC).

**Firmware do motor:** [hoverboard-firmware-hack-FOC-USART3](https://github.com/Jean-DrEaD/hoverboard-firmware-hack-FOC-USART3)

---

## Branches

| Branch | Plataforma | Pasta do sketch |
|--------|-----------|----------------|
| `main` | Pro Micro (ATmega32U4) | `brWheel_HID/` |
| `esp32-s3` | ESP32-S3 Zero / Mini | `brWheel_ESP32S3/` |

---

## Branch `main` â€” Pro Micro

### Hardware

| FunÃ§Ã£o | Pino |
|--------|------|
| UART TX â†’ GD32 RX | TX1 (Serial1) |
| UART RX â† GD32 TX | RX1 (Serial1) |
| Acelerador | A0 |
| Freio | A1 |
| Embreagem | A2 (opcional) |
| Gear UP | D9 (INPUT_PULLUP, NA) |
| Gear DN | D10 (INPUT_PULLUP, NA) |

### ConfiguraÃ§Ã£o (`brWheel_HID/Config.h`)

```c
#define USE_PROMICRO
#define USE_EEPROM
#define USE_SEQ_SHIFTER
#define SHIFTER_UP_PIN  9
#define SHIFTER_DN_PIN  10
```

### Arduino IDE

```
Board: SparkFun Pro Micro (5V / 16MHz)
```

---

## Protocolo serial GD32 â†” Pro Micro / ESP32-S3

Baud: **500 000 bps**, 8N1.

```
PC â†’ GD32  (8 bytes):   0xABCD | steer=0 | speed=torque[-1000..1000] | checksum(XOR)
GD32 â†’ PC (16 bytes):   0xABCD | cmd1 | cmd2 | speedR | speedL=-enc_pos | batV | temp | led | checksum
```

`speedL_meas = -enc_pos` (negado no GD32). Torque positivo â†’ motor traciona â†’ enc_pos diminui â†’ loop fecha.

---

## Bugs corrigidos (vs upstream ranenbg)

1. **Runaway pÃ³s-alinhamento** â€” sinal de torque invertido em `stmSendCmd()`
2. **Jump de posiÃ§Ã£o fase 2â†’3** â€” `seed_count` usava contador open-loop em vez de TIM4 CNT real
3. **Auto-center reset ao cruzar pos=0** â€” removida guarda espÃºria `speedL_meas == 0`
4. **Queda de tensÃ£o antes do center** â€” `TORQUE_MAX` 950â†’700, `TORQUE_SLEW` 250â†’150
5. **`finalize_y_alignment()`** â€” resets overcurrent/watchdog/serial timeout ausentes
6. **`handle_y_high_power_phase()`** â€” zero-torque ausente antes de `ApplyDirection`

---

## Efeitos FFB suportados

Constant Â· Spring Â· Damper Â· Inertia Â· Friction Â· Sine Â· Square Â· Triangle Â· SawtoothUp/Down Â· Ramp Â· Envelope Â· Desktop autocenter/damper/inertia/friction

Ganhos configurÃ¡veis via **WheelControl** (CDC serial, report `0xF1`).
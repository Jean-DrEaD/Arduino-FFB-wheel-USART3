# Arduino-FFB-wheel — USART3 Fork

Fork do [Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel) com suporte a comunicação serial USART3 para motores GD32/STM32 (hoverboard-firmware-hack-FOC).

**Firmware do motor:** [hoverboard-firmware-hack-FOC-USART3](https://github.com/Jean-DrEaD/hoverboard-firmware-hack-FOC-USART3)

---

## Branches

| Branch | Plataforma | Pasta do sketch |
|--------|-----------|----------------|
| `main` | Pro Micro (ATmega32U4) | `brWheel_HID/` |
| `esp32-s3` | ESP32-S3 Zero / Mini | `brWheel_ESP32S3/` |

---

## Branch `main` — Pro Micro

### Hardware

| Função | Pino |
|--------|------|
| UART TX → GD32 RX | TX1 (Serial1) |
| UART RX ← GD32 TX | RX1 (Serial1) |
| Acelerador | A0 |
| Freio | A1 |
| Embreagem | A2 (opcional) |
| Gear UP | D9 (INPUT\_PULLUP, NA) |
| Gear DN | D10 (INPUT\_PULLUP, NA) |

### Configuração (`brWheel_HID/Config.h`)

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

## Protocolo serial GD32 ↔ Pro Micro / ESP32-S3

Baud: **500 000 bps**, 8N1.

```
PC → GD32  (8 bytes):   0xABCD | steer=0 | speed=torque[-1000..1000] | checksum(XOR)
GD32 → PC  (16 bytes):  0xABCD | cmd1 | cmd2 | speedR | speedL=-enc_pos | batV | temp | led | checksum
```

`speedL_meas = -enc_pos` (negado no GD32). Torque positivo → motor traciona → enc_pos diminui → loop fecha.

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

Constant · Spring · Damper · Inertia · Friction · Sine · Square · Triangle · SawtoothUp/Down · Ramp · Envelope · Desktop autocenter/damper/inertia/friction

Ganhos configuráveis via **WheelControl** (CDC serial, report `0xF1`).

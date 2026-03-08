# Arduino-FFB-wheel â€” USART3 Fork

Fork do [Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel) com suporte a comunicaÃ§Ã£o serial USART3 para motores GD32/STM32 (hoverboard-firmware-hack-FOC).

**Firmware do motor:** [hoverboard-firmware-hack-FOC-USART3](https://github.com/Jean-DrEaD/hoverboard-firmware-hack-FOC-USART3)

---

## Branches

| Branch | Plataforma | Pasta do sketch |
|--------|-----------|----------------|
| `main` | Pro Micro (ATmega32U4) | `brWheel_HID/` |
| **`esp32-s3`** â† vocÃª estÃ¡ aqui | **ESP32-S3 Zero / Mini** | **`brWheel_ESP32S3/`** |

---

## Branch `esp32-s3` â€” ESP32-S3 Zero / Mini

### Hardware

| FunÃ§Ã£o | GPIO | Obs |
|--------|------|-----|
| USB D- / D+ | 19 / 20 | Fixo |
| UART TX â†’ GD32 RX | 17 | PB11 do GD32 (USART3 RX) |
| UART RX â† GD32 TX | 18 | PB10 do GD32 (USART3 TX) |
| Acelerador | 1 | ADC1_CH0, max 3.3V |
| Freio | 2 | ADC1_CH1 |
| Embreagem | 3 | Opcional |
| Gear UP | 38 | INPUT_PULLUP, switch -> GND |
| Gear DN | 39 | INPUT_PULLUP, switch -> GND |
| Freio mao digital | 40 | Descomentar em Config |
| Botoes 0-3 | 5-8 | INPUT_PULLUP |
| Debug TX/RX | 43 / 44 | UART0 via USB chip |

> ADC max **3.3V**. Pedais em 5V: usar divisor resistivo 2:1 (ex: 2x10kOhm).

### Arduino IDE â€” obrigatorio

```
Board:           ESP32S3 Dev Module
USB Mode:        USB-OTG (TinyUSB)   <- sem isso nao funciona
USB CDC On Boot: Disabled            <- sem isso conflita com HID
Upload Mode:     UART0 / Hardware CDC
CPU Frequency:   240 MHz
```

Instalar arduino-esp32 >= 3.0:
`https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

Primeira gravacao: **BOOT** segurado -> **EN** pressionado -> soltar BOOT -> Upload.

### Arquivos do sketch (`brWheel_ESP32S3/`)

| Arquivo | Descricao |
|---------|-----------|
| `brWheel_ESP32S3.ino` | Loop principal: posicao, torque, pedais, botoes, HID |
| `Config_ESP32S3.h` | Pinos, parametros, globais compartilhados |
| `HID_Wheel_ESP32.h` | TinyUSB HID descriptor identico ao Pro Micro |
| `StmFrames.h` | Protocolo serial GD32 (identico ao Pro Micro) |
| `ffb.h` | Tipos FFB (guarda `#ifdef ESP32`) |
| `ffb.ino` | Parsing USB output reports (guards AVR/ESP32) |
| `ffb_pro.h` | Declaracoes do motor FFB |
| `ffb_pro.ino` | Efeitos completos com guards AVR/ESP32 |
| `debug.h` | No-ops no ESP32 |

### Diferencas vs Pro Micro

| Item | Pro Micro | ESP32-S3 |
|------|-----------|----------|
| USB callback | `HID_ReceiveReport_Callback` (AVR ISR) | `_onSetReport()` (TinyUSB) |
| ADC | 10 bits | 12 bits nativos |
| Calibracao mecanica | `BRFFB::calibrate()` com encoder local | Stub â€” alinhamento feito pelo GD32 |
| PROGMEM / `<util/delay.h>` | Necessario | Guards `#ifndef ESP32` |
| CPU / RAM | 16 MHz / 2.5 KB | 240 MHz / 512 KB |

---

## Protocolo serial GD32 <-> ESP32-S3

Baud: **500 000 bps**, 8N1.

```
PC -> GD32  (8 bytes):   0xABCD | steer=0 | speed=torque[-1000..1000] | checksum(XOR)
GD32 -> PC (16 bytes):   0xABCD | cmd1 | cmd2 | speedR | speedL=-enc_pos | batV | temp | led | checksum
```

---

## Efeitos FFB suportados

Constant Â· Spring Â· Damper Â· Inertia Â· Friction Â· Sine Â· Square Â· Triangle Â· SawtoothUp/Down Â· Ramp Â· Envelope Â· Desktop autocenter/damper/inertia/friction

Ganhos e rotacao configurÃ¡veis via **WheelControl** (CDC serial, report `0xF1`).

---

*Branch main (Pro Micro):* https://github.com/Jean-DrEaD/Arduino-FFB-wheel-USART3/tree/main
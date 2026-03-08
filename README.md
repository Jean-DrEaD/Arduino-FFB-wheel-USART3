# Arduino-FFB-wheel — USART3 Fork

Fork do [Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel) com suporte a comunicação serial USART3 para motores GD32/STM32 (hoverboard-firmware-hack-FOC).

**Firmware do motor:** [hoverboard-firmware-hack-FOC-USART3](https://github.com/Jean-DrEaD/hoverboard-firmware-hack-FOC-USART3)

---

## Branches

| Branch | Plataforma | Pasta do sketch |
|--------|-----------|----------------|
| `main` | Pro Micro (ATmega32U4) | `brWheel_HID/` |
| **`esp32-s3`** ← você está aqui | **ESP32-S3 Zero / Mini** | **`brWheel_ESP32S3/`** |

---

## Branch `esp32-s3` — ESP32-S3 Zero / Mini

### Hardware

| Função | GPIO | Obs |
|--------|------|-----|
| USB D- / D+ | 19 / 20 | Fixo |
| UART TX → GD32 RX | 17 | PB11 do GD32 (USART3 RX) |
| UART RX ← GD32 TX | 18 | PB10 do GD32 (USART3 TX) |
| Acelerador | 1 | ADC1\_CH0, máx 3.3V |
| Freio | 2 | ADC1\_CH1 |
| Embreagem | 3 | Opcional |
| Gear UP | 38 | INPUT\_PULLUP, switch → GND |
| Gear DN | 39 | INPUT\_PULLUP, switch → GND |
| Freio mão digital | 40 | Descomentar em Config |
| Botões 0–3 | 5–8 | INPUT\_PULLUP |
| Debug TX/RX | 43 / 44 | UART0 via USB chip |

> ⚠️ ADC máx **3.3V**. Pedais alimentados em 5V precisam de divisor resistivo 2:1 (ex: 2×10kΩ).

### Arduino IDE — obrigatório

```
Board:           ESP32S3 Dev Module
USB Mode:        USB-OTG (TinyUSB)   ← sem isso não funciona
USB CDC On Boot: Disabled            ← sem isso conflita com HID
Upload Mode:     UART0 / Hardware CDC
CPU Frequency:   240 MHz
```

Instalar arduino-esp32 ≥ 3.0:
`https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

Primeira gravação: **BOOT** segurado → **EN** pressionado → soltar BOOT → Upload.

### Arquivos do sketch (`brWheel_ESP32S3/`)

| Arquivo | Descrição |
|---------|-----------|
| `brWheel_ESP32S3.ino` | Loop principal: posição, torque, pedais, botões, HID |
| `Config_ESP32S3.h` | Pinos, parâmetros, globais compartilhados |
| `HID_Wheel_ESP32.h` | TinyUSB HID descriptor idêntico ao Pro Micro |
| `StmFrames.h` | Protocolo serial GD32 (idêntico ao Pro Micro) |
| `ffb.h` | Tipos FFB (guarda `#ifdef ESP32`) |
| `ffb.ino` | Parsing USB output reports (guards AVR/ESP32) |
| `ffb_pro.h` | Declarações do motor FFB |
| `ffb_pro.ino` | Efeitos completos: spring, damper, inertia, friction, periodic, ramp (guards AVR/ESP32) |
| `debug.h` | No-ops no ESP32 |

### Diferenças vs Pro Micro

| Item | Pro Micro | ESP32-S3 |
|------|-----------|----------|
| USB callback | `HID_ReceiveReport_Callback` (AVR ISR) | `_onSetReport()` (TinyUSB) |
| ADC | 10 bits, `<<2` para 12 bits HID | 12 bits nativos |
| Calibração mecânica | `BRFFB::calibrate()` com encoder local | Stub — alinhamento feito pelo GD32 |
| `FfbSetDriver(0)` | Registra callbacks USB | Chama `FfbInit()` (sem callbacks) |
| PROGMEM / `<util/delay.h>` | Necessário | Guards `#ifndef ESP32` |
| CPU / RAM | 16 MHz / 2.5 KB | 240 MHz / 512 KB |

---

## Protocolo serial GD32 ↔ ESP32-S3

Baud: **500 000 bps**, 8N1.

```
PC → GD32  (8 bytes):   0xABCD | steer=0 | speed=torque[-1000..1000] | checksum(XOR)
GD32 → PC  (16 bytes):  0xABCD | cmd1 | cmd2 | speedR | speedL=-enc_pos | batV | temp | led | checksum
```

`speedL_meas = -enc_pos` — posição em ticks do encoder (GD32 nega antes de enviar).

---

## Efeitos FFB suportados

Constant · Spring · Damper · Inertia · Friction · Sine · Square · Triangle · SawtoothUp/Down · Ramp · Envelope · Desktop autocenter/damper/inertia/friction

Ganhos e rotação configuráveis via **WheelControl** (CDC serial, report `0xF1`).

---

## Branch `main` — Pro Micro

Ver [README na branch main](https://github.com/Jean-DrEaD/Arduino-FFB-wheel-USART3/blob/main/README.md).

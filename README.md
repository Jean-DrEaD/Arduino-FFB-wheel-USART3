# PORT para ESP32-S3 do meu Arduino-FFB-wheel — USART3 Fork

---

## Branches

| Branch | Plataforma | Pasta do sketch |
|--------|-----------|----------------|
| `main` | Pro Micro (ATmega32U4) | `brWheel_HID/` |
| **`esp32-s3`** — você está aqui | ESP32-S3 Zero / Mini | `brWheel_ESP32S3/` |

---

## Branch `esp32-s3` — ESP32-S3 Zero / Mini

**Firmware do motor:** [hoverboard-firmware-hack-FOC-USART3](https://github.com/Jean-DrEaD/hoverboard-firmware-hack-FOC-USART3)

### Hardware
> ⚠️ ADC máx **3.3V**. Pedais com 5V precisam de divisor resistivo 2:1 (ex: 2×10kΩ).

| Função | GPIO | Obs |
|--------|------|-----|
| USB D- / D+ | 19 / 20 | Fixo, conectar ao PC |
| UART TX → STM32/GD32 RX | 17 | PB11 do STM32/GD32 (USART3 RX), 500kbps |
| UART RX ← STM32/GD32 TX | 18 | PB10 do STM32/GD32 (USART3 TX) |
| Acelerador | 1 | ADC1_CH0, máx **3.3V** |
| Freio | 2 | ADC1_CH1, ou HX711 com USE_LOAD_CELL |
| Embreagem | 3 | Opcional |
| HX711 DOUT | 11 | Somente com USE_LOAD_CELL |
| HX711 SCK | 12 | Somente com USE_LOAD_CELL |
| Gear UP | 38 | INPUT_PULLUP, switch NA → GND |
| Gear DN | 39 | INPUT_PULLUP, switch NA → GND |
| Freio mão digital | 40 | Opcional, descomentar em Config |
| Botões 0–3 | 5–8 | INPUT_PULLUP |
| Debug TX/RX | 43 / 44 | UART0 via USB chip da placa |

> ⚠️ ADC máx **3.3V**. Pedais com 5V precisam de divisor resistivo 2:1 (ex: 2×10kΩ).

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

### Configuração (`brWheel_ESP32S3/Config_ESP32S3.h`)

```c
//#define USE_LOAD_CELL   // descomente para load cell HX711 no freio
#define LC_DOUT_PIN  11
#define LC_SCK_PIN   12
#define LC_GAIN      128
```

Para usar load cell instale a biblioteca **"HX711_ADC"** por Olav Kallhovd no Gerenciador de Bibliotecas.

### Calibração load cell

`LC_scaling` (1–255) ajusta a sensibilidade via comando serial `B<n>` (WheelControl ou monitor serial).

```
calFactor = 0.25 × LC_scaling   →   padrão 128 → calFactor = 32.0
```

Aumentar `LC_scaling` = pedal mais sensível (menos força necessária para atingir 100%).

---

## Protocolo serial STM32/GD32 ↔ ESP32-S3

Baud: **500 000 bps**, 8N1.

```
ESP32 → STM32/GD32   (8 bytes):   0xABCD | steer=0 | speed=torque[-1000..1000] | checksum(XOR)
STM32/GD32 → ESP32  (16 bytes):   0xABCD | cmd1 | cmd2 | speedR | speedL=-enc_pos | batV | temp | led | checksum
```

---

## Paridade de features com branch main

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

## Efeitos FFB suportados

Constant · Spring · Damper · Inertia · Friction · Sine · Square · Triangle · SawtoothUp · SawtoothDown · Ramp · Envelope · Desktop autocenter / damper / inertia / friction

Ganhos e rotação configuráveis via **WheelControl** (CDC serial, report `0xF1`).

---

*Branch main (Pro Micro):* https://github.com/Jean-DrEaD/Arduino-FFB-wheel-USART3/tree/main

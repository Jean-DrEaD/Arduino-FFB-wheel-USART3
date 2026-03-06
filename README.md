# Arduino-FFB-wheel-USART3

Fork do [ranenbg/Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel) adaptado para comunicação Serial com STM32/GD32 via USART3 em vez de encoder físico no Arduino e saída PWM.

Usado em conjunto com [Jean-DrEaD/hoverboard-firmware-hack-FOC-USART3](https://github.com/Jean-DrEaD/hoverboard-firmware-hack-FOC-USART3) para montar um Direct Drive FFB Wheel.

---

## Hardware

| Componente | Detalhe |
|---|---|
| Arduino | Pro Micro / Leonardo (ATmega32U4) |
| Placa STM32/GD32 | Hoverboard STM32F103RC ou GD32F103RCT6 |
| Encoder | MT6701 no STM32 (ABZ, CPR = 4096) — não conectado ao Arduino |
| Comunicação | Serial1 (pinos 0/1) @ 500 000 baud via divisor 1kΩ/2kΩ |

> **Serial1 (pinos RX1/TX1)** é o link físico com o STM32. **Serial (USB CDC)** é a porta do Wheel Control GUI — os dois são independentes e não interferem.

---

## Conexões

| Sinal | Arduino Pro Micro | STM32 (placa) |
|---|---|---|
| RX1 — feedback STM32 | Pin 0 (RX1) | Right Sideboard PB10 / TX |
| TX1 — comando torque | Pin 1 (TX1) via **R1 1kΩ + R2 2kΩ** | Right Sideboard PB11 / RX |
| GND | GND | Right Sideboard GND |

---

## Compilação

Abrir `brWheel_HID/brWheel_HID.ino` na Arduino IDE.

- Placa: **Arduino Leonardo** ou **SparkFun Pro Micro 5V/16MHz**
- Não requer bibliotecas externas além das inclusas no fork

---

## Fixes aplicados neste fork

| # | Arquivo | Descrição |
|---|---|---|
| 1 | `brWheel_HID.ino` | `TOP = 1000` e `MM_MAX_MOTOR_TORQUE = 1000` em `setup()` — sem isso `EffectDivider() = INF` e todos os efeitos FFB retornam 0 |
| 2 | `brWheel_HID.ino` | `gEncPos_f = +f.speedL_meas - gPosOffset` (sem negação) — direita=positivo, alinhado com encoder MT6701 |
| 3 | `brWheel_HID.ino` | `hidPos` sem negação — eixo HID correto no Wheel Control GUI e no jogo |
| 4 | `brWheel_HID.ino` | `stmSendCmd(0, -gTorqueOut)` — compensa `INVERT_R_DIRECTION` no STM32; GD32 usa mesmo sinal (hardware já compensado) |
| 5 | `brWheel_HID.ino` | Auto-centro no primeiro frame válido após 5 s de boot — aguarda alinhamento FOC do STM32 antes de capturar `gPosOffset` |
| 6 | `brWheel_HID.ino` | `gPosOffset += gEncPos_f` no botão Center — referência acumulada correta; funciona com cliques consecutivos |
| 7 | `brWheel_HID.ino` | `CONFIG_SERIAL.setTimeout(10)` — reduz bloqueio do `parseInt()` de 1000 ms para 10 ms, evitando HID desaparecer ao fechar o Wheel Control |
| 8 | `brWheel_HID.ino` | `stmSendCmdKeepalive()` no `loop()` e dentro do `calibrate()` — STM32 nunca entra em timeout serial durante operação normal ou calibração |
| 9 | `ffb_pro.ino` | `command.x +=` em `USB_EFFECT_CONSTANT` e `USB_EFFECT_RAMP` (era `-=`) — kerbs e batidas chegam no sentido correto para motor DD Serial |
| 10 | `SerialInterface.ino` | Comando `'C'` retorna `1` ao Wheel Control (habilita botão Center no GUI) e seta flag `gResetPosition` |
| 11 | `StmFrames.h` | Definição do frame STM32 (8 bytes, Little Endian) em arquivo separado |

---

## Parâmetros principais (`brWheel_HID.ino`)

```cpp
#define ROTATION_DEG  1080    // ±540° lock-to-lock
#define CPR           4096    // MT6701 1024 PPR × 4
#define TOP           1000    // range de saída Serial → STM32
#define TORQUE_MAX     950    // pico de torque (escala 0–1000)
#define TORQUE_SLEW    250    // rampa máx por ciclo a 500 Hz (~4 ms de 0→950)
```

---

## Wheel Control GUI

Usar [ranenbg/Arduino-FFB-gui](https://github.com/ranenbg/Arduino-FFB-gui) v2.6.5 ou superior.

Copiar o arquivo `DD_base` (sem extensão) para a pasta `data/` do Wheel Control.

| Parâmetro | Valor | Nota |
|---|---|---|
| Rotation | 1080° | ±540° |
| CPR | 4096 | MT6701 |
| General Gain | 255 | 100% |
| Constant Gain | 255 | impactos e kerbs — canal principal DD |
| Periodic Gain | 230 | texturas e vibração de motor |
| Damper Gain | 40 | habilitado |
| Inertia Gain | 80 | habilitado |
| Spring Gain | 0 | desabilitado (motor DD) |
| Friction Gain | 0 | desabilitado (motor DD) |
| Center Spring | 0 | desabilitado |
| Stop Force | 255 | batentes de rotação |
| Desktop Effects | Damper + Inertia | `effstate = 6` |

---

## Fluxo de sinal

```
PC (jogo)
    │  USB HID FFB effects
    ▼
Arduino Pro Micro
    │  ffb_pro.ino — CalcTorqueCommands()
    │  command.x += magnitude  (constant/ramp)
    │  → gTorqueOut (com slew TORQUE_SLEW)
    │  stmSendCmd(0, -gTorqueOut)
    │  Serial1 @ 500 kbps
    ▼
STM32 / GD32F103
    │  FOC TRQ_MODE — cmdR = torque
    │  Motor BLDC → MT6701 → enc_pos
    │  SerialFeedback: speedL_meas = enc_pos
    ▼
Arduino Pro Micro
    │  gEncPos_f = speedL_meas - gPosOffset
    │  turn.x = gEncPos_f  →  HID axis X
    ▼
PC (jogo lê posição + calcula próximo FFB)
```

---

## Licença

GPL-3.0 — herdada do projeto original [ranenbg/Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel).

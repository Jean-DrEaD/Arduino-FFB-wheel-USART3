# hoverboard-firmware-hack-FOC — ONE_AXIS_USART_VARIANT (DD FFB Wheel)

Fork do [SiMachines/hoverboard-firmware-hack-FOC](https://github.com/simachines/hoverboard-firmware-hack-FOC) adaptado para encoder magnético MT6701 e comunicação Serial.

---

## Hardware

| Componente | Detalhe |
|---|---|
| Placa de controle | Hoverboard STM32F103 (compatível GD32F103RCT6) |
| Motor | Motor direito do hoverboard (BLDC, 15-polos) |
| Encoder | MT6701 — ABZ, 1024 PPR, CPR = 4096, conectado em PB6/PB7 (TIM4) |
| Interface HID | Arduino Pro Micro / Leonardo (ATmega32U4) | --------------------------- Em breve ESP32 também!
| Alimentação | PSU 6S (24 V), XT60 |
| Resistor de freio | R3 — 100 W, 10 Ω — Fase A(U) do motor esquerdo |
| Comunicação | USART3 a 500000 baud (Arduino TX→STM32 RX via divisor 1k/2k) |

---

## Pinout / Ligação

![Pinout](pinout.png)

### Conexões relevantes

| Sinal | STM32 (placa) | Arduino Pro Micro |
|---|---|---|
| USART3 TX (feedback) | Right Sideboard — PB10 / TX | RX1 (pin 0) |
| USART3 RX (comando) | Right Sideboard — PB11 / RX | TX1 (pin 1) via **R1 1kΩ + R2 2kΩ** |
| GND | Right Sideboard — GND | GND |
| Encoder A | Left Hall — PB6 | — |
| Encoder B | Left Hall — PB7 | — |
| Encoder VCC | Left Hall — 5 V | — |
| SWD Flash | PA14 (SWCLK) / PA13 (SWDIO) | ST-Link |

> **Divisor de tensão TX Arduino → RX STM32:** o Pro Micro opera a 5 V e o STM32 aceita 3,3 V. Use R1 = 1 kΩ em série com o TX e R2 = 2 kΩ entre o fio e GND, conforme diagrama.

> **Capacitores:** remover os capacitores indicados no diagrama para que o encoder ABZ e funcione corretamente.

---

## Firmware STM32 — variant `ONE_AXIS_USART_VARIANT`

### Configuração principal (`Inc/config.h`)

```c
#define ONE_AXIS_USART_VARIANT
#define ENCODER_X_PPR        1024       // MT6701 ABZ
#define CTRL_MOD_REQ         CFG_TRQ_MODE
#define TANK_STEERING
#define INVERT_R_DIRECTION
#define ALIGNMENT_X_POWER    8000       // 50% — mínimo que alinha o MT6701
#define USART3_BAUD          500000
#define FLASH_WRITE_KEY      0x1025
```

### Mods/Fixes aplicados neste fork

| # | Arquivo | Descrição |
|---|---|---|
| 1 | `Inc/config.h` | Resolve conflict markers do merge upstream (bloco `TWO_AXIS_VARIANT`) |
| 2 | `Src/util.c` | Resolve conflict markers no bloco de timeout/safe-state; mantém `encoder_x.ali` guard + usa `CFG_OPEN_MODE` |
| 3 | `Src/main.c` | Corrige `count_prev` inválido no reset de overflow int16: ressincroniza com `TIM4->CNT` real, eliminando salto de ~4096 ticks |
| 4 | `Inc/config.h` | `USART3_BAUD 500000` declarado explicitamente no bloco `ONE_AXIS_USART_VARIANT` |
| 5 | `Src/main.c` | Beep de timeout serial: 3 bips apenas na borda de subida do timeout, depois silêncio (sem bip contínuo ao desconectar o Arduino) |

---

## Firmware Arduino — brWheel_HID

Fork do [ranenbg/Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel) com adaptações para Encoder direto no conector Hall(esq) na placa STM32 via USART3 em vez de encoder físico no Arduino e comunicação PWM.

### Mods/Fixes aplicados

| # | Arquivo | Descrição |
|---|---|---|
| 1 | `brWheel_HID.ino` | `TOP = 1000`, `MM_MAX_MOTOR_TORQUE = 1000` em setup — sem isso `EffectDivider() = INF` e todos os efeitos FFB retornam 0 |
| 2 | `brWheel_HID.ino` | `gEncPos_f = +f.speedL_meas` (sem negação) — posição correta direita=positivo |
| 3 | `brWheel_HID.ino` | `hidPos` negado — eixo HID alinhado com sentido físico do volante |
| 4 | `brWheel_HID.ino` | `stmSendCmd(0, -gTorqueOut)` — torque no sentido correto para o STM32 com `INVERT_R_DIRECTION` |
| 5 | `brWheel_HID.ino` | `gPosOffset` para centro funcional — botão Center do Wheel Control funciona via offset em vez de reset absoluto |
| 6 | `ffb_pro.ino` | `command.x +=` em `USB_EFFECT_CONSTANT` e `USB_EFFECT_RAMP` (era `-=`) — kerbs e batidas chegam no sentido correto |
| 7 | `ffb_pro.ino` | `calibrate()` envia keepalive `stmSendCmd(0,0)` a cada 300 ms — STM32 não entra em timeout durante calibração do Wheel Control |
| 8 | `SerialInterface.ino` | Comando `'C'` retorna `1` (habilita botão Center no GUI) e seta flag `gResetPosition` |
| 9 | `StmFrames.h` | Frame STM(8 Bytes Little Indian) dedicado em arquivo separado.
 
### Parâmetros de setup (`brWheel_HID.ino`)

```cpp
ROTATION_DEG = 1080;  // ±540° lock-to-lock
CPR          = 4096;  // MT6701 1024 PPR × 4
TOP          = 1000;  // range Serial → STM32
TORQUE_MAX   = 950;   // pico de torque (escala 0-1000)
TORQUE_SLEW  = 250;   // rampa máx/ciclo a 500 Hz (~4 ms 0→950)
```

---

## Wheel Control GUI

Usar [ranenbg/Arduino-FFB-gui](https://github.com/ranenbg/Arduino-FFB-gui) v2.6.5 ou superior.

### Perfil base para DD (`DD_base`)

Copiar o arquivo `DD_base` (sem extensão) para a pasta `data/` do Wheel Control.

| Parâmetro | Valor | Nota |
|---|---|---|
| Rotation | 1080° | ±540° |
| CPR | 4096 | MT6701 |
| General Gain | 255 | 100% |
| Constant Gain | 255 | impactos/kerbs — canal principal DD |
| Periodic Gain | 230 | vibração de motor e texturas |
| Damper Gain | 40 | habilitado |
| Inertia Gain | 80 | habilitado |
| Spring Gain | 0 | desabilitado (motor DD) |
| Friction Gain | 0 | desabilitado (motor DD) |
| Center Spring | 0 | desabilitado |
| Stop Force | 255 | batentes de rotação |
| Desktop Effects | Damper + Inertia | effstate = 6 |

---

## Fluxo de sinal

```
PC (jogo) ──HID USB──► Arduino Pro Micro
                            │  CalcTorqueCommands()
                            │  Serial1 @ 500kbps (SerialCommand)
                            ▼
                       STM32 GD32F103
                            │  FOC TRQ_MODE
                            │  Motor BLDC
                            │  MT6701 → TIM4
                            │  Serial1 (SerialFeedback: speedL_meas = posição)
                            ▼
                       Arduino Pro Micro
                            │  gEncPos_f = speedL_meas - gPosOffset
                            │  HID report (eixo X = posição volante)
                            ▼
                       PC (jogo lê posição + envia próximo FFB)
```

---

## Compilação

```bash
# PlatformIO
pio run -e ONE_AXIS_USART_VARIANT

# Arduino IDE: abrir brWheel_HID/brWheel_HID.ino
# Placa: Arduino Leonardo ou SparkFun Pro Micro 5V/16MHz
```

---

## Licença

GPL-3.0 — herdada do projeto original [Emanuel-GitH/hoverboard-firmware-hack-FOC](https://github.com/Emanuel-GitH/hoverboard-firmware-hack-FOC).

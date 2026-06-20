# M5Unit Roller485 — Haptic Detent Firmware

### SKU:U182

This is a fork of the M5Stack Roller485 internal firmware that turns the unit
into a **haptic detent knob** (a "smart knob"). Instead of the stock current /
speed / position motor-controller modes, the firmware boots straight into a
closed-loop FOC haptic mode that uses the motor to render **detents** — you feel
evenly-spaced clicks as you turn the dial, and you can query and configure the
behavior over I2C / RS485.

## Hardware

Roller485 Unit is a brushless DC motor (BLDC) motion actuator kit with an
integrated FOC closed-loop drive system. It uses a 3504 200KV brushless motor
(max ~0.5A continuous phase current, ~1A peak) with a magnetic encoder for
feedback, driven by an STM32G431 (Cortex-M4F). The unit has a 0.66" OLED
display, an RGB indicator LED, a function button, and RS485 + I2C interfaces.
It accepts 6–16V via the PWR485 interface or 5V via Grove, and exposes SWD/SWO
for debugging. See the [datasheet](https://docs.m5stack.com/en/unit/Unit-Roller485)
for full hardware details.

## What this firmware does

- Boots directly into the haptic **detent** mode (no host command required).
- **12 detents per revolution** by default, continuous (no endstops), tuned for
  a light feel with a firm click into each position.
- The detent count, bounded-vs-continuous behavior, and strength are
  configurable live over I2C / RS485, and the current detent position can be
  read back.

The detent algorithm is based on Scott Bezek's [smartknob](https://github.com/scottbez1/smartknob).

### Control registers (I2C / RS485)

| Register | R/W | Meaning |
|----------|-----|---------|
| `0x3C`–`0x3F` | R | Current detent position (int32) |
| `0xD0` | R/W | Bounded flag — `0` = continuous, `1` = bounded with endstops |
| `0xD1`/`0xD2` | R/W | Detents per revolution (uint16, LSB/MSB) |
| `0xD3` | R/W | Detent strength (`0` = free spinning, higher = stiffer; P-gain = value × 10) |

The default I2C slave address is `0x64`.

## Building

The firmware builds with the GNU Arm toolchain bundled in
[STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)
(GCC + CMake + Ninja). FreeRTOS is not used — the application runs as a
super-loop.

```bash
cd code/ROLLER485
cmake -S . -B gcc_build -G Ninja -DCMAKE_BUILD_TYPE=Release
ninja -C gcc_build
```

Outputs land in `code/ROLLER485/gcc_build/`: `ROLLER485.elf`, `.hex`, `.bin`.

## Flashing

> **Note:** this firmware is linked to run directly from `0x08000000` (the whole
> flash image, no separate bootloader). Flash the complete `.hex` at the start of
> flash via SWD with an ST-Link, e.g. using the STM32CubeProgrammer CLI:

```bash
STM32_Programmer_CLI -c port=SWD mode=UR -d gcc_build/ROLLER485.hex -v -rst
```

The non-volatile parameter page (which holds the encoder calibration) is left
untouched, so an existing factory encoder calibration is preserved.

## Demo branch

The `demo-detent-presets` branch adds a demo where a **button click** cycles
through a set of detent presets (fine / coarse / bounded / on-off switch /
free-spin), each shown with a distinct RGB LED color.

## Related Link

See also examples using conventional methods here.

- [Unit Roller485 & Datasheet](https://docs.m5stack.com/en/unit/Unit-Roller485)

## Related Project

This project references the following open source projects.

- [smartknob](https://github.com/scottbez1/smartknob)
- [PID_Controller](https://github.com/tcleg/PID_Controller)
- [u8g2](https://github.com/olikraus/u8g2)

## License

- [smartknob][] Copyright (c) 2022 Scott Bezek and licensed under Apache License, Version 2.0 License.
- [PID_Controller][] Copyright (c) 2013-2014 tcleg and licensed under GPLv3 License.
- [u8g2][] Copyright (c) 2016 olikraus and licensed under BSD License.

[smartknob]: https://github.com/scottbez1/smartknob
[PID_Controller]: https://github.com/tcleg/PID_Controller
[u8g2]: https://github.com/olikraus/u8g2

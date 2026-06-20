/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
 * Placeholder for a legacy include. mysys.c carries a vestigial `#include
 * "oled.h"` but uses no symbols from it (the OLED is driven via oled_u8g2.h /
 * u8g2_disp_fun.h). This empty header lets the GCC/CMake build resolve the
 * include without altering mysys.c.
 */
#ifndef __OLED_H__
#define __OLED_H__

#endif /* __OLED_H__ */

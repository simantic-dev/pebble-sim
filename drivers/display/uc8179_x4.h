/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/drivers/display/ssd1677.h>

// UC8179 panel (LUT_VER 0x01 / 0x40): hardware reset and controller init.
int uc8179_x4_init(const struct pbl_display_ssd1677 *epd);
// Full-screen black/white refresh. Synchronous: completes the display update before returning.
int uc8179_x4_update(const struct pbl_display_ssd1677 *epd, const struct pbl_display_buffer *buf);

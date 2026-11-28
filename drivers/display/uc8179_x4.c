/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// Init and full black/white refresh for the UC8179 controller on the X4 Pro glass.
// Ported from freeink-sdk libs/display/FreeInkDisplay/src/driver/Uc8179Driver.cpp
// (Copyright (c) 2026 FreeInk, MIT). Uses the panel's OTP waveform: no LUT upload.
// Not ported: grayscale, partial/fast refresh, power-save, cursor.

#include "uc8179_x4.h"
#include "uc8279_x4.h"

#include <errno.h>

#include <pbl/drivers/pinctrl.h>
#include <pbl/kernel/delay.h>

#define UC8179_IDLE_TIMEOUT_MS 5000U

struct uc8179_cmd {
  uint8_t command;
  uint8_t length;
  uint8_t data[4];
};

// BUSY reads 1 when the controller is idle (same polarity as uc8279_x4.c prv_uc_wait).
static int prv_wait_idle(const struct pbl_display_ssd1677 *epd) {
  return PBL_POLL_SLEEP_MS(pbl_gpio_get(&epd->busy) == 1, 1, UC8179_IDLE_TIMEOUT_MS);
}

static int prv_send(const struct pbl_display_ssd1677 *epd, const struct uc8179_cmd *list,
                    unsigned count) {
  for (unsigned i = 0; i < count; i++) {
    int ret = epaper_write_command(epd, list[i].command, list[i].data, list[i].length);
    if (ret != 0) {
      return ret;
    }
  }
  return 0;
}

int uc8179_x4_init(const struct pbl_display_ssd1677 *epd) {
  static const struct uc8179_cmd init[] = {
    {0x00, 2, {0x3f, 0x0a}},             // PSR: OTP waveform, SHL
    {0x61, 4, {0x03, 0x20, 0x02, 0x58}}, // TRES 800x600
    {0x65, 4, {0, 0, 0, 0}},             // GSST
    {0x03, 1, {0x20}},                   // PFS
    {0x06, 4, {0x25, 0x25, 0x3c, 0x25}}, // BTST
    {0xe1, 1, {0x02}},                   // gate scan
  };
  // Reset pulse before init (reference bus.reset(50)).
  (void)pbl_gpio_set(&epd->reset, true);
  pbl_sleep_ms(50);
  (void)pbl_gpio_set(&epd->reset, false);
  pbl_sleep_ms(30);
  return prv_send(epd, init, PBL_ARRAY_SIZE(init));
}

int uc8179_x4_update(const struct pbl_display_ssd1677 *epd, const struct pbl_display_buffer *buf) {
  static const struct uc8179_cmd pre[] = {
    {0x50, 2, {0x29, 0x07}}, // CDI active
    {0xe0, 1, {0x02}},       // CCSET
    {0xe5, 1, {0x1e}},       // TSSET full refresh
    {0x00, 2, {0x1f, 0x0a}}, // PSR: REG cleared -> OTP waveform
  };
  static const struct uc8179_cmd post[] = {
    {0x50, 2, {0xa9, 0x07}}, // CDI idle
  };
  if (buf->pitch != (epd->disp.props.width + 7U) / 8U) {
    return -EINVAL;
  }
  // NEW plane = frame, OLD plane = white (full flash, as the reference's fillPlane 0xFF).
  int ret = epaper_write_frame(epd, 0x13, buf->data);
  if (ret == 0) {
    ret = epaper_write_frame(epd, 0x10, nullptr);
  }
  if (ret == 0) {
    ret = prv_send(epd, pre, PBL_ARRAY_SIZE(pre));
  }
  if (ret == 0) {
    ret = epaper_write_command(epd, 0x04, nullptr, 0); // PON
  }
  if (ret == 0) {
    ret = prv_wait_idle(epd);
  }
  if (ret == 0) {
    ret = epaper_write_command(epd, 0x12, nullptr, 0); // DRF
  }
  if (ret == 0) {
    (void)PBL_POLL_SLEEP_MS(pbl_gpio_get(&epd->busy) == 0, 1, 50);
    ret = prv_wait_idle(epd);
  }
  if (ret == 0) {
    ret = prv_send(epd, post, PBL_ARRAY_SIZE(post));
  }
  if (ret == 0) {
    ret = epaper_write_command(epd, 0x02, nullptr, 0); // POF: no idle power-off path for this panel
  }
  if (ret == 0) {
    ret = prv_wait_idle(epd);
  }
  return ret;
}

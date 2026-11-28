/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// Init and full black/white refresh for the UC8179 controller on the X4 Pro glass.
// Ported from freeink-sdk libs/display/FreeInkDisplay/src/driver/Uc8179Driver.cpp
// (Copyright (c) 2026 FreeInk, MIT). Uses the panel's OTP waveform: no LUT upload.
// Not ported: grayscale, partial/fast refresh, power-save, cursor.

#include "uc8179_x4.h"
#include "uc8279_x4.h"

#include <errno.h>
#include <string.h>

#include <pbl/drivers/pinctrl.h>
#include <pbl/kernel/delay.h>

// Quarter turn from RAM (x across 800, y down 480) to the 480x800 framebuffer, upright
// with the keys at the bottom (checked on an X4 Pro, panel LUT_VER 01).
#define UC8179_X4_FB_X(x, y) (479U - (y))
#define UC8179_X4_FB_Y(x, y) (x)

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
  int ret = prv_send(epd, init, PBL_ARRAY_SIZE(init));
  return ret != 0 ? ret : uc8279_x4_start_thread(epd);
}

// Landscape RAM as the reference streams it (Uc8179Driver::streamPlane): 800-pixel rows of 100
// bytes, sent bottom row first, then white for the controller's 120 gates past the glass.
// RAM (x, y) <- framebuffer pixel; the quarter turn is chosen against the calibrated sim panel.
#define UC8179_W   800U
#define UC8179_H   480U
#define UC8179_PAD 120U

static bool prv_fb_white(const struct pbl_display_buffer *buf, uint32_t x, uint32_t y) {
  // Framebuffer is 480 wide x 800 tall, LSB-first, 1 = white.
  const uint32_t fx = UC8179_X4_FB_X(x, y), fy = UC8179_X4_FB_Y(x, y);
  return (buf->data[fy * buf->pitch + fx / 8U] >> (fx % 8U)) & 1U;
}

static int prv_stream(const struct pbl_display_ssd1677 *epd, uint8_t cmd,
                      const struct pbl_display_buffer *buf) {
  uint8_t *row = epd->row;
  int ret = epaper_write_command(epd, cmd, nullptr, 0);
  for (int y = (int)UC8179_H - 1; ret == 0 && y >= 0; y--) {
    for (uint32_t b = 0; b < UC8179_W / 8U; b++) {
      uint8_t v = 0xff;
      if (buf != nullptr) {
        v = 0;
        for (uint32_t i = 0; i < 8U; i++) {
          v |= (uint8_t)(prv_fb_white(buf, b * 8U + i, (uint32_t)y) << (7U - i));
        }
      }
      row[b] = v;
    }
    ret = pbl_spi_write(&epd->spi, row, UC8179_W / 8U);
  }
  memset(row, 0xff, UC8179_W / 8U);
  for (uint32_t y = 0; ret == 0 && y < UC8179_PAD; y++) {
    ret = pbl_spi_write(&epd->spi, row, UC8179_W / 8U);
  }
  return ret;
}

// Full (clearing) refresh on the first frame and every UC8179_FULL_EVERY-th, fast partial
// otherwise, as the reference does (Uc8179Driver::startBwRefresh / displayFinish). One panel.
#define UC8179_FULL_EVERY 20U
static uint32_t s_fast_left;
static bool s_powered;

int uc8179_x4_update(const struct pbl_display_ssd1677 *epd, const struct pbl_display_buffer *buf) {
  if (buf->pitch != (epd->disp.props.width + 7U) / 8U) {
    return -EINVAL;
  }
  const bool fast = s_fast_left > 0U;
  const struct uc8179_cmd pre[] = {
    {0x50, 2, {0x29, 0x07}},              // CDI active
    {0xe0, 1, {0x02}},                    // CCSET
    {0xe5, 1, {fast ? 0x5a : 0x1e}},      // TSSET: fast frame count / full
    {0x00, 2, {0x1f, 0x0a}},              // PSR: REG cleared -> OTP waveform
  };
  static const struct uc8179_cmd fast_pre[] = {
    {0x03, 1, {0x20}}, // PFS re-assert: without it the OTP waveform runs full length
    {0xe1, 1, {0x02}}, // gate scan
  };
  static const struct uc8179_cmd post[] = {
    {0x50, 2, {0xa9, 0x07}}, // CDI idle
  };
  // NEW plane = frame. Full: OLD = white (clearing flash). Fast: OLD already holds the last frame.
  int ret = prv_stream(epd, 0x13, buf);
  if (ret == 0 && !fast) {
    ret = prv_stream(epd, 0x10, nullptr);
  }
  if (ret == 0) {
    ret = prv_send(epd, pre, PBL_ARRAY_SIZE(pre));
  }
  if (ret == 0 && fast) {
    ret = prv_send(epd, fast_pre, PBL_ARRAY_SIZE(fast_pre));
  }
  if (ret == 0 && !s_powered) {
    ret = epaper_write_command(epd, 0x04, nullptr, 0); // PON
    if (ret == 0) {
      ret = prv_wait_idle(epd);
    }
    s_powered = ret == 0;
  }
  if (ret == 0 && fast) {
    ret = epaper_write_command(epd, 0x91, nullptr, 0); // PTIN: whole-panel partial
  }
  if (ret == 0) {
    ret = epaper_write_command(epd, 0x12, nullptr, 0); // DRF
  }
  if (ret == 0) {
    (void)PBL_POLL_SLEEP_MS(pbl_gpio_get(&epd->busy) == 0, 1, 50);
    ret = prv_wait_idle(epd);
  }
  if (ret == 0 && fast) {
    ret = epaper_write_command(epd, 0x92, nullptr, 0); // PTOUT
  }
  if (ret == 0) {
    ret = prv_send(epd, post, PBL_ARRAY_SIZE(post));
  }
  // OLD = what is now on the glass, so the next fast refresh drives only changed pixels.
  if (ret == 0) {
    ret = prv_stream(epd, 0x10, buf);
  }
  s_fast_left = ret != 0 ? 0U : fast ? s_fast_left - 1U : UC8179_FULL_EVERY - 1U;
  return ret;
}

int uc8179_x4_power_off(const struct pbl_display_ssd1677 *epd) {
  if (!s_powered) {
    return 0;
  }
  int ret = epaper_write_command(epd, 0x02, nullptr, 0); // POF
  if (ret == 0) {
    ret = prv_wait_idle(epd);
  }
  s_powered = false;
  return ret;
}

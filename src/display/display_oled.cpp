/*
 * SparkMiner - OLED Display Driver Implementation
 * U8g2-based driver for small monochrome OLED displays
 *
 * Retro look: Press Start 2P titles (scaled 2x/3x for big numbers), a 5x7
 * GLCD-style font for dense text, an inverted header strip with the SNES
 * A B X Y buttons (A/B convex = filled, X/Y concave = hollow), keycaps with
 * a thick bottom edge, and progress drawn as blocks instead of a smooth bar.
 *
 * With OLED_ON_TFT (T-Display) the same screens are drawn into a 1-bit
 * OLED_WIDTH x OLED_HEIGHT canvas that is pushed to the ST7789 at 2x, so the
 * pixels, fonts and colors are those of the SSD1306.
 *
 * GPL v3 License
 */

#include <Arduino.h>
#include <board_config.h>
#include "display/display_oled.h"

#if USE_OLED_DISPLAY

#include <U8g2lib.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Preferences.h>

#if OLED_ON_TFT
    #include <TFT_eSPI.h>
#else
    #include <Wire.h>
#endif

// ============================================================
// Configuration
// ============================================================

// Default I2C pins (can be overridden in board_config.h)
#ifndef OLED_SDA_PIN
    #define OLED_SDA_PIN 21
#endif

#ifndef OLED_SCL_PIN
    #define OLED_SCL_PIN 22
#endif

#ifndef OLED_I2C_ADDR
    #define OLED_I2C_ADDR 0x3C
#endif

// Display dimensions (set in board_config.h)
#ifndef OLED_WIDTH
    #define OLED_WIDTH 128
#endif

#ifndef OLED_HEIGHT
    #define OLED_HEIGHT 64
#endif

// Screens, cycled with a single click on BOOT
#define OLED_SCREEN_MAIN    0
#define OLED_SCREEN_LOTTO   1
#define OLED_SCREEN_POOL    2
#define OLED_SCREEN_CHAIN   3
#define OLED_SCREEN_COUNT   4

// Fonts: Press Start 2P for titles/labels/numbers, 5x7 for dense text
#define FONT_TITLE  u8g2_font_pressstart2p_8r
#define FONT_TEXT   u8g2_font_5x7_tr
#define FONT_TINY   u8g2_font_4x6_tr

#define TEXT_W      5       // FONT_TEXT advance
#define TITLE_W     8       // FONT_TITLE advance
#define HEADER_H    13      // strip (10) + gap (1) + rule (2)

// Screens advance on their own every AUTO_CYCLE_MS when auto cycle is on
#define AUTO_CYCLE_MS   8000
#define TOAST_MS        1800

// ============================================================
// U8g2 Display Object
// ============================================================

#if OLED_ON_TFT

// Scale and panel size (landscape)
#define TFT_SCALE   2
#define PANEL_W     240
#define PANEL_H     135
#define CANVAS_X    ((PANEL_W - OLED_WIDTH * TFT_SCALE) / 2)
#define CANVAS_Y    ((PANEL_H - OLED_HEIGHT * TFT_SCALE) / 2)

// The SSD1306 colors: lit pixels white, dark pixels black
#ifndef MONO_LIT
    #define MONO_LIT    0xFFFF
#endif
#ifndef MONO_DARK
    #define MONO_DARK   0x0000
#endif

// Backlight PWM (LEDC channel 7 stays clear of anything else in this build)
#define BL_LEDC_CH      7
#define BL_LEDC_FREQ    5000
#define BL_LEDC_BITS    8

// U8g2 with no hardware behind it: a full W x H buffer in the SSD1306 layout
// (bytes are 8-pixel columns, LSB on top)
template <int W, int H>
class U8G2_CANVAS : public U8G2 {
public:
    U8G2_CANVAS() : U8G2() {
        u8g2_SetupDisplay(&u8g2, setupCb, u8x8_cad_empty, u8x8_byte_empty, u8x8_dummy_cb);
        u8g2_SetupBuffer(&u8g2, s_buf, (H + 7) / 8, u8g2_ll_hvline_vertical_top_lsb, U8G2_R0);
    }

private:
    static uint8_t s_buf[W * ((H + 7) / 8)];

    static uint8_t setupCb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
        if (msg == U8X8_MSG_DISPLAY_SETUP_MEMORY) {
            static u8x8_display_info_t info;
            memset(&info, 0, sizeof(info));
            info.tile_width = (W + 7) / 8;
            info.tile_height = (H + 7) / 8;
            info.pixel_width = W;
            info.pixel_height = H;
            u8x8_d_helper_display_setup_memory(u8x8, &info);
        }
        return 1;
    }
};

template <int W, int H>
uint8_t U8G2_CANVAS<W, H>::s_buf[W * ((H + 7) / 8)];

static U8G2_CANVAS<OLED_WIDTH, OLED_HEIGHT> s_u8g2;
// One-page (128x8) canvas for rendering glyphs before scaling them up
static U8G2_CANVAS<128, 8> s_glyphs;
static TFT_eSPI s_tft;
static uint16_t s_line[OLED_WIDTH * TFT_SCALE];

// Select display constructor based on size
#elif (OLED_HEIGHT == 64)
    // 128x64 SSD1306
    static U8G2_SSD1306_128X64_NONAME_F_HW_I2C s_u8g2(U8G2_R0, U8X8_PIN_NONE);
#elif (OLED_HEIGHT == 32)
    // 128x32 SSD1306
    static U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C s_u8g2(U8G2_R0, U8X8_PIN_NONE);
#else
    // Default to 128x64
    static U8G2_SSD1306_128X64_NONAME_F_HW_I2C s_u8g2(U8G2_R0, U8X8_PIN_NONE);
#endif

#if !OLED_ON_TFT
// One-page (128x8) canvas for rendering glyphs before scaling them up.
// Never begun, so it never touches the bus.
static U8G2_SSD1306_128X64_NONAME_1_HW_I2C s_glyphs(U8G2_R0, U8X8_PIN_NONE);
#endif

// ============================================================
// State
// ============================================================

static uint8_t s_currentScreen = OLED_SCREEN_MAIN;
static uint8_t s_brightness = 255;
static uint8_t s_rotation = 0;
static bool s_needsRedraw = true;
static bool s_inverted = true;     // dark theme until set_inverted()
static bool s_screenOff = false;
static bool s_blink = false;        // toggles every update, drives attention lamps

// Rendering happens on the monitor task and, for instant feedback, on the
// button task: the lock keeps them from drawing into the buffer at once
static SemaphoreHandle_t s_lock = NULL;
static display_data_t s_last;       // latest data, to redraw right after a button press
static bool s_hasData = false;
static bool s_modal = false;        // reset screens own the display until redraw()

static bool s_autoCycle = false;
static uint32_t s_lastCycle = 0;

// Toast: a small box over the current screen (brightness, auto cycle...)
static char s_toastTitle[12];
static char s_toastText[16];
static float s_toastLevel = -1;     // >= 0 draws a block bar instead of text
static uint32_t s_toastUntil = 0;

struct DisplayLock {
    DisplayLock()  { if (s_lock) xSemaphoreTakeRecursive(s_lock, portMAX_DELAY); }
    ~DisplayLock() { if (s_lock) xSemaphoreGiveRecursive(s_lock); }
};

// ============================================================
// Formatting Helpers (fixed buffers, no String)
// ============================================================

// Splits a value into a number with 3-4 significant digits and an SI prefix
static void fmtSI(double v, char *num, size_t n, char *prefix) {
    static const char kPrefix[] = {' ', 'K', 'M', 'G', 'T', 'P', 'E', 'Z'};
    int i = 0;
    while (v >= 1000.0 && i < 7) {
        v /= 1000.0;
        i++;
    }
    if (v < 10.0)       snprintf(num, n, "%.2f", v);
    else if (v < 100.0) snprintf(num, n, "%.1f", v);
    else                snprintf(num, n, "%.0f", v);
    *prefix = kPrefix[i];
}

// Compact SI string, e.g. "1.23M", "725K", "12"
static void fmtCompact(double v, char *out, size_t n) {
    char num[12], p;
    if (v < 1000.0) {
        if (v < 1.0 && v > 0.0) snprintf(out, n, "%.4f", v);
        else                    snprintf(out, n, "%.0f", v);
        return;
    }
    fmtSI(v, num, sizeof(num), &p);
    snprintf(out, n, "%s%c", num, p);
}

static void fmtUptime(uint32_t s, char *out, size_t n) {
    uint32_t d = s / 86400, h = (s % 86400) / 3600, m = (s % 3600) / 60;
    if (d > 0)      snprintf(out, n, "%luD%02luH", (unsigned long)d, (unsigned long)h);
    else if (h > 0) snprintf(out, n, "%luH%02luM", (unsigned long)h, (unsigned long)m);
    else            snprintf(out, n, "%luM%02luS", (unsigned long)m, (unsigned long)(s % 60));
}

// 912345 -> "912,345"
static void fmtThousands(uint32_t v, char *out, size_t n) {
    char raw[12];
    snprintf(raw, sizeof(raw), "%lu", (unsigned long)v);
    int len = strlen(raw), o = 0;
    for (int i = 0; i < len && o < (int)n - 1; i++) {
        if (i > 0 && (len - i) % 3 == 0 && o < (int)n - 2) out[o++] = ',';
        out[o++] = raw[i];
    }
    out[o] = '\0';
}

static void upcase(char *s) {
    for (; *s; s++) *s = toupper((unsigned char)*s);
}

// Cut to maxChars, marking the cut with a trailing '~'
static void fitChars(char *s, int maxChars) {
    if (maxChars < 1) { s[0] = '\0'; return; }
    if ((int)strlen(s) > maxChars) {
        s[maxChars - 1] = '~';
        s[maxChars] = '\0';
    }
}

// ============================================================
// Drawing Primitives
// ============================================================

#if OLED_ON_TFT
// invertColors=true is the dark theme: lit pixels on black, like the OLED
static uint16_t colorLit()  { return s_inverted ? MONO_LIT : MONO_DARK; }
static uint16_t colorDark() { return s_inverted ? MONO_DARK : MONO_LIT; }

// Each canvas pixel becomes a TFT_SCALE x TFT_SCALE block
static void blitCanvas() {
    const uint8_t *buf = s_u8g2.getBufferPtr();
    const uint16_t lit = colorLit(), dark = colorDark();
    s_tft.startWrite();
    s_tft.setAddrWindow(CANVAS_X, CANVAS_Y, OLED_WIDTH * TFT_SCALE, OLED_HEIGHT * TFT_SCALE);
    for (int y = 0; y < OLED_HEIGHT; y++) {
        const uint8_t *col = buf + (y >> 3) * OLED_WIDTH;
        const uint8_t bit = 1 << (y & 7);
        uint16_t *out = s_line;
        for (int x = 0; x < OLED_WIDTH; x++) {
            uint16_t c = (col[x] & bit) ? lit : dark;
            for (int s = 0; s < TFT_SCALE; s++) *out++ = c;
        }
        for (int s = 0; s < TFT_SCALE; s++) s_tft.pushPixels(s_line, OLED_WIDTH * TFT_SCALE);
    }
    s_tft.endWrite();
}

static void setBacklight(uint8_t pct) {
    ledcWrite(BL_LEDC_CH, (uint32_t)pct * ((1 << BL_LEDC_BITS) - 1) / 100);
}

static void panelRotation(uint8_t rotation) {
    // Landscape both ways: 0 -> TFT rotation 3, 2 -> TFT rotation 1
    s_tft.setRotation(rotation == 2 ? 1 : 3);
    s_tft.fillScreen(colorDark());
    blitCanvas();
}
#else
static void panelRotation(uint8_t rotation) {
    s_u8g2.setDisplayRotation(rotation == 2 ? U8G2_R2 : U8G2_R0);
}
#endif

static void present() {
#if OLED_ON_TFT
    blitCanvas();
#else
    s_u8g2.sendBuffer();
#endif
#ifdef OLED_SCREENSHOT
    // Debug: dump the frame so it can be rendered on the host
    const uint8_t *buf = s_u8g2.getBufferPtr();
    Serial.print("OLEDSHOT ");
    for (int i = 0; i < OLED_WIDTH * OLED_HEIGHT / 8; i++) Serial.printf("%02x", buf[i]);
    Serial.println();
#endif
}

// Press Start 2P scaled by an integer factor: glyphs are rendered on the
// one-page canvas (caps on rows 0-6) and each pixel becomes a scale x scale box.
// y is the top of the capitals.
static int drawBig(int x, int y, const char *s, uint8_t scale) {
    s_glyphs.clearBuffer();
    s_glyphs.setFont(FONT_TITLE);
    s_glyphs.setFontMode(1);
    s_glyphs.setFontPosBaseline();
    int w = s_glyphs.getStrWidth(s);
    if (w > OLED_WIDTH) w = OLED_WIDTH;
    s_glyphs.drawStr(0, 8, s);
    const uint8_t *glyphs = s_glyphs.getBufferPtr();

    for (int cx = 0; cx < w; cx++) {
        uint8_t col = glyphs[cx];
        for (int cy = 0; col && cy < 8; cy++, col >>= 1) {
            if (col & 1) s_u8g2.drawBox(x + cx * scale, y + cy * scale, scale, scale);
        }
    }
    return w * scale;
}

static int bigWidth(const char *s, uint8_t scale) {
    s_u8g2.setFont(FONT_TITLE);
    return s_u8g2.getStrWidth(s) * scale;
}

static void textAt(int x, int y, const char *s, const uint8_t *font = FONT_TEXT) {
    s_u8g2.setFont(font);
    s_u8g2.setFontPosTop();
    s_u8g2.drawStr(x, y, s);
}

static int textWidth(const char *s, const uint8_t *font = FONT_TEXT) {
    s_u8g2.setFont(font);
    return s_u8g2.getStrWidth(s);
}

static void textRight(int xr, int y, const char *s, const uint8_t *font = FONT_TEXT) {
    textAt(xr - textWidth(s, font), y, s, font);
}

static void textCenter(int y, const char *s, const uint8_t *font = FONT_TEXT) {
    textAt((OLED_WIDTH - textWidth(s, font)) / 2, y, s, font);
}

// SNES pad: A and B convex (filled), X and Y concave (hollow)
static void abxyDots(int x, int cy) {
    s_u8g2.drawDisc(x, cy, 2);
    s_u8g2.drawDisc(x + 6, cy, 2);
    s_u8g2.drawCircle(x + 12, cy, 2);
    s_u8g2.drawCircle(x + 18, cy, 2);
}

// Header strip: filled bar, dark text, A B X Y, page pips, then a 2px rule
static void header(const char *title, bool alert, bool showPips = true) {
    s_u8g2.setDrawColor(1);
    s_u8g2.drawBox(0, 0, OLED_WIDTH, 10);

    s_u8g2.setDrawColor(0);
    abxyDots(4, 4);
    textAt(28, 1, title, FONT_TITLE);

    int x = OLED_WIDTH - 2;
    if (showPips) {
        for (int i = OLED_SCREEN_COUNT - 1; i >= 0; i--) {
            x -= 4;
            if (i == s_currentScreen) s_u8g2.drawBox(x, 3, 3, 3);
            else                      s_u8g2.drawPixel(x + 1, 4);
        }
        x -= 3;
    }

    // Attention lamp: a '!' that flips between dark and lit every update
    if (alert) {
        x -= 7;
        if (s_blink) {
            s_u8g2.drawBox(x, 1, 7, 8);
            s_u8g2.setDrawColor(1);
        }
        textAt(x + 1, 2, "!");
    }

    s_u8g2.setDrawColor(1);
    s_u8g2.drawBox(0, 11, OLED_WIDTH, 2);
}

// Keycap: rounded outline with a 2px bottom edge, label centered.
// Selected keys are filled with dark text.
static void keycap(int x, int y, int w, const char *label, bool selected = false) {
    const int h = 12;
    if (selected) {
        s_u8g2.drawRBox(x, y, w, h, 2);
        s_u8g2.setDrawColor(0);
    } else {
        s_u8g2.drawRFrame(x, y, w, h, 2);
        s_u8g2.drawHLine(x + 1, y + h - 2, w - 2);
    }
    char buf[16];
    strncpy(buf, label, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    fitChars(buf, (w - 4) / TEXT_W);
    textAt(x + (w - textWidth(buf)) / 2, y + 2, buf);
    s_u8g2.setDrawColor(1);
}

// Progress as discrete blocks with 2px gaps: filled = on, outline = off
static void blockBar(int x, int y, int w, int h, int segs, float frac) {
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    int segW = (w - (segs - 1) * 2) / segs;
    int on = (int)(frac * segs + 0.5f);
    for (int i = 0; i < segs; i++) {
        int sx = x + i * (segW + 2);
        if (i < on)        s_u8g2.drawBox(sx, y, segW, h);
        else if (segW >= 3) s_u8g2.drawFrame(sx, y, segW, h);
        else               s_u8g2.drawHLine(sx, y + h - 1, segW);
    }
}

// WiFi signal as 4 rising bars (dots where the signal is missing)
static void signalBars(int x, int yBottom, bool connected, int rssi) {
    int level = !connected ? 0 : rssi > -55 ? 4 : rssi > -65 ? 3 : rssi > -75 ? 2 : rssi > -85 ? 1 : 0;
    for (int i = 0; i < 4; i++) {
        int h = 2 + i * 2;
        if (i < level) s_u8g2.drawBox(x + i * 3, yBottom - h + 1, 2, h);
        else           s_u8g2.drawPixel(x + i * 3, yBottom);
    }
}

// Label in 5x7 on the left, value on the right (5x7 or Press Start 2P)
static void row(int y, const char *label, const char *value, bool big = false) {
    textAt(2, big ? y + 2 : y, label);
    int room = OLED_WIDTH - 2 - textWidth(label) - 6;
    char buf[40];
    strncpy(buf, value, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    fitChars(buf, room / (big ? TITLE_W : TEXT_W));
    textRight(OLED_WIDTH - 2, y, buf, big ? FONT_TITLE : FONT_TEXT);
}

// ============================================================
// Screen Drawing Functions
// ============================================================

// MINER: big hashrate, shares / best / uptime keycaps, pool status
static void drawMainScreen(const display_data_t *data) {
    bool problem = !data->wifiConnected || !data->poolConnected;
    bool jackpot = data->blocksFound > 0;
    header(jackpot ? "JACKPOT" : "MINER", problem || jackpot);

    // Hashrate, Press Start 2P at 2x, unit in 5x7 on the baseline
    char num[12], p, unit[8];
    fmtSI(data->hashRate, num, sizeof(num), &p);
    if (data->hashRate >= 1000.0 && data->hashRate < 1e6) {
        snprintf(num, sizeof(num), "%.1f", data->hashRate / 1000.0);
    }
    if (p == ' ') snprintf(unit, sizeof(unit), "H/S");
    else          snprintf(unit, sizeof(unit), "%cH/S", p);
    int numW = bigWidth(num, 2);
    int unitW = textWidth(unit);
    int x = (OLED_WIDTH - numW - 3 - unitW) / 2;
    drawBig(x, 16, num, 2);
    textAt(x + numW + 3, 24, unit);

    // Labels above the keycaps: three keys, 3px apart, 1px margins
    // (128 wide: 40px keys at 1/44/87; 120 wide: 37px keys at 1/41/81)
    const int kw = (OLED_WIDTH - 2 - 2 * 3) / 3;
    const int kx[3] = {1, 1 + kw + 3, 1 + 2 * (kw + 3)};
    const char *labels[3] = {"SHARES", "BEST", "UPTIME"};
    for (int i = 0; i < 3; i++) {
        textAt(kx[i] + (kw - textWidth(labels[i], FONT_TINY)) / 2, 33, labels[i], FONT_TINY);
    }

    char shares[12], best[12], up[12];
    snprintf(shares, sizeof(shares), "%lu", (unsigned long)data->sharesAccepted);
    fmtCompact(data->bestDifficulty, best, sizeof(best));
    fmtUptime(data->uptimeSeconds, up, sizeof(up));
    keycap(kx[0], 40, kw, shares);
    keycap(kx[1], 40, kw, best);
    keycap(kx[2], 40, kw, up);

    // Status line: '>' doing now, '!' needs you
    char status[32];
    if (!data->wifiConnected) {
        snprintf(status, sizeof(status), "! NO WIFI");
    } else if (!data->poolConnected) {
        snprintf(status, sizeof(status), "! NO POOL");
    } else {
        snprintf(status, sizeof(status), "> %s", data->poolName ? data->poolName : "MINING");
        upcase(status);
    }
    fitChars(status, (OLED_WIDTH - 16) / TEXT_W);
    textAt(1, 56, status);
    signalBars(OLED_WIDTH - 11, 62, data->wifiConnected, data->wifiRssi);
}

// LOTTO: best share against the block target, and the odds of hitting it
static void drawLottoScreen(const display_data_t *data) {
    bool jackpot = data->blocksFound > 0;
    header(jackpot ? "JACKPOT" : "LOTTO", jackpot);

    double target = data->networkDifficultyRaw;
    char best[12], tgt[12];
    fmtCompact(data->bestDifficulty, best, sizeof(best));
    if (target > 0) fmtCompact(target, tgt, sizeof(tgt));
    else            snprintf(tgt, sizeof(tgt), "---");

    row(14, "BEST", best, true);
    row(23, "BLOCK", tgt, true);

    // How far the best share got, in orders of magnitude, towards a block
    float luck = 0;
    if (target > 1 && data->bestDifficulty > 1) {
        luck = log10(data->bestDifficulty) / log10(target);
    }
    blockBar(2, 34, OLED_WIDTH - 4, 6, 20, luck);

    char odds[16], eta[16];
    if (target > 0 && data->hashRate > 0) {
        // Each hash wins with probability 1 / (difficulty * 2^32)
        double hashesPerBlock = target * 4294967296.0;
        double perDay = hashesPerBlock / (data->hashRate * 86400.0);
        double years = hashesPerBlock / data->hashRate / 31557600.0;
        char n[12];
        fmtCompact(perDay, n, sizeof(n));
        snprintf(odds, sizeof(odds), "1:%s", n);
        fmtCompact(years, n, sizeof(n));
        snprintf(eta, sizeof(eta), "%sY", n);
    } else {
        snprintf(odds, sizeof(odds), "---");
        snprintf(eta, sizeof(eta), "---");
    }
    if (jackpot) {
        snprintf(eta, sizeof(eta), "%lu BLK", (unsigned long)data->blocksFound);
        row(43, "ODDS/DAY", odds, true);
        row(53, "FOUND", eta, true);
    } else {
        row(43, "ODDS/DAY", odds, true);
        row(53, "ETA", eta, true);
    }
}

// POOL: connection and share details
static void drawPoolScreen(const display_data_t *data) {
    header("POOL", !data->poolConnected);

    char v[40];
    snprintf(v, sizeof(v), "%s", data->poolConnected && data->poolName ? data->poolName : "OFFLINE");
    upcase(v);
    row(15, "POOL", v);

    snprintf(v, sizeof(v), "%lu OK %lu BAD",
             (unsigned long)data->sharesAccepted, (unsigned long)data->sharesRejected);
    row(23, "SHARES", v);

    fmtCompact(data->poolDifficulty, v, sizeof(v));
    row(31, "DIFF", v);

    snprintf(v, sizeof(v), "%lu", (unsigned long)data->templates);
    row(39, "JOBS", v);

    if (data->avgLatency > 0) snprintf(v, sizeof(v), "%luMS", (unsigned long)data->avgLatency);
    else                      snprintf(v, sizeof(v), "---");
    row(47, "PING", v);

    snprintf(v, sizeof(v), "%s", data->wifiConnected && data->ipAddress ? data->ipAddress : "NO WIFI");
    row(55, "IP", v);
}

// CHAIN: Bitcoin network from the live stats API
static void drawChainScreen(const display_data_t *data) {
    header("CHAIN", false);

    char v[24];
    if (data->btcPrice > 0) {
        char n[16];
        fmtThousands((uint32_t)data->btcPrice, n, sizeof(n));
        snprintf(v, sizeof(v), "$%s", n);
    } else {
        snprintf(v, sizeof(v), "---");
    }
    row(15, "PRICE", v, true);

    if (data->blockHeight > 0) fmtThousands(data->blockHeight, v, sizeof(v));
    else                       snprintf(v, sizeof(v), "---");
    row(25, "BLOCK", v, true);

    snprintf(v, sizeof(v), "%s", data->networkHashrate[0] ? data->networkHashrate : "---");
    upcase(v);
    row(36, "HASH", v);

    if (data->halfHourFee > 0) snprintf(v, sizeof(v), "%d SAT/VB", data->halfHourFee);
    else                       snprintf(v, sizeof(v), "---");
    row(45, "FEE", v);

    // Difficulty epoch: progress through the 2016 blocks, expected change
    // Bar leaves 36px on the right for the change ("+12.3%")
    textAt(2, 55, "EPOCH");
    blockBar(32, 55, OLED_WIDTH - 68, 7, 10, data->difficultyProgress / 100.0f);
    if (data->difficultyProgress > 0) {
        snprintf(v, sizeof(v), "%+.1f%%", data->difficultyChange);
        textRight(OLED_WIDTH - 2, 55, v);
    }
}

// ============================================================
// Toast: boxed message over the current screen
// ============================================================

static void drawToast() {
    const int w = OLED_WIDTH - 16, h = 32;
    const int x = 8, y = 18;

    // Blank the band between the header and the status line, then a double
    // border like the boot screen
    s_u8g2.setDrawColor(0);
    s_u8g2.drawBox(0, HEADER_H + 1, OLED_WIDTH, 40);
    s_u8g2.setDrawColor(1);
    s_u8g2.drawFrame(x, y, w, h);
    s_u8g2.drawFrame(x + 2, y + 2, w - 4, h - 4);

    textAt(x + (w - textWidth(s_toastTitle, FONT_TITLE)) / 2, y + 7, s_toastTitle, FONT_TITLE);
    if (s_toastLevel >= 0) {
        blockBar(x + 8, y + 19, w - 16, 6, 10, s_toastLevel);
    } else {
        textAt(x + (w - textWidth(s_toastText)) / 2, y + 19, s_toastText);
    }
}

static void showToast(const char *title, const char *text, float level) {
    strncpy(s_toastTitle, title, sizeof(s_toastTitle) - 1);
    s_toastTitle[sizeof(s_toastTitle) - 1] = '\0';
    strncpy(s_toastText, text ? text : "", sizeof(s_toastText) - 1);
    s_toastText[sizeof(s_toastText) - 1] = '\0';
    s_toastLevel = level;
    s_toastUntil = millis() + TOAST_MS;
}

// ============================================================
// Rendering
// ============================================================

// Draws the current screen from data and pushes it. Caller holds the lock.
static void render(const display_data_t *data) {
    s_u8g2.clearBuffer();
    switch (s_currentScreen) {
        case OLED_SCREEN_LOTTO:
            drawLottoScreen(data);
            break;
        case OLED_SCREEN_POOL:
            drawPoolScreen(data);
            break;
        case OLED_SCREEN_CHAIN:
            drawChainScreen(data);
            break;
        default:
            drawMainScreen(data);
            break;
    }
    if (s_toastUntil && (int32_t)(millis() - s_toastUntil) < 0) drawToast();
    else s_toastUntil = 0;
    present();
    s_needsRedraw = false;
}

// Redraw now with the latest data (button feedback can't wait for the next
// one-second update)
static void renderNow() {
    DisplayLock lock;
    if (s_screenOff || s_modal || !s_hasData) return;
    render(&s_last);
}

// ============================================================
// Public API Implementation
// ============================================================

void oled_display_init(uint8_t rotation, uint8_t brightness) {
    Serial.printf("[OLED] Initializing %dx%d display\n", OLED_WIDTH, OLED_HEIGHT);

    if (!s_lock) s_lock = xSemaphoreCreateRecursiveMutex();
    DisplayLock lock;

    // Auto cycle survives reboots (cleared by factory reset with the rest)
    Preferences prefs;
    if (prefs.begin("sparkminer", true)) {
        s_autoCycle = prefs.getBool("autocycle", false);
        prefs.end();
    }

    s_rotation = (rotation >= 2) ? 2 : 0;
    s_brightness = (brightness * 255) / 100;

#if OLED_ON_TFT
    s_tft.init();
    ledcSetup(BL_LEDC_CH, BL_LEDC_FREQ, BL_LEDC_BITS);
    ledcAttachPin(TFT_BL_PIN, BL_LEDC_CH);
    setBacklight(brightness);
    panelRotation(s_rotation);
#else
    // Initialize I2C with custom pins
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    s_u8g2.begin();
    panelRotation(s_rotation);
    s_u8g2.setContrast(s_brightness);
#endif

    // Show boot screen
    oled_display_show_boot();

    Serial.println("[OLED] Display initialized");
}

void oled_display_update(const display_data_t *data) {
    DisplayLock lock;
    s_last = *data;
    s_hasData = true;
    if (s_screenOff || s_modal) return;

#ifdef OLED_SCREENSHOT
    // Debug: walk through every screen so each one gets dumped
    static uint8_t s_frames = 0;
    if (++s_frames % 4 == 0) s_currentScreen = (s_currentScreen + 1) % OLED_SCREEN_COUNT;
#endif

    if (s_autoCycle && millis() - s_lastCycle >= AUTO_CYCLE_MS) {
        s_currentScreen = (s_currentScreen + 1) % OLED_SCREEN_COUNT;
        s_lastCycle = millis();
    }

    s_blink = !s_blink;
    render(data);
}

void oled_display_set_brightness(uint8_t brightness) {
    DisplayLock lock;
    s_brightness = (brightness * 255) / 100;
#if OLED_ON_TFT
    setBacklight(brightness);
#else
    s_u8g2.setContrast(s_brightness);
#endif
}

void oled_display_next_screen() {
    s_currentScreen = (s_currentScreen + 1) % OLED_SCREEN_COUNT;
    s_lastCycle = millis();
    s_needsRedraw = true;
    Serial.printf("[OLED] Screen: %d\n", s_currentScreen);
    renderNow();
}

void oled_display_prev_screen() {
    s_currentScreen = (s_currentScreen + OLED_SCREEN_COUNT - 1) % OLED_SCREEN_COUNT;
    s_lastCycle = millis();
    s_needsRedraw = true;
    Serial.printf("[OLED] Screen: %d\n", s_currentScreen);
    renderNow();
}

void oled_display_show_level(const char *title, uint8_t percent) {
    showToast(title, NULL, percent / 100.0f);
    renderNow();
}

bool oled_display_toggle_auto_cycle() {
    s_autoCycle = !s_autoCycle;
    s_lastCycle = millis();
    Preferences prefs;
    if (prefs.begin("sparkminer", false)) {
        prefs.putBool("autocycle", s_autoCycle);
        prefs.end();
    }
    char text[16];
    if (s_autoCycle) snprintf(text, sizeof(text), "ON  EVERY %dS", AUTO_CYCLE_MS / 1000);
    else             snprintf(text, sizeof(text), "OFF");
    showToast("AUTO", text, -1);
    Serial.printf("[OLED] Auto cycle %s\n", s_autoCycle ? "on" : "off");
    renderNow();
    return s_autoCycle;
}

void oled_display_show_ap_config(const char *ssid, const char *password, const char *ip) {
    DisplayLock lock;
    s_u8g2.clearBuffer();
    s_blink = !s_blink;
    header("SETUP", true, false);

    textAt(2, 15, "! JOIN THIS WIFI");
    if (textWidth(ssid, FONT_TITLE) <= OLED_WIDTH - 4) textAt(2, 24, ssid, FONT_TITLE);
    else                                               textAt(2, 25, ssid);

    // The password is case sensitive: never upcase it
    textAt(2, 35, "PASS");
    textAt(28, 35, password);

    textAt(2, 45, "THEN OPEN");
    textAt(2, 54, ip, FONT_TITLE);

    present();
}

void oled_display_show_boot() {
    DisplayLock lock;
    s_u8g2.clearBuffer();

    // Double border, like a cartridge label
    s_u8g2.drawFrame(0, 0, OLED_WIDTH, OLED_HEIGHT);
    s_u8g2.drawFrame(2, 2, OLED_WIDTH - 4, OLED_HEIGHT - 4);

    drawBig((OLED_WIDTH - bigWidth("SPARK", 2)) / 2, 8, "SPARK", 2);
    drawBig((OLED_WIDTH - bigWidth("MINER", 2)) / 2, 25, "MINER", 2);

    abxyDots(OLED_WIDTH / 2 - 9, 45);

    char line[32];
    snprintf(line, sizeof(line), "%s  BTC LOTTERY", AUTO_VERSION);
    upcase(line);
    textCenter(53, line, FONT_TINY);

    present();
}

void oled_display_show_reset_countdown(int seconds) {
    DisplayLock lock;
    s_modal = true;
    s_u8g2.clearBuffer();
    s_blink = !s_blink;
    header("RESET", true, false);

    textCenter(16, "! KEEP HOLDING TO WIPE");

    char n[4];
    snprintf(n, sizeof(n), "%d", seconds);
    drawBig((OLED_WIDTH - bigWidth(n, 3)) / 2, 30, n, 3);

    present();
}

void oled_display_show_reset_complete() {
    DisplayLock lock;
    s_modal = true;
    s_u8g2.clearBuffer();
    header("RESET", false, false);

    drawBig((OLED_WIDTH - bigWidth("DONE", 2)) / 2, 22, "DONE", 2);
    textCenter(48, "> REBOOTING");

    present();
}

void oled_display_redraw() {
    s_modal = false;
    s_needsRedraw = true;
    renderNow();
}

uint8_t oled_display_flip_rotation() {
    oled_display_set_rotation(s_rotation == 0 ? 2 : 0);
    return s_rotation;
}

void oled_display_set_rotation(uint8_t rotation) {
    // Only 0 and 2 (180 degree flip): both are landscape
    {
        DisplayLock lock;
        s_rotation = (rotation >= 2) ? 2 : 0;
        panelRotation(s_rotation);
        s_needsRedraw = true;
    }
    renderNow();
}

void oled_display_set_inverted(bool inverted) {
    {
        DisplayLock lock;
        s_inverted = inverted;
#if OLED_ON_TFT
        s_tft.fillScreen(colorDark());
        blitCanvas();   // keep what is on screen (boot screen before any data)
#else
        // invertColors=true is the dark theme (the CYD panel is inverted), which on
        // an OLED is the normal mode. SSD1306: 0xA6 normal, 0xA7 inverted.
        s_u8g2.sendF("c", inverted ? 0xA6 : 0xA7);
#endif
        s_needsRedraw = true;
    }
    renderNow();
}

uint16_t oled_display_get_width() {
    return OLED_WIDTH;
}

uint16_t oled_display_get_height() {
    return OLED_HEIGHT;
}

bool oled_display_is_portrait() {
    return false;  // OLEDs are typically landscape
}

uint8_t oled_display_get_screen() {
    return s_currentScreen;
}

void oled_display_set_screen(uint8_t screen) {
    if (screen < OLED_SCREEN_COUNT) {
        s_currentScreen = screen;
        s_needsRedraw = true;
    }
}

// ============================================================
// Display Driver Interface
// ============================================================

static DisplayDriver s_oledDriver = {
    .init = oled_display_init,
    .update = oled_display_update,
    .set_brightness = oled_display_set_brightness,
    .next_screen = oled_display_next_screen,
    .show_ap_config = oled_display_show_ap_config,
    .show_boot = oled_display_show_boot,
    .show_reset_countdown = oled_display_show_reset_countdown,
    .show_reset_complete = oled_display_show_reset_complete,
    .redraw = oled_display_redraw,
    .flip_rotation = oled_display_flip_rotation,
    .set_inverted = oled_display_set_inverted,
    .get_width = oled_display_get_width,
    .get_height = oled_display_get_height,
    .is_portrait = oled_display_is_portrait,
    .get_screen = oled_display_get_screen,
    .set_screen = oled_display_set_screen,
    .name = "U8g2 OLED"
};

DisplayDriver* oled_get_driver() {
    return &s_oledDriver;
}

// ============================================================
// Standard display API wrappers (for main.cpp compatibility)
// ============================================================

void display_init(uint8_t rotation, uint8_t brightness) {
    oled_display_init(rotation, brightness);
}

void display_update(const display_data_t *data) {
    oled_display_update(data);
}

void display_set_brightness(uint8_t brightness) {
    oled_display_set_brightness(brightness);
}

void display_set_screen(uint8_t screen) {
    oled_display_set_screen(screen);
}

uint8_t display_get_screen() {
    return oled_display_get_screen();
}

void display_next_screen() {
    oled_display_next_screen();
}

void display_redraw() {
    oled_display_redraw();
}

uint8_t display_flip_rotation() {
    return oled_display_flip_rotation();
}

void display_set_rotation(uint8_t rotation) {
    oled_display_set_rotation(rotation);
}

uint16_t display_get_width() {
    return oled_display_get_width();
}

uint16_t display_get_height() {
    return oled_display_get_height();
}

bool display_is_portrait() {
    return oled_display_is_portrait();
}

bool display_touched() {
    return false;  // OLED has no touch
}

void display_handle_touch() {
    // No-op for OLED
}

void display_show_ap_config(const char *ssid, const char *password, const char *ip) {
    oled_display_show_ap_config(ssid, password, ip);
}

void display_set_inverted(bool inverted) {
    oled_display_set_inverted(inverted);
}

void display_show_reset_countdown(int seconds) {
    oled_display_show_reset_countdown(seconds);
}

void display_show_reset_complete() {
    oled_display_show_reset_complete();
}

void display_prev_screen() {
    oled_display_prev_screen();
}

void display_show_level(const char *title, uint8_t percent) {
    oled_display_show_level(title, percent);
}

bool display_toggle_auto_cycle() {
    return oled_display_toggle_auto_cycle();
}

// Screen timeout. OLED: no backlight, so power the panel down instead (also
// saves the pixels from burn-in). TFT: backlight off, panel asleep.
void display_set_backlight_off() {
    DisplayLock lock;
    s_screenOff = true;
#if OLED_ON_TFT
    setBacklight(0);
    s_tft.writecommand(ST7789_SLPIN);
#else
    s_u8g2.setPowerSave(1);
#endif
}

void display_set_backlight_on() {
    {
        DisplayLock lock;
        if (!s_screenOff) return;
        s_screenOff = false;
#if OLED_ON_TFT
        s_tft.writecommand(ST7789_SLPOUT);
        delay(5);
        setBacklight(s_brightness * 100 / 255);
#else
        s_u8g2.setPowerSave(0);
#endif
    }
    renderNow();
}

bool display_is_backlight_off() { return s_screenOff; }

#endif // USE_OLED_DISPLAY

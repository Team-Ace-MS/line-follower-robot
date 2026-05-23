/*
 * ================================================================
 *   LINE FOLLOWING ROBOT — COMPETITION FIRMWARE  v2.0
 *   Hardware : Raspberry Pi Pico 2 (RP2350)
 *   Sensor   : 16-channel QRE1113 array (3.3 V, digital)
 *   Driver   : TB6612FNG dual motor driver
 *   Display  : SSD1306 0.96" I2C OLED (128×64)
 *   Motors   : N20 800 RPM (×2)
 * ================================================================
 *
 *  ┌─────────────────────────────────────────────────────────┐
 *  │                  MAZE MEMORY SYSTEM                     │
 *  │                                                         │
 *  │  RUN 1 : Left-Hand Rule exploration                     │
 *  │          Every junction decision is recorded.           │
 *  │          Dead-ends → backtrack + fix path in memory.    │
 *  │          Finish detected → path saved to flash          │
 *  │                           (via Pico's XIP flash).       │
 *  │                                                         │
 *  │  RUN 2+ : Replay optimized path from memory.            │
 *  │           Skips all wrong turns. Full speed.            │
 *  │                                                         │
 *  │  FINISH DETECTION (all 3 types handled):                │
 *  │   A) Full black bar  — all sensors ON ≥ 120 ms          │
 *  │   B) T / cross end   — specific sensor pattern          │
 *  │   C) Return to start — encoder / timer distance         │
 *  └─────────────────────────────────────────────────────────┘
 *
 *  BUTTON MAP (active-LOW, internal pull-up)
 *  BTN_SEL   — cycle { Kp | Ki | Kd | Speed }
 *  BTN_UP    — increment selected variable
 *  BTN_DOWN  — decrement selected variable
 *  BTN_START — start / stop run + lap timer
 *
 *  OLED LAYOUT during run:
 *  ┌──────────────────────┐
 *  │ T 00:00.000  R1/OPT  │   (run type)
 *  │ Kp 1.200             │
 *  │ Ki 0.001             │
 *  │ Kd 8.500             │
 *  │ J:12 S:180 E:-350    │   (junctions seen, speed, error)
 *  └──────────────────────┘
 * ================================================================
 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ================================================================
//  PIN DEFINITIONS
// ================================================================

// 16-channel sensor array  (GP0–GP15, digital, active-HIGH = on line)
static const uint SENSOR_PINS[16] = {
    0,1,2,3,4,5,6,7,
    8,9,10,11,12,13,14,15
};

// TB6612FNG — Motor A (LEFT)
#define MOTOR_A_IN1   16
#define MOTOR_A_IN2   17
#define MOTOR_A_PWM   18

// TB6612FNG — Motor B (RIGHT)
#define MOTOR_B_IN1   19
#define MOTOR_B_IN2   20
#define MOTOR_B_PWM   21

#define MOTOR_STBY    22

// I2C / OLED
#define I2C_PORT      i2c1
#define I2C_SDA       26
#define I2C_SCL       27
#define OLED_ADDR     0x3C

// Buttons (active-LOW)
// NOTE: GP24/GP25 are free on RP2350 — use those so GP0-GP15 are
// fully available for the sensor array.
#define BTN_SEL       24
#define BTN_UP        25
#define BTN_DOWN      28
#define BTN_START     29

// ================================================================
//  FLASH STORAGE  — last 4 KB sector of 2 MB flash
//  Pico 2 flash: 0x10000000 base, 2 MB = 0x200000
//  Last sector  : 0x101FF000  (sector size = 4096 bytes)
// ================================================================
#define FLASH_SIZE_BYTES   (2 * 1024 * 1024)
#define FLASH_SECTOR_SIZE   4096
#define FLASH_PATH_OFFSET  (FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
// XIP base: flash appears at 0x10000000 in address space
#define FLASH_PATH_ADDR    (XIP_BASE + FLASH_PATH_OFFSET)

#define FLASH_MAGIC        0xDEADBEEF

// ================================================================
//  PID DEFAULTS  (float throughout)
// ================================================================
static float Kp         = 1.200f;
static float Ki         = 0.001f;
static float Kd         = 8.500f;
static int   BASE_SPEED = 180;

#define KP_STEP    0.050f
#define KI_STEP    0.001f
#define KD_STEP    0.250f
#define SPD_STEP   5

#define MAX_INTEGRAL    5000.0f
#define MAX_CORRECTION   255.0f

// ================================================================
//  SENSOR WEIGHTS  (-7500 … +7500)
// ================================================================
static const int SENSOR_WEIGHT[16] = {
    -7500,-6500,-5500,-4500,
    -3500,-2500,-1500, -500,
      500, 1500, 2500, 3500,
     4500, 5500, 6500, 7500
};

// ================================================================
//  JUNCTION / MAZE DEFINITIONS
// ================================================================

// Maximum junctions we can remember per run
#define MAX_JUNCTIONS   256

// Direction tokens — stored as uint8_t in flash
typedef enum : uint8_t {
    DIR_STRAIGHT = 'S',
    DIR_LEFT     = 'L',
    DIR_RIGHT    = 'R',
    DIR_BACK     = 'B',   // U-turn (dead end)
    DIR_FINISH   = 'F',   // finish reached
    DIR_NONE     = 0
} Direction;

// Junction type detected from sensor pattern
typedef enum {
    JT_NONE,        // not a junction — normal line
    JT_LEFT,        // line continues left (T-left or fork)
    JT_RIGHT,       // line continues right
    JT_T_BOTH,      // T-junction: left + straight + right
    JT_CROSS,       // full cross: all four ways
    JT_DEAD_END,    // no line anywhere ahead
    JT_FINISH_BAR,  // full-black bar (type A finish)
    JT_FINISH_T,    // T-end (type B finish)
} JunctionType;

// A recorded path — up to MAX_JUNCTIONS decisions
struct PathRecord {
    uint32_t  magic;                    // FLASH_MAGIC
    uint8_t   length;                   // number of junction decisions
    Direction decisions[MAX_JUNCTIONS]; // sequence of DIR_* tokens
    uint32_t  time_ms;                  // run time when finish reached
};

// ================================================================
//  GLOBAL MAZE STATE
// ================================================================

// Exploration path (built during Run 1)
static Direction explore_path[MAX_JUNCTIONS];
static int       explore_len = 0;

// Optimized path (built from explore_path by simplification)
static Direction optimal_path[MAX_JUNCTIONS];
static int       optimal_len = 0;

// Replay index (used during Run 2+)
static int       replay_idx  = 0;

// ── Run mode
typedef enum {
    MODE_EXPLORE,   // Run 1: Left-Hand Rule + record
    MODE_REPLAY,    // Run 2+: follow optimal_path
} RunMode;
static RunMode run_mode = MODE_EXPLORE;

// ── Flash helpers ────────────────────────────────────────────

static void flash_save_path(const Direction *path, int len, uint32_t time_ms) {
    // Build record in RAM first
    static PathRecord rec;
    memset(&rec, 0xFF, sizeof(rec)); // flash erases to 0xFF
    rec.magic   = FLASH_MAGIC;
    rec.length  = (uint8_t)(len > MAX_JUNCTIONS ? MAX_JUNCTIONS : len);
    rec.time_ms = time_ms;
    memcpy(rec.decisions, path, rec.length);

    // Pad to multiple of FLASH_PAGE_SIZE (256 bytes)
    uint8_t page_buf[FLASH_SECTOR_SIZE];
    memset(page_buf, 0xFF, FLASH_SECTOR_SIZE);
    memcpy(page_buf, &rec, sizeof(PathRecord));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_PATH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_PATH_OFFSET, page_buf, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

static bool flash_load_path(Direction *path_out, int *len_out, uint32_t *time_out) {
    const PathRecord *rec = (const PathRecord *)FLASH_PATH_ADDR;
    if (rec->magic != FLASH_MAGIC) return false;
    if (rec->length == 0 || rec->length > MAX_JUNCTIONS) return false;
    *len_out  = rec->length;
    *time_out = rec->time_ms;
    memcpy(path_out, rec->decisions, rec->length);
    return true;
}

static void flash_erase_path() {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_PATH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

// ================================================================
//  LEFT-HAND RULE  (exploration decision)
//  Priority: LEFT > STRAIGHT > RIGHT > BACK
// ================================================================
static Direction left_hand_rule(JunctionType jt) {
    switch (jt) {
        case JT_LEFT:
        case JT_T_BOTH:
        case JT_CROSS:
            return DIR_LEFT;
        case JT_RIGHT:
            return DIR_RIGHT;
        case JT_DEAD_END:
            return DIR_BACK;
        default:
            return DIR_STRAIGHT;
    }
}

// ================================================================
//  PATH OPTIMISATION
//  The Left-Hand Rule produces a sequence like:
//    L B R S L B S ...
//  Any  X B Y  triple means "went X, hit dead-end, came back, took Y"
//  → replace the triple with the single turn that avoids the dead end.
//
//  Replacement table (standard maze solver simplification):
//    L B L  →  S       (went left, dead-end, went left again = straight)
//    L B S  →  R       (went left, dead-end, straight = right)
//    L B R  →  B       (went left, dead-end, right = full U-turn)
//    S B S  →  B
//    S B L  →  R
//    R B L  →  S  ... etc.
// ================================================================

static Direction simplify_triple(Direction a, Direction /*b*/, Direction c) {
    // b is always DIR_BACK; we don't need it
    // Table: (a, c) → replacement
    if (a == DIR_LEFT  && c == DIR_LEFT)     return DIR_STRAIGHT;
    if (a == DIR_LEFT  && c == DIR_STRAIGHT) return DIR_RIGHT;
    if (a == DIR_LEFT  && c == DIR_RIGHT)    return DIR_BACK;
    if (a == DIR_STRAIGHT && c == DIR_STRAIGHT) return DIR_BACK;
    if (a == DIR_STRAIGHT && c == DIR_LEFT)  return DIR_RIGHT;
    if (a == DIR_STRAIGHT && c == DIR_RIGHT) return DIR_BACK;
    if (a == DIR_RIGHT && c == DIR_LEFT)     return DIR_STRAIGHT;
    if (a == DIR_RIGHT && c == DIR_STRAIGHT) return DIR_LEFT;
    if (a == DIR_RIGHT && c == DIR_RIGHT)    return DIR_BACK;
    return c; // fallback
}

static void optimise_path(const Direction *src, int src_len,
                           Direction *dst, int *dst_len) {
    // Copy src → dst first
    int len = 0;
    for (int i = 0; i < src_len; i++) {
        dst[len++] = src[i];
    }
    // Repeatedly scan for  X B Y  pattern and collapse
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i + 2 < len; i++) {
            if (dst[i+1] == DIR_BACK) {
                Direction rep = simplify_triple(dst[i], dst[i+1], dst[i+2]);
                // Remove i, i+1, i+2 and insert rep at i
                dst[i] = rep;
                // Shift left
                for (int j = i+1; j < len-2; j++) dst[j] = dst[j+2];
                len -= 2;
                changed = true;
                break; // restart scan
            }
        }
    }
    *dst_len = len;
}

// ================================================================
//  PWM / MOTOR
// ================================================================
static void pwm_init_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);
    pwm_set_enabled(slice, true);
}

static void motor_set(uint in1, uint in2, uint pwm_pin, int speed) {
    if (speed >= 0) { gpio_put(in1,1); gpio_put(in2,0); }
    else            { gpio_put(in1,0); gpio_put(in2,1); speed=-speed; }
    if (speed > 255) speed = 255;
    pwm_set_gpio_level(pwm_pin, (uint16_t)speed);
}

static inline void motors_drive(int left, int right) {
    motor_set(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM, left);
    motor_set(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM, right);
}
static inline void motors_stop()   { motors_drive(0,0); gpio_put(MOTOR_STBY,0); }
static inline void motors_enable() { gpio_put(MOTOR_STBY,1); }

// ── U-turn manoeuvre (in-place 180°)
static void execute_uturn() {
    // Spin right until line found on centre sensors
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    motors_drive(BASE_SPEED, -BASE_SPEED);
    while (to_ms_since_boot(get_absolute_time()) - t0 < 1200) {
        // Wait for centre sensors (6-9) to see the line again
        bool c1 = gpio_get(SENSOR_PINS[7]);
        bool c2 = gpio_get(SENSOR_PINS[8]);
        if (c1 && c2) break;
        sleep_us(200);
    }
    motors_drive(0,0);
    sleep_ms(50);
}

// ── Execute a single junction direction command
static void execute_junction_turn(Direction d) {
    switch (d) {
        case DIR_LEFT:
            motors_drive(-BASE_SPEED, BASE_SPEED);
            sleep_ms(300); // pivot ~90° — tune per robot
            break;
        case DIR_RIGHT:
            motors_drive(BASE_SPEED, -BASE_SPEED);
            sleep_ms(300);
            break;
        case DIR_BACK:
            execute_uturn();
            break;
        case DIR_STRAIGHT:
        default:
            // Just keep going — PID handles re-centering
            break;
    }
}

// ================================================================
//  SENSOR READING
// ================================================================
struct SensorReading {
    uint16_t raw;
    int      active;
    float    position;
    bool     all_white;
    bool     all_black;
    bool     sharp_left;
    bool     sharp_right;
    bool     has_left;
    bool     has_right;
    bool     has_forward;
};

#define SENSOR_ALL_BLACK_N   14
#define SENSOR_ALL_WHITE_N    1
#define SENSOR_SHARP_N        6

static SensorReading read_sensors() {
    SensorReading s = {};
    long wsum = 0;
    for (int i = 0; i < 16; i++) {
        bool on = gpio_get(SENSOR_PINS[i]);
        if (on) { s.raw |= (1u<<i); s.active++; wsum += SENSOR_WEIGHT[i]; }
    }
    s.all_black = (s.active >= SENSOR_ALL_BLACK_N);
    s.all_white = (s.active <= SENSOR_ALL_WHITE_N);
    if (s.active > 0) s.position = (float)wsum / (float)s.active;

    int lc=0, rc=0;
    for (int i=0; i<8;  i++) if ((s.raw>>i)&1) lc++;
    for (int i=8; i<16; i++) if ((s.raw>>i)&1) rc++;
    s.sharp_left  = (rc >= SENSOR_SHARP_N && lc == 0);
    s.sharp_right = (lc >= SENSOR_SHARP_N && rc == 0);

    // Forward = centre 4 sensors (S6-S9)
    s.has_forward = ((s.raw >> 6) & 0xF) != 0;
    // Left branch  = outermost left sensors (S0-S3)
    s.has_left    = ((s.raw) & 0xF) != 0;
    // Right branch = outermost right sensors (S12-S15)
    s.has_right   = ((s.raw >> 12) & 0xF) != 0;
    return s;
}

// ── Classify junction type from sensor reading
static JunctionType classify_junction(const SensorReading &s) {
    if (s.all_black)               return JT_FINISH_BAR;
    if (s.all_white)               return JT_DEAD_END;
    bool L = s.has_left;
    bool F = s.has_forward;
    bool R = s.has_right;
    if (L && F && R)               return JT_T_BOTH;
    if (L && R && !F)              return JT_CROSS; // treat as T_BOTH
    if (L && !F && !R)             return JT_LEFT;
    if (R && !F && !L)             return JT_RIGHT;
    if (!L && !F && !R)            return JT_DEAD_END;
    return JT_NONE;
}

// ── Junction debounce: must see junction for ≥ 40 ms to confirm
static JunctionType debounce_junction(const SensorReading &s) {
    static JunctionType last_jt   = JT_NONE;
    static uint32_t     first_ms  = 0;
    JunctionType jt = classify_junction(s);
    if (jt != JT_NONE && jt != JT_FINISH_BAR) {
        if (jt != last_jt) { last_jt = jt; first_ms = to_ms_since_boot(get_absolute_time()); }
        if (to_ms_since_boot(get_absolute_time()) - first_ms >= 40) return jt;
    } else {
        last_jt  = JT_NONE;
        first_ms = 0;
    }
    return JT_NONE;
}

// ── Finish line detection (types A, B, and C)
// Type A: all_black held ≥ 120 ms
// Type B: T-end detected after robot has crossed most of the track
// Type C: implied by reaching DIR_FINISH token in replay
#define FINISH_HOLD_MS   120
#define FINISH_MIN_RUN   600

static uint32_t finish_ts = 0;

static bool check_finish(const SensorReading &s, uint32_t elapsed) {
    if (elapsed < FINISH_MIN_RUN) return false;
    // Type A
    if (s.all_black) {
        if (!finish_ts) finish_ts = to_ms_since_boot(get_absolute_time());
        if (to_ms_since_boot(get_absolute_time()) - finish_ts >= FINISH_HOLD_MS) return true;
    } else {
        finish_ts = 0;
    }
    // Type B: T-junction at very end (no forward, both sides active)
    if (s.has_left && s.has_right && !s.has_forward && elapsed > FINISH_MIN_RUN)
        return true;
    return false;
}

// ================================================================
//  SSD1306 OLED DRIVER (minimal, no external lib)
// ================================================================
#define OLED_W     128
#define OLED_H      64
#define OLED_PAGES  (OLED_H/8)

static uint8_t oled_buf[OLED_W * OLED_PAGES];

static void oled_cmd(uint8_t c) {
    uint8_t b[2] = {0x00, c};
    i2c_write_blocking(I2C_PORT, OLED_ADDR, b, 2, false);
}
static void oled_init() {
    const uint8_t seq[] = {
        0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,
        0x8D,0x14,0x20,0x00,0xA1,0xC8,0xDA,0x12,
        0x81,0xCF,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF
    };
    for (auto b : seq) oled_cmd(b);
}
static void oled_flush() {
    oled_cmd(0x21); oled_cmd(0); oled_cmd(127);
    oled_cmd(0x22); oled_cmd(0); oled_cmd(7);
    for (int p = 0; p < OLED_PAGES; p++) {
        uint8_t tmp[129]; tmp[0]=0x40;
        memcpy(&tmp[1], &oled_buf[p*OLED_W], OLED_W);
        i2c_write_blocking(I2C_PORT, OLED_ADDR, tmp, 129, false);
    }
}
static void oled_clear() { memset(oled_buf,0,sizeof(oled_buf)); }
static void oled_pixel(int x, int y, bool on) {
    if (x<0||x>=OLED_W||y<0||y>=OLED_H) return;
    if (on) oled_buf[(y/8)*OLED_W+x] |=  (1<<(y%8));
    else    oled_buf[(y/8)*OLED_W+x] &= ~(1<<(y%8));
}

// 5×7 font
static const uint8_t FONT[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},
    {0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},
    {0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},
    {0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},
    {0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},
    {0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},
    {0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x40,0x7C},
    {0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},
};

static void oled_char(int x, int y, char c) {
    if (c < 32 || c > 127) c = '?';
    const uint8_t *g = FONT[c-32];
    for (int col=0;col<5;col++) {
        uint8_t ln = g[col];
        for (int row=0;row<7;row++) oled_pixel(x+col,y+row,(ln>>row)&1);
    }
}
static void oled_str(int x, int y, const char *s) {
    while (*s) { oled_char(x,y,*s++); x+=6; }
}

// ================================================================
//  BUTTON  (debounced, non-blocking)
// ================================================================
#define DEBOUNCE_MS  50
static const uint BTN_PINS[4] = {BTN_SEL,BTN_UP,BTN_DOWN,BTN_START};
static bool     btn_last[4]   = {true,true,true,true};
static uint32_t btn_time[4]   = {};

static bool btn_pressed(int id) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    bool cur = gpio_get(BTN_PINS[id]);
    if (!cur && btn_last[id] && (now-btn_time[id])>DEBOUNCE_MS) {
        btn_last[id]=false; btn_time[id]=now; return true;
    }
    if (cur) btn_last[id]=true;
    return false;
}

// ================================================================
//  TUNE MENU
// ================================================================
enum TuneParam { TP_KP=0, TP_KI, TP_KD, TP_SPD, TP_ERASE, TP_COUNT };
static const char *TP_LABEL[TP_COUNT] = {"Kp","Ki","Kd","SPD","ERASE"};
static int tune_sel = 0;

static void tune_adjust(int d) {
    switch(tune_sel) {
        case TP_KP:  Kp += d*KP_STEP;  if(Kp<0)Kp=0; break;
        case TP_KI:  Ki += d*KI_STEP;  if(Ki<0)Ki=0; break;
        case TP_KD:  Kd += d*KD_STEP;  if(Kd<0)Kd=0; break;
        case TP_SPD: BASE_SPEED+=d*SPD_STEP;
                     if(BASE_SPEED<0)BASE_SPEED=0;
                     if(BASE_SPEED>255)BASE_SPEED=255; break;
        case TP_ERASE:
            if(d>0) { flash_erase_path(); explore_len=0; optimal_len=0;
                      run_mode=MODE_EXPLORE; }
            break;
    }
}

// ── float → "X.XXX" string (no sprintf %f on bare metal)
static void ftoa3(float v, char *out) {
    if (v < 0) { *out++='-'; v=-v; }
    int w = (int)v;
    int f = (int)((v-(float)w)*1000.0f+0.5f);
    if (f>=1000){w++;f-=1000;}
    char tmp[8]; int ti=0;
    if(w==0){tmp[ti++]='0';}
    else{int ww=w; while(ww>0){tmp[ti++]='0'+ww%10;ww/=10;}
         // reverse
         for(int a=0,b=ti-1;a<b;a++,b--){char t=tmp[a];tmp[a]=tmp[b];tmp[b]=t;}}
    for(int i=0;i<ti;i++) *out++=tmp[i];
    *out++='.';
    *out++='0'+f/100;
    *out++='0'+(f/10)%10;
    *out++='0'+f%10;
    *out=0;
}

// ================================================================
//  OLED SCREENS
// ================================================================
static void render_idle(bool has_flash, uint32_t best_ms) {
    oled_clear();
    oled_str(0, 0, run_mode==MODE_EXPLORE ? "MODE: EXPLORE" : "MODE: REPLAY");
    char buf[24];
    if (has_flash) {
        uint32_t m=best_ms/60000, s=(best_ms%60000)/1000, ms=best_ms%1000;
        snprintf(buf,sizeof(buf),"BEST %02u:%02u.%03u",(unsigned)m,(unsigned)s,(unsigned)ms);
        oled_str(0, 10, buf);
    } else {
        oled_str(0, 10, "No path saved");
    }
    // Tune params
    for (int i=0;i<TP_COUNT;i++) {
        int y=20+i*9;
        if(i==tune_sel) oled_str(0,y,">");
        if(i<3){
            char fv[10]; ftoa3(i==0?Kp:i==1?Ki:Kd,fv);
            snprintf(buf,sizeof(buf)," %s %s",TP_LABEL[i],fv);
        } else if(i==TP_SPD) {
            snprintf(buf,sizeof(buf)," %s %d",TP_LABEL[i],BASE_SPEED);
        } else {
            snprintf(buf,sizeof(buf)," %s [UP=yes]",TP_LABEL[i]);
        }
        oled_str(0,y,buf);
    }
    oled_flush();
}

static void render_run(uint32_t elapsed_ms, float error, int junctions) {
    oled_clear();
    char buf[24];
    uint32_t m=elapsed_ms/60000,s=(elapsed_ms%60000)/1000,ms=elapsed_ms%1000;
    snprintf(buf,sizeof(buf),"T%02u:%02u.%03u %s",
             (unsigned)m,(unsigned)s,(unsigned)ms,
             run_mode==MODE_EXPLORE?"EXP":"OPT");
    oled_str(0,0,buf);
    char kpv[10],kiv[10],kdv[10];
    ftoa3(Kp,kpv); ftoa3(Ki,kiv); ftoa3(Kd,kdv);
    snprintf(buf,sizeof(buf),"Kp%s",kpv); oled_str(0,11,buf);
    snprintf(buf,sizeof(buf),"Ki%s",kiv); oled_str(0,20,buf);
    snprintf(buf,sizeof(buf),"Kd%s",kdv); oled_str(0,29,buf);
    snprintf(buf,sizeof(buf),"J%d S%d E%d",junctions,BASE_SPEED,(int)error);
    oled_str(0,38,buf);
    // sensor bar
    {
        SensorReading sr = read_sensors();
        for(int i=0;i<16;i++){
            bool on=(sr.raw>>i)&1;
            oled_pixel(0+i*8,55,on); oled_pixel(1+i*8,55,on);
            oled_pixel(0+i*8,56,on); oled_pixel(1+i*8,56,on);
        }
    }
    oled_flush();
}

static void render_finish(uint32_t t_ms, bool is_new_best) {
    oled_clear();
    oled_str(22,0,"** FINISH **");
    char buf[24];
    uint32_t m=t_ms/60000,s=(t_ms%60000)/1000,ms=t_ms%1000;
    snprintf(buf,sizeof(buf),"%02u:%02u.%03u",(unsigned)m,(unsigned)s,(unsigned)ms);
    oled_str(18,16,buf);
    if(is_new_best) oled_str(10,30,"NEW BEST!");
    oled_str(0,44,run_mode==MODE_EXPLORE?"Path saved!":"Optimal run");
    oled_str(0,54,"START=menu");
    oled_flush();
}

// ================================================================
//  MAIN
// ================================================================
int main() {
    stdio_init_all();

    // Sensors
    for(int i=0;i<16;i++){
        gpio_init(SENSOR_PINS[i]);
        gpio_set_dir(SENSOR_PINS[i],GPIO_IN);
        gpio_pull_down(SENSOR_PINS[i]);
    }
    // Motor dir pins
    const uint mpins[]={MOTOR_A_IN1,MOTOR_A_IN2,MOTOR_B_IN1,MOTOR_B_IN2,MOTOR_STBY};
    for(auto p:mpins){gpio_init(p);gpio_set_dir(p,GPIO_OUT);gpio_put(p,0);}
    pwm_init_pin(MOTOR_A_PWM);
    pwm_init_pin(MOTOR_B_PWM);

    // I2C + OLED
    i2c_init(I2C_PORT,400000);
    gpio_set_function(I2C_SDA,GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL,GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA); gpio_pull_up(I2C_SCL);
    sleep_ms(120);
    oled_init(); oled_clear();
    oled_str(15,24,"LFR  v2.0"); oled_flush();
    sleep_ms(1200);

    // Buttons
    for(int i=0;i<4;i++){
        gpio_init(BTN_PINS[i]);
        gpio_set_dir(BTN_PINS[i],GPIO_IN);
        gpio_pull_up(BTN_PINS[i]);
    }

    // ── Try to load saved path from flash
    bool     has_flash   = false;
    uint32_t flash_time  = 0;
    {
        Direction loaded[MAX_JUNCTIONS];
        int       llen = 0;
        if (flash_load_path(loaded, &llen, &flash_time)) {
            memcpy(optimal_path, loaded, llen);
            optimal_len = llen;
            has_flash   = true;
            run_mode    = MODE_REPLAY; // default to replay if path exists
        }
    }

    // ── State machine
    enum State { ST_IDLE, ST_RUNNING, ST_FINISHED };
    State state = ST_IDLE;

    // PID state
    float pid_integral   = 0.0f;
    float pid_prev_error = 0.0f;
    float last_position  = 0.0f;
    float current_error  = 0.0f;

    // Run tracking
    uint32_t run_start_ms = 0;
    uint32_t final_time   = 0;
    int      junction_cnt = 0;
    bool     new_best     = false;

    // Loop timing
    uint32_t last_loop_us = time_us_32();
    uint32_t last_oled_ms = 0;
    #define OLED_PERIOD  100

    // Junction cooldown: after acting on a junction, ignore
    // sensor patterns for 400 ms to let robot clear the intersection
    uint32_t junction_cooldown_until = 0;

    while (true) {
        uint32_t now_us = time_us_32();
        float dt = (float)(now_us - last_loop_us) * 1e-6f;
        if (dt <= 0.0f || dt > 0.05f) dt = 0.001f;
        last_loop_us = now_us;
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // ──────────────────────────────────────────────────────
        //  BUTTON HANDLING (all states)
        // ──────────────────────────────────────────────────────
        if (btn_pressed(0)) { // SEL
            tune_sel = (tune_sel+1) % TP_COUNT;
        }
        if (btn_pressed(1)) tune_adjust(+1); // UP
        if (btn_pressed(2)) tune_adjust(-1); // DOWN

        // ──────────────────────────────────────────────────────
        //  ST_IDLE
        // ──────────────────────────────────────────────────────
        if (state == ST_IDLE) {
            motors_stop();
            if (now_ms - last_oled_ms > OLED_PERIOD) {
                render_idle(has_flash, flash_time);
                last_oled_ms = now_ms;
            }
            if (btn_pressed(3)) { // START
                // Reset PID
                pid_integral = pid_prev_error = last_position = 0.0f;
                finish_ts    = 0;
                junction_cnt = 0;
                new_best     = false;
                junction_cooldown_until = 0;

                if (run_mode == MODE_EXPLORE) {
                    explore_len = 0;
                    memset(explore_path, DIR_NONE, sizeof(explore_path));
                } else {
                    replay_idx = 0;
                }

                run_start_ms = now_ms;
                motors_enable();
                state = ST_RUNNING;
            }
        }

        // ──────────────────────────────────────────────────────
        //  ST_RUNNING
        // ──────────────────────────────────────────────────────
        else if (state == ST_RUNNING) {
            uint32_t elapsed = now_ms - run_start_ms;
            SensorReading s  = read_sensors();

            // ── Manual stop
            if (btn_pressed(3)) {
                motors_stop();
                final_time = elapsed;
                new_best   = false;
                state      = ST_FINISHED;
                render_finish(final_time, false);
                last_oled_ms = now_ms;
                continue;
            }

            // ── Finish detection (types A + B)
            if (check_finish(s, elapsed)) {
                motors_stop();
                final_time = elapsed;

                if (run_mode == MODE_EXPLORE) {
                    // Record finish token
                    if (explore_len < MAX_JUNCTIONS)
                        explore_path[explore_len++] = DIR_FINISH;
                    // Optimise and save
                    optimise_path(explore_path, explore_len,
                                  optimal_path, &optimal_len);
                    flash_save_path(optimal_path, optimal_len, final_time);
                    has_flash   = true;
                    flash_time  = final_time;
                    run_mode    = MODE_REPLAY;
                    new_best    = true;
                } else {
                    // Replay finished — check if new best
                    new_best = (final_time < flash_time);
                    if (new_best) {
                        flash_time = final_time;
                        flash_save_path(optimal_path, optimal_len, final_time);
                    }
                }

                state = ST_FINISHED;
                render_finish(final_time, new_best);
                last_oled_ms = now_ms;
                continue;
            }

            // ── POSITION / ERROR
            float position;
            if (s.all_white) {
                position = last_position;
            } else if (s.all_black) {
                position = 0.0f;
                pid_integral = 0.0f;
            } else {
                position = s.position;
                last_position = position;
            }
            float error = position;
            current_error = error;

            // ── PID
            pid_integral += error * dt;
            if (pid_integral >  MAX_INTEGRAL) pid_integral =  MAX_INTEGRAL;
            if (pid_integral < -MAX_INTEGRAL) pid_integral = -MAX_INTEGRAL;
            float derivative   = (error - pid_prev_error) / dt;
            pid_prev_error     = error;
            float correction   = Kp*error + Ki*pid_integral + Kd*derivative;

            // ── JUNCTION HANDLING
            bool in_cooldown = (now_ms < junction_cooldown_until);
            JunctionType jt  = in_cooldown ? JT_NONE : debounce_junction(s);

            if (jt != JT_NONE) {
                Direction decision;

                if (run_mode == MODE_EXPLORE) {
                    // Left-Hand Rule
                    decision = left_hand_rule(jt);
                    if (explore_len < MAX_JUNCTIONS)
                        explore_path[explore_len++] = decision;
                } else {
                    // Follow optimal path
                    if (replay_idx < optimal_len)
                        decision = optimal_path[replay_idx++];
                    else
                        decision = DIR_STRAIGHT;
                }

                junction_cnt++;
                execute_junction_turn(decision);
                pid_integral = 0.0f;   // reset integral after turn
                junction_cooldown_until = now_ms + 400;
                continue;
            }

            // ── MOTOR OUTPUT
            int left_spd, right_spd;
            if (s.sharp_left) {
                left_spd = -BASE_SPEED; right_spd = BASE_SPEED;
            } else if (s.sharp_right) {
                left_spd = BASE_SPEED;  right_spd = -BASE_SPEED;
            } else {
                left_spd  = (int)(BASE_SPEED + correction);
                right_spd = (int)(BASE_SPEED - correction);
                if (left_spd  >  255) left_spd  =  255;
                if (left_spd  < -255) left_spd  = -255;
                if (right_spd >  255) right_spd =  255;
                if (right_spd < -255) right_spd = -255;
            }
            motors_drive(left_spd, right_spd);

            // ── OLED
            if (now_ms - last_oled_ms > OLED_PERIOD) {
                render_run(elapsed, current_error, junction_cnt);
                last_oled_ms = now_ms;
            }
        }

        // ──────────────────────────────────────────────────────
        //  ST_FINISHED
        // ──────────────────────────────────────────────────────
        else {
            motors_stop();
            if (now_ms - last_oled_ms > OLED_PERIOD) {
                render_finish(final_time, new_best);
                last_oled_ms = now_ms;
            }
            if (btn_pressed(3)) state = ST_IDLE;
        }

        sleep_us(200);
    }
}

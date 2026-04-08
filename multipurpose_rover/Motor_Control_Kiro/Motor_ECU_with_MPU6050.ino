/************************************************************
 * ESP32 Motor Driver ECU — FINAL STABLE VERSION (v4.x core)
 * MPU6050 IMU (replaces ADXL345)
 *
 * CAN Frame Map:
 *   0x101 (RX) — Command:    y[0-1], x[2-3], speed[4], enable[5], mode[6]
 *   0x200 (TX) — IMU data:   ax[0-1], ay[2-3], az[4-5], gx[6-7]  (×100, int16 big-endian)
 *   0x201 (TX) — Encoders:   M1[0-1], M4[2-3]  (int16 big-endian)
 *   0x202 (TX) — IMU gyro 2: gy[0-1], gz[2-3]  (×100, int16 big-endian)
 *   0x300 (TX) — Fault:      faultStatus[0]
 ************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "driver/twai.h"
#include "driver/pcnt.h"

/* ═══════════════════════════════════════════════════════════
 * MOTOR PINS
 * ═══════════════════════════════════════════════════════════ */
#define MOTOR1_RPWM 27   // rear  right  RPWM
#define MOTOR1_LPWM 26   // rear  right  LPWM
#define MOTOR2_RPWM 33   // front right  RPWM
#define MOTOR2_LPWM 32   // front right  LPWM
#define MOTOR3_RPWM 25   // front left   RPWM
#define MOTOR3_LPWM 14   // front left   LPWM
#define MOTOR4_RPWM 12   // rear  left   RPWM
#define MOTOR4_LPWM 13   // rear  left   LPWM

#define PWM_FREQ   5000
#define PWM_RES       8
#define MAX_PWM     255
#define MIN_PWM      30

/* ═══════════════════════════════════════════════════════════
 * ENCODER PINS  (quadrature, PCNT)
 * ═══════════════════════════════════════════════════════════ */
#define M1_ENA  23
#define M1_ENB  15
#define M4_ENA  19
#define M4_ENB  18
#define ENC_M1_UNIT  PCNT_UNIT_0
#define ENC_M4_UNIT  PCNT_UNIT_1

/* ═══════════════════════════════════════════════════════════
 * MPU6050 — I²C address
 * ═══════════════════════════════════════════════════════════ */
#define MPU6050_ADDR        0x68   // AD0 = GND → 0x68 ; AD0 = VCC → 0x69
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_SMPLRT_DIV  0x19
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B  // first of 14 consecutive registers
#define MPU6050_WHO_AM_I    0x75

// Sensitivity divisors (full-scale defaults after init below)
// Accel FS = ±2 g  → 16384 LSB/g  → divide by 16384 to get g, ×9.81 for m/s²
// Gyro  FS = ±250°/s → 131 LSB/°/s → divide by 131 for °/s
#define MPU6050_ACCEL_SCALE  16384.0f   // LSB per g
#define MPU6050_GYRO_SCALE     131.0f   // LSB per deg/s
#define GRAVITY              9.80665f

/* ═══════════════════════════════════════════════════════════
 * CAN IDs
 * ═══════════════════════════════════════════════════════════ */
#define CAN_TX_PIN      GPIO_NUM_5
#define CAN_RX_PIN      GPIO_NUM_4
#define CAN_ID_CMD      0x101   // receive commands
#define CAN_ID_IMU      0x200   // transmit ax, ay, az, gx
#define CAN_ID_ENCODER  0x201   // transmit M1 / M4 encoder ticks
#define CAN_ID_IMU2     0x202   // transmit gy, gz
#define CAN_ID_FAULT    0x300   // transmit fault byte

/* ═══════════════════════════════════════════════════════════
 * FAULT FLAGS
 * ═══════════════════════════════════════════════════════════ */
#define FAULT_CAN_RX_TIMEOUT  0
#define FAULT_MPU6050_ERROR   1
#define FAULT_MOTOR_FAULT     2

/* ═══════════════════════════════════════════════════════════
 * SHARED DATA STRUCTURES
 * ═══════════════════════════════════════════════════════════ */
typedef struct {
    int16_t x;
    int16_t y;
    uint8_t speed;
    uint8_t enable;
    uint8_t mode;
} ControlData_t;

typedef struct {
    float ax, ay, az;   // m/s²
    float gx, gy, gz;   // °/s
} ImuData_t;

ControlData_t   ctrl     = {0};
ImuData_t       imu_data = {0};
SemaphoreHandle_t ctrlMutex;
SemaphoreHandle_t imuMutex;

uint32_t last_rx_time = 0;
uint8_t  faultStatus  = 0;

/* ═══════════════════════════════════════════════════════════
 * MPU6050 HELPERS
 * ═══════════════════════════════════════════════════════════ */

/**
 * Write one byte to an MPU6050 register.
 */
static void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

/**
 * Read `len` consecutive bytes starting at `reg` into `buf`.
 * Returns true on success.
 */
static bool mpu_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t got = Wire.requestFrom((uint8_t)MPU6050_ADDR, len);
    if (got != len) return false;

    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

/**
 * Combine two consecutive bytes (big-endian) into a signed 16-bit value.
 */
static inline int16_t be16(const uint8_t *b) {
    return (int16_t)((b[0] << 8) | b[1]);
}

/**
 * Initialise the MPU6050.
 * Wakes the chip, sets 1 kHz sample rate / 42 Hz DLPF,
 * ±2 g accelerometer, ±250 °/s gyroscope.
 * Returns true if the WHO_AM_I register matches 0x68.
 */
static bool mpu_init() {
    // --- WHO_AM_I sanity check ---
    uint8_t who = 0;
    if (!mpu_read(MPU6050_WHO_AM_I, &who, 1)) return false;
    if (who != 0x68 && who != 0x72) return false;   // 0x72 = MPU6050C variant

    // Wake from sleep, use internal 8 MHz oscillator
    mpu_write(MPU6050_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Sample rate divider: SMPLRT_DIV = 7 → 1000 / (1+7) = 125 Hz
    mpu_write(MPU6050_SMPLRT_DIV, 0x07);

    // DLPF config = 3 → 42 Hz bandwidth, 4.8 ms delay (good for a rover)
    mpu_write(MPU6050_CONFIG, 0x03);

    // Gyro full-scale = ±250 °/s  (bits [4:3] = 00)
    mpu_write(MPU6050_GYRO_CONFIG, 0x00);

    // Accel full-scale = ±2 g  (bits [4:3] = 00)
    mpu_write(MPU6050_ACCEL_CONFIG, 0x00);

    return true;
}

/**
 * Read all 6 axes in one 14-byte burst (accel + temp + gyro).
 * Converts raw integers to SI units:
 *   accel: m/s²   (raw / 16384 × 9.80665)
 *   gyro:  °/s    (raw / 131)
 * Returns false if the I²C transaction fails.
 */
static bool mpu_read_all(ImuData_t &out) {
    uint8_t raw[14];
    if (!mpu_read(MPU6050_ACCEL_XOUT_H, raw, 14)) return false;

    // raw[0-1]=AX, [2-3]=AY, [4-5]=AZ, [6-7]=TEMP, [8-9]=GX, [10-11]=GY, [12-13]=GZ
    out.ax = (be16(raw + 0)  / MPU6050_ACCEL_SCALE) * GRAVITY;
    out.ay = (be16(raw + 2)  / MPU6050_ACCEL_SCALE) * GRAVITY;
    out.az = (be16(raw + 4)  / MPU6050_ACCEL_SCALE) * GRAVITY;
    out.gx =  be16(raw + 8)  / MPU6050_GYRO_SCALE;
    out.gy =  be16(raw + 10) / MPU6050_GYRO_SCALE;
    out.gz =  be16(raw + 12) / MPU6050_GYRO_SCALE;
    return true;
}

/* ═══════════════════════════════════════════════════════════
 * MOTOR FUNCTIONS
 * ═══════════════════════════════════════════════════════════ */
static void motor_init() {
    ledcAttach(MOTOR1_RPWM, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR1_LPWM, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR2_RPWM, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR2_LPWM, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR3_RPWM, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR3_LPWM, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR4_RPWM, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR4_LPWM, PWM_FREQ, PWM_RES);
}

static void set_motor(uint8_t motor, int8_t dir, uint8_t pwm) {
    uint8_t r, l;
    switch (motor) {
        case 1: r = MOTOR1_RPWM; l = MOTOR1_LPWM; break;
        case 2: r = MOTOR2_RPWM; l = MOTOR2_LPWM; break;
        case 3: r = MOTOR3_RPWM; l = MOTOR3_LPWM; break;
        default:r = MOTOR4_RPWM; l = MOTOR4_LPWM; break;
    }
    if      (dir > 0) { ledcWrite(r, pwm); ledcWrite(l,   0); }
    else if (dir < 0) { ledcWrite(r,   0); ledcWrite(l, pwm); }
    else              { ledcWrite(r,   0); ledcWrite(l,   0); }
}

static void stop_all() {
    for (int m = 1; m <= 4; m++) set_motor(m, 0, 0);
}

/* ═══════════════════════════════════════════════════════════
 * ENCODER FUNCTIONS
 * ═══════════════════════════════════════════════════════════ */
static void encoder_init(pcnt_unit_t unit, int pin_a, int pin_b) {
    pcnt_config_t cfg = {
        .pulse_gpio_num = pin_a,
        .ctrl_gpio_num  = pin_b,
        .lctrl_mode     = PCNT_MODE_REVERSE,
        .hctrl_mode     = PCNT_MODE_KEEP,
        .pos_mode       = PCNT_COUNT_INC,
        .neg_mode       = PCNT_COUNT_DEC,
        .counter_h_lim  =  32767,
        .counter_l_lim  = -32767,
        .unit           = unit,
        .channel        = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&cfg);
    pcnt_filter_enable(unit);
    pcnt_set_filter_value(unit, 100);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

/* ═══════════════════════════════════════════════════════════
 * CAN FUNCTIONS
 * ═══════════════════════════════════════════════════════════ */
static void can_send(uint32_t id, uint8_t *data, uint8_t dlc = 8) {
    twai_message_t tx;
    tx.identifier       = id;
    tx.data_length_code = dlc;
    tx.flags            = 0;
    memcpy(tx.data, data, dlc);
    twai_transmit(&tx, pdMS_TO_TICKS(10));
}

static void can_init() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g, &t, &f);
    twai_start();
}

/* ═══════════════════════════════════════════════════════════
 * TASK: CAN RX
 * Priority 3, Core 0
 * Receives 0x101 command frames and updates ctrl struct.
 * Watchdog: forces enable=0 if no frame for 500 ms.
 * ═══════════════════════════════════════════════════════════ */
void can_rx_task(void *arg) {
    twai_message_t rx;
    while (1) {
        if (twai_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK) {
            if (rx.identifier == CAN_ID_CMD && rx.data_length_code == 8) {
                if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) {
                    ctrl.y      = (int16_t)((rx.data[0] << 8) | rx.data[1]);
                    ctrl.x      = (int16_t)((rx.data[2] << 8) | rx.data[3]);
                    ctrl.speed  = rx.data[4];
                    ctrl.enable = rx.data[5];
                    ctrl.mode   = rx.data[6];
                    xSemaphoreGive(ctrlMutex);
                }
                last_rx_time = millis();
                faultStatus &= ~(1 << FAULT_CAN_RX_TIMEOUT);
            }
        } else {
            if (millis() - last_rx_time > 500) {
                faultStatus |= (1 << FAULT_CAN_RX_TIMEOUT);
            }
        }

        // Watchdog independent of receive path
        if (millis() - last_rx_time > 500) {
            if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) {
                ctrl.enable = 0;
                ctrl.x      = 0;
                ctrl.y      = 0;
                xSemaphoreGive(ctrlMutex);
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════
 * TASK: MOTOR CONTROL
 * Priority 3, Core 1
 * Differential skid-steer mixing with mode scaling.
 * Motor layout:
 *   M1 = rear right,  M2 = front right  (right side group)
 *   M3 = front left,  M4 = rear left    (left side group)
 * M3/M4 polarity inverted in firmware to match wiring.
 * ═══════════════════════════════════════════════════════════ */
void control_task(void *arg) {
    while (1) {
        ControlData_t c;
        if (!xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        c = ctrl;
        xSemaphoreGive(ctrlMutex);

        if (!c.enable) {
            stop_all();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // mode_scale: 0=Eco(0.3×), 1=Normal(0.6×), 2=Sport(1.0×)
        float mode_scale = (c.mode == 0) ? 0.3f : (c.mode == 1) ? 0.6f : 1.0f;

        float throttle = (c.x / 100.0f) * c.speed * mode_scale;
        float turn     = -(c.y / 100.0f) * c.speed * mode_scale;

        int16_t left  = (int16_t)constrain(throttle + turn, -100.0f, 100.0f);
        int16_t right = (int16_t)constrain(throttle - turn, -100.0f, 100.0f);

        // Scale to PWM range
        left  = (int16_t)((left  / 100.0f) * MAX_PWM);
        right = (int16_t)((right / 100.0f) * MAX_PWM);

        // Enforce minimum drive threshold (prevents stall hum at low values)
        auto apply_min = [](int16_t v) -> int16_t {
            if (v > 0 && v < MIN_PWM)  return MIN_PWM;
            if (v < 0 && v > -MIN_PWM) return -MIN_PWM;
            return v;
        };
        left  = apply_min(left);
        right = apply_min(right);

        // Hard zero dead-zone
        if (abs(left)  < 10) left  = 0;
        if (abs(right) < 10) right = 0;

        // Right side: M1 (RR), M2 (FR)
        set_motor(1, right >= 0 ? 1 : -1, (uint8_t)abs(right));
        set_motor(2, right >= 0 ? 1 : -1, (uint8_t)abs(right));
        // Left side: M3 (FL), M4 (RL) — wiring polarity inverted
        set_motor(3, left  >= 0 ? -1 : 1, (uint8_t)abs(left));
        set_motor(4, left  >= 0 ? -1 : 1, (uint8_t)abs(left));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ═══════════════════════════════════════════════════════════
 * TASK: MPU6050 READ
 * Priority 2, Core 0
 * Reads accel + gyro at ~100 Hz.
 * Protects shared imu_data struct with imuMutex.
 * ═══════════════════════════════════════════════════════════ */
void imu_task(void *arg) {
    while (1) {
        ImuData_t sample;
        if (!mpu_read_all(sample)) {
            faultStatus |= (1 << FAULT_MPU6050_ERROR);
        } else {
            if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(10))) {
                imu_data = sample;
                faultStatus &= ~(1 << FAULT_MPU6050_ERROR);
                xSemaphoreGive(imuMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));   // ~100 Hz
    }
}

/* ═══════════════════════════════════════════════════════════
 * TASK: IMU CAN TX
 * Priority 2, Core 0
 * Sends two CAN frames every 20 ms (50 Hz):
 *
 *   0x200 — IMU frame A (8 bytes):
 *     [0-1] ax × 100  int16  big-endian   (m/s²)
 *     [2-3] ay × 100  int16  big-endian
 *     [4-5] az × 100  int16  big-endian
 *     [6-7] gx × 100  int16  big-endian   (°/s)
 *
 *   0x202 — IMU frame B (8 bytes):
 *     [0-1] gy × 100  int16  big-endian   (°/s)
 *     [2-3] gz × 100  int16  big-endian
 *     [4]   faultStatus
 *     [5-7] 0x00 (reserved)
 *
 * Multiply by 100 before packing to preserve 2 decimal places
 * inside a 16-bit integer without floating-point on the receiver.
 * ═══════════════════════════════════════════════════════════ */
void imu_tx_task(void *arg) {
    while (1) {
        if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(5))) {
            ImuData_t d = imu_data;
            xSemaphoreGive(imuMutex);

            // Pack ×100 into signed 16-bit big-endian
            int16_t ax = (int16_t)(d.ax * 100.0f);
            int16_t ay = (int16_t)(d.ay * 100.0f);
            int16_t az = (int16_t)(d.az * 100.0f);
            int16_t gx = (int16_t)(d.gx * 100.0f);
            int16_t gy = (int16_t)(d.gy * 100.0f);
            int16_t gz = (int16_t)(d.gz * 100.0f);

            // Frame A: ax, ay, az, gx
            uint8_t frameA[8] = {
                (uint8_t)(ax >> 8), (uint8_t)(ax & 0xFF),
                (uint8_t)(ay >> 8), (uint8_t)(ay & 0xFF),
                (uint8_t)(az >> 8), (uint8_t)(az & 0xFF),
                (uint8_t)(gx >> 8), (uint8_t)(gx & 0xFF),
            };
            can_send(CAN_ID_IMU, frameA);

            // Frame B: gy, gz, fault
            uint8_t frameB[8] = {
                (uint8_t)(gy >> 8), (uint8_t)(gy & 0xFF),
                (uint8_t)(gz >> 8), (uint8_t)(gz & 0xFF),
                faultStatus,
                0x00, 0x00, 0x00
            };
            can_send(CAN_ID_IMU2, frameB);
        }

        vTaskDelay(pdMS_TO_TICKS(20));   // 50 Hz
    }
}

/* ═══════════════════════════════════════════════════════════
 * TASK: ENCODER CAN TX
 * Priority 2, Core 0
 * Sends M1 and M4 PCNT counter values at 20 Hz.
 *
 *   0x201 — Encoder frame (8 bytes):
 *     [0-1] M1 ticks  int16  big-endian  (rear right)
 *     [2-3] M4 ticks  int16  big-endian  (rear left)
 *     [4-7] 0x00 (reserved)
 * ═══════════════════════════════════════════════════════════ */
void encoder_tx_task(void *arg) {
    while (1) {
        int16_t cnt_m1 = 0, cnt_m4 = 0;
        pcnt_get_counter_value(ENC_M1_UNIT, &cnt_m1);
        pcnt_get_counter_value(ENC_M4_UNIT, &cnt_m4);

        uint8_t frame[8] = {
            (uint8_t)(cnt_m1 >> 8), (uint8_t)(cnt_m1 & 0xFF),
            (uint8_t)(cnt_m4 >> 8), (uint8_t)(cnt_m4 & 0xFF),
            0x00, 0x00, 0x00, 0x00
        };
        can_send(CAN_ID_ENCODER, frame);

        vTaskDelay(pdMS_TO_TICKS(50));   // 20 Hz
    }
}

/* ═══════════════════════════════════════════════════════════
 * TASK: FAULT CAN TX
 * Priority 1, Core 0
 * Broadcasts fault byte at 2 Hz so Pi can monitor ECU health.
 *
 *   0x300 — Fault frame:
 *     [0] faultStatus  bit0=CAN_TIMEOUT  bit1=MPU_ERROR  bit2=MOTOR_FAULT
 * ═══════════════════════════════════════════════════════════ */
void fault_tx_task(void *arg) {
    while (1) {
        uint8_t frame[8] = { faultStatus, 0, 0, 0, 0, 0, 0, 0 };
        can_send(CAN_ID_FAULT, frame);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ═══════════════════════════════════════════════════════════
 * SETUP
 * ═══════════════════════════════════════════════════════════ */
void setup() {
    Serial.begin(115200);

    // I²C for MPU6050  (SDA=21, SCL=22 by default on ESP32)
    Wire.begin();
    Wire.setClock(400000);   // 400 kHz Fast Mode

    motor_init();
    can_init();

    // Initialise MPU6050 — set fault bit if not found
    if (!mpu_init()) {
        faultStatus |= (1 << FAULT_MPU6050_ERROR);
        Serial.println("[WARN] MPU6050 not found — check wiring / I2C address");
    } else {
        Serial.println("[OK]  MPU6050 initialised");
    }

    // Encoders
    encoder_init(ENC_M1_UNIT, M1_ENA, M1_ENB);
    encoder_init(ENC_M4_UNIT, M4_ENA, M4_ENB);

    // Mutexes
    ctrlMutex = xSemaphoreCreateMutex();
    imuMutex  = xSemaphoreCreateMutex();

    // ─── FreeRTOS Tasks ───────────────────────────────────
    //  Task             Name    Stack  Arg  Prio  Handle  Core
    xTaskCreatePinnedToCore(can_rx_task,      "rx",   2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(control_task,     "ctrl", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(imu_task,         "imu",  2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(imu_tx_task,      "itx",  2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(encoder_tx_task,  "enc",  2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(fault_tx_task,    "flt",  1024, NULL, 1, NULL, 0);

    Serial.println("Motor ECU READY");
}

/* Arduino loop() must not run — FreeRTOS owns the scheduler */
void loop() { vTaskDelete(NULL); }

/************************************************************
 * ESP32 Motor Driver ECU
 * Simple differential drive motor control via CAN
 ************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Adafruit_ADXL345_U.h>
#include "driver/twai.h"
#include "driver/pcnt.h"

/* ================= MOTOR PINS ================= */
#define MOTOR1_RPWM 26  // rear right
#define MOTOR1_LPWM 27
#define MOTOR2_RPWM 32  // front right
#define MOTOR2_LPWM 33
#define MOTOR3_RPWM 25  // front left
#define MOTOR3_LPWM 14
#define MOTOR4_RPWM 12  // rear left
#define MOTOR4_LPWM 13

/* ================= PWM ================= */
#define PWM_FREQ  5000
#define PWM_RES   8
#define MAX_PWM   255

/* ================= ENCODER PINS ================= */
#define M1_ENA 23
#define M1_ENB 15
#define M4_ENA 19
#define M4_ENB 18

#define PCNT_M1 PCNT_UNIT_0
#define PCNT_M4 PCNT_UNIT_1

/* ================= CAN ================= */
#define CAN_TX GPIO_NUM_5
#define CAN_RX GPIO_NUM_4

#define CAN_ID_CMD      0x101
#define CAN_ID_FEEDBACK 0x200
#define CAN_ID_ENCODER  0x201
#define CAN_ID_FAULT    0x300

/* ================= CONTROL DATA ================= */
typedef struct {
  int16_t x;       // throttle  -100 to 100
  int16_t y;       // steering  -100 to 100
  uint8_t speed;   // 0 to 100
  uint8_t enable;  // 1 = run, 0 = stop
  uint8_t mode;    // unused — scaling done on Pi
} ControlData_t;

/* ================= GLOBALS ================= */
ControlData_t ctrl = {0};
SemaphoreHandle_t ctrlMutex;
uint32_t last_rx_time = 0;

Adafruit_ADXL345_Unified adxl(12345);
SemaphoreHandle_t accelMutex;
float ax_ = 0, ay_ = 0, az_ = 0;
uint8_t faultStatus = 0;

/* ================= ENCODER ================= */
volatile int32_t enc_m1 = 0;
volatile int32_t enc_m4 = 0;

static void IRAM_ATTR pcnt_isr(void *arg) {
  uint32_t status;
  pcnt_get_event_status(PCNT_M1, &status);
  if (status & PCNT_EVT_H_LIM) enc_m1 += 32767;
  if (status & PCNT_EVT_L_LIM) enc_m1 -= 32767;
  pcnt_get_event_status(PCNT_M4, &status);
  if (status & PCNT_EVT_H_LIM) enc_m4 += 32767;
  if (status & PCNT_EVT_L_LIM) enc_m4 -= 32767;
}

void pcnt_init(pcnt_unit_t unit, int pulse, int ctrl_pin) {
  pcnt_config_t cfg = {
    .pulse_gpio_num = pulse,
    .ctrl_gpio_num  = ctrl_pin,
    .lctrl_mode     = PCNT_MODE_REVERSE,
    .hctrl_mode     = PCNT_MODE_KEEP,
    .pos_mode       = PCNT_COUNT_INC,
    .neg_mode       = PCNT_COUNT_DIS,
    .counter_h_lim  =  32767,
    .counter_l_lim  = -32767,
    .unit           = unit,
    .channel        = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&cfg);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit, PCNT_EVT_L_LIM);
  pcnt_counter_resume(unit);
}

int32_t get_enc(pcnt_unit_t unit, volatile int32_t &acc) {
  int16_t v = 0;
  pcnt_get_counter_value(unit, &v);
  return acc + v;
}

/* ================= MOTOR ================= */
void motor_init() {
  ledcAttach(MOTOR1_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR1_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR2_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR2_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR3_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR3_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR4_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR4_LPWM, PWM_FREQ, PWM_RES);
}

// pwm: -255 to 255, positive = forward
void set_motor(uint8_t rpwm, uint8_t lpwm, int16_t pwm) {
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (pwm > 0) {
    ledcWrite(rpwm, pwm);
    ledcWrite(lpwm, 0);
  } else if (pwm < 0) {
    ledcWrite(rpwm, 0);
    ledcWrite(lpwm, -pwm);
  } else {
    ledcWrite(rpwm, 0);
    ledcWrite(lpwm, 0);
  }
}

void stop_all() {
  set_motor(MOTOR1_RPWM, MOTOR1_LPWM, 0);
  set_motor(MOTOR2_RPWM, MOTOR2_LPWM, 0);
  set_motor(MOTOR3_RPWM, MOTOR3_LPWM, 0);
  set_motor(MOTOR4_RPWM, MOTOR4_LPWM, 0);
}

/* ================= CAN ================= */
void can_send(uint32_t id, uint8_t *data) {
  twai_message_t tx = {.identifier = id, .data_length_code = 8};
  memcpy(tx.data, data, 8);
  twai_transmit(&tx, pdMS_TO_TICKS(10));
}

void can_init() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g, &t, &f);
  twai_start();
}

/* ================= CAN RX TASK ================= */
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
      }
    } else {
      if (millis() - last_rx_time > 500) {
        if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) {
          ctrl.enable = 0;
          xSemaphoreGive(ctrlMutex);
        }
      }
    }
  }
}

/* ================= CONTROL TASK ================= */
// Simple differential drive:
//   left  = x - y
//   right = x + y
// x = throttle (-100 to 100), y = steering (-100 to 100)
// Both already scaled on Pi side — just map to PWM directly
void control_task(void *arg) {
  while (1) {
    ControlData_t c;
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) {
      c = ctrl;
      xSemaphoreGive(ctrlMutex);
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (!c.enable) {
      stop_all();
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Map -100..100 to -255..255
    int16_t left  = (int16_t)(((float)(c.x - c.y) / 100.0f) * MAX_PWM);
    int16_t right = (int16_t)(((float)(c.x + c.y) / 100.0f) * MAX_PWM);

    left  = constrain(left,  -MAX_PWM, MAX_PWM);
    right = constrain(right, -MAX_PWM, MAX_PWM);

    // Right side motors (M1 rear right, M2 front right)
    set_motor(MOTOR1_RPWM, MOTOR1_LPWM,  right);
    set_motor(MOTOR2_RPWM, MOTOR2_LPWM,  right);

    // Left side motors (M3 front left, M4 rear left) — inverted mounting
    set_motor(MOTOR3_RPWM, MOTOR3_LPWM, -left);
    set_motor(MOTOR4_RPWM, MOTOR4_LPWM, -left);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ================= ACCEL TASK ================= */
void accel_task(void *arg) {
  sensors_event_t e;
  while (1) {
    if (adxl.getEvent(&e)) {
      if (xSemaphoreTake(accelMutex, pdMS_TO_TICKS(10))) {
        ax_ = e.acceleration.x;
        ay_ = e.acceleration.y;
        az_ = e.acceleration.z;
        xSemaphoreGive(accelMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* ================= FEEDBACK TX TASK ================= */
void can_tx_task(void *arg) {
  uint32_t lastTx = 0;
  while (1) {
    if (millis() - lastTx < 200) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    uint8_t d[8] = {0};
    if (xSemaphoreTake(accelMutex, pdMS_TO_TICKS(5))) {
      int16_t ax = ax_ * 100, ay = ay_ * 100, az = az_ * 100;
      d[0]=ax>>8; d[1]=ax; d[2]=ay>>8; d[3]=ay; d[4]=az>>8; d[5]=az;
      xSemaphoreGive(accelMutex);
    }
    d[6] = faultStatus;
    can_send(CAN_ID_FEEDBACK, d);
    lastTx = millis();
  }
}

/* ================= ENCODER TX TASK ================= */
void encoder_tx_task(void *arg) {
  while (1) {
    int32_t m1 = get_enc(PCNT_M1, enc_m1);
    int32_t m4 = get_enc(PCNT_M4, enc_m4);

    uint8_t d[8];
    d[0]=(m1>>24)&0xFF; d[1]=(m1>>16)&0xFF; d[2]=(m1>>8)&0xFF; d[3]=m1&0xFF;
    d[4]=(m4>>24)&0xFF; d[5]=(m4>>16)&0xFF; d[6]=(m4>>8)&0xFF; d[7]=m4&0xFF;

    can_send(CAN_ID_ENCODER, d);
    vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz
  }
}

/* ================= FAULT TX TASK ================= */
void fault_tx_task(void *arg) {
  uint8_t d[8] = {0};
  while (1) {
    d[0] = faultStatus;
    can_send(CAN_ID_FAULT, d);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  motor_init();
  can_init();

  if (!adxl.begin()) faultStatus |= 0x01;

  pcnt_init(PCNT_M1, M1_ENA, M1_ENB);
  pcnt_init(PCNT_M4, M4_ENA, M4_ENB);
  pcnt_isr_register(pcnt_isr, NULL, 0, NULL);
  pcnt_intr_enable(PCNT_M1);
  pcnt_intr_enable(PCNT_M4);

  ctrlMutex  = xSemaphoreCreateMutex();
  accelMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(can_rx_task,     "rx",  2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(control_task,    "ctrl",2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(accel_task,      "acl", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(can_tx_task,     "tx",  2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(fault_tx_task,   "flt", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(encoder_tx_task, "enc", 2048, NULL, 2, NULL, 0);

  Serial.println("[BOOT] Ready");
}

void loop() { vTaskDelete(NULL); }

/************************************************************
 * ESP32 Motor Driver ECU + ADXL345 (Lightweight)
 * Eco/Normal/Sport Mode + Fault Detection
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
#define MOTOR1_RPWM 26
#define MOTOR1_LPWM 27
#define MOTOR2_RPWM 32
#define MOTOR2_LPWM 33
#define MOTOR3_RPWM 25
#define MOTOR3_LPWM 14
#define MOTOR4_RPWM 12
#define MOTOR4_LPWM 13

/* ================= PWM ================= */
#define PWM_FREQ 5000
#define PWM_RES  8
#define MAX_PWM  255
#define MIN_PWM  30

/* ================= ENCODER PINS ================= */
#define M1_ENA 23   // rear right — PCNT pulse
#define M1_ENB 15   // rear right — PCNT control (direction)
#define M4_ENA 19   // rear left  — PCNT pulse
#define M4_ENB 18   // rear left  — PCNT control (direction)

#define PCNT_M1 PCNT_UNIT_0
#define PCNT_M4 PCNT_UNIT_1

/* ================= CAN ================= */
#define CAN_TX GPIO_NUM_5
#define CAN_RX GPIO_NUM_4

#define CAN_ID_RECEIVER_DATA 0x101
#define CAN_ID_FEEDBACK      0x200
#define CAN_ID_ENCODER       0x201   // encoder tick counts
#define CAN_ID_FAULT         0x300

/* ================= FAULT FLAGS ================= */
#define FAULT_CAN_RX_TIMEOUT   0
#define FAULT_ADXL_ERROR       1
#define FAULT_MOTOR_FAULT      2

/* ================= STRUCTS ================= */
typedef struct {
  int16_t x;
  int16_t y;
  uint8_t speed;
  uint8_t enable;
  uint8_t mode;
} ControlData_t;

typedef struct {
  float ax, ay, az;
} AccelData_t;

/* ================= GLOBALS ================= */
ControlData_t control = {0};
AccelData_t accel = {0};
SemaphoreHandle_t accelMutex;
SemaphoreHandle_t controlMutex;   // ← protects control struct
uint32_t last_rx_time = 0;
Adafruit_ADXL345_Unified adxl(12345);
uint8_t faultStatus = 0;

/* ================= ENCODER GLOBALS ================= */
volatile int32_t enc_m1 = 0;   // M1 accumulated count — rear right
volatile int32_t enc_m4 = 0;   // M4 accumulated count — rear left

// PCNT overflows at ±32767 — accumulate into int32
static void IRAM_ATTR pcnt_isr(void *arg) {
  // Called per-unit — arg is NULL, check both units
  uint32_t status;
  pcnt_get_event_status(PCNT_M1, &status);
  if (status & PCNT_EVT_H_LIM) enc_m1 += 32767;
  if (status & PCNT_EVT_L_LIM) enc_m1 -= 32767;

  pcnt_get_event_status(PCNT_M4, &status);
  if (status & PCNT_EVT_H_LIM) enc_m4 += 32767;
  if (status & PCNT_EVT_L_LIM) enc_m4 -= 32767;
}

void pcnt_init_encoder(pcnt_unit_t unit, int pulse_pin, int ctrl_pin) {
  pcnt_config_t cfg = {
    .pulse_gpio_num  = pulse_pin,
    .ctrl_gpio_num   = ctrl_pin,
    .lctrl_mode      = PCNT_MODE_REVERSE,
    .hctrl_mode      = PCNT_MODE_KEEP,
    .pos_mode        = PCNT_COUNT_INC,
    .neg_mode        = PCNT_COUNT_DIS,
    .counter_h_lim   =  32767,
    .counter_l_lim   = -32767,
    .unit            = unit,
    .channel         = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&cfg);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit, PCNT_EVT_L_LIM);
  // Note: pcnt_intr_enable called after pcnt_isr_register in setup
  pcnt_counter_resume(unit);
}

int32_t get_encoder(pcnt_unit_t unit, volatile int32_t &accum) {
  int16_t count = 0;
  pcnt_get_counter_value(unit, &count);
  return accum + count;
}

/* ================= MOTOR FUNCTIONS ================= */
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

void set_motor(uint8_t motor, int8_t dir, uint8_t pwm) {
  uint8_t r, l;
  if(motor==1){r=MOTOR1_RPWM;l=MOTOR1_LPWM;}
  else if(motor==2){r=MOTOR2_RPWM;l=MOTOR2_LPWM;}
  else if(motor==3){r=MOTOR3_RPWM;l=MOTOR3_LPWM;}
  else{r=MOTOR4_RPWM;l=MOTOR4_LPWM;}

  if(dir>0){ledcWrite(r,pwm);ledcWrite(l,0);}
  else if(dir<0){ledcWrite(r,0);ledcWrite(l,pwm);}
  else{ledcWrite(r,0);ledcWrite(l,0);}
}

/* ================= CAN FUNCTIONS ================= */
void can_send(uint32_t id, uint8_t *data) {
  twai_message_t tx = {.identifier = id, .data_length_code = 8};
  memcpy(tx.data, data, 8);
  twai_transmit(&tx, pdMS_TO_TICKS(10));  // wait up to 10ms for TX buffer space
}

void can_init() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g, &t, &f);
  twai_start();
}

/* ================= CAN RX TASK ================= */
void can_rx_task(void *arg) {
  twai_message_t rx;
  while(1) {
    // twai_status_info_t status;
    // twai_get_status_info(&status);
    // Serial.printf("[CAN] state=%d rx_err=%d tx_err=%d\n",
    //   status.state, status.rx_error_counter, status.tx_error_counter);

    if(twai_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK) {
      if(rx.identifier == CAN_ID_RECEIVER_DATA && rx.data_length_code == 8) {
        if(xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          control.y      = (int16_t)((rx.data[0]<<8)|rx.data[1]);
          control.x      = (int16_t)((rx.data[2]<<8)|rx.data[3]);
          control.speed  = rx.data[4];
          control.enable = rx.data[5];
          control.mode   = rx.data[6];
          xSemaphoreGive(controlMutex);
        }
        last_rx_time = millis();
        faultStatus &= ~(1 << FAULT_CAN_RX_TIMEOUT);
        // Serial.printf("[CAN RX] x=%d y=%d speed=%d enable=%d mode=%d\n",
        //   control.x, control.y, control.speed, control.enable, control.mode);
      }
      // else {
      //   Serial.printf("[CAN RX] Unknown ID=0x%X dlc=%d\n",
      //     rx.identifier, rx.data_length_code);
      // }
    } else {
      if(millis() - last_rx_time > 500) {
        faultStatus |= (1 << FAULT_CAN_RX_TIMEOUT);
        // Serial.println("[CAN RX] TIMEOUT — no frame for 500ms");
      }
    }
  }
}

/* ================= CONTROL TASK ================= */
void control_task(void *arg) {
  while(1) {
    if(millis() - last_rx_time > 500) {
      if(xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        control.enable = 0;
        xSemaphoreGive(controlMutex);
      }
      faultStatus |= (1 << FAULT_CAN_RX_TIMEOUT);
    }

    // Take a local snapshot of control — hold mutex as briefly as possible
    ControlData_t local;
    if(xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      local = control;
      xSemaphoreGive(controlMutex);
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if(!local.enable) {
      for(int i=1; i<=4; i++) set_motor(i, 0, 0);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    float mode_scale = (local.mode==0) ? 0.3f : (local.mode==1) ? 0.6f : 1.0f;
    float throttle = (local.x / 100.0f) * local.speed * mode_scale;
    float turn = -(local.y / 100.0f) * local.speed * mode_scale;

    int16_t left  = (int16_t)constrain(throttle + turn, -100, 100);
    int16_t right = (int16_t)constrain(throttle - turn, -100, 100);

    left  = (int16_t)((left  / 100.0f) * MAX_PWM);
    right = (int16_t)((right / 100.0f) * MAX_PWM);

    if(left  > 0) left  = (left  < MIN_PWM) ? MIN_PWM : left;
    if(left  < 0) left  = (left  > -MIN_PWM) ? -MIN_PWM : left;
    if(right > 0) right = (right < MIN_PWM) ? MIN_PWM : right;
    if(right < 0) right = (right > -MIN_PWM) ? -MIN_PWM : right;

    if(abs(left)  < 10) left  = 0;
    if(abs(right) < 10) right = 0;

    set_motor(1, left>=0?1:-1,  abs(left));
    set_motor(4, right>=0?1:-1, abs(right));
    set_motor(2, left>=0?-1:1,  abs(left));
    set_motor(3, right>=0?-1:1, abs(right));

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ================= ACCEL TASK ================= */
void accel_task(void *arg) {
  sensors_event_t e;
  while(1) {
    if(!adxl.getEvent(&e)) {
      faultStatus |= (1 << FAULT_ADXL_ERROR);
    } else {
      if(xSemaphoreTake(accelMutex, pdMS_TO_TICKS(10))) {
        accel.ax = e.acceleration.x;
        accel.ay = e.acceleration.y;
        accel.az = e.acceleration.z;
        faultStatus &= ~(1 << FAULT_ADXL_ERROR);
        xSemaphoreGive(accelMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* ================= CAN TX TASK ================= */
void can_tx_task(void *arg) {
  uint32_t lastTx = 0;
  while(1) {
    if(millis() - lastTx < 200) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if(xSemaphoreTake(accelMutex, pdMS_TO_TICKS(5))) {
      uint8_t tx_data[8];
      int16_t ax = accel.ax * 100;
      int16_t ay = accel.ay * 100;
      int16_t az = accel.az * 100;

      tx_data[0] = ax>>8; tx_data[1] = ax;
      tx_data[2] = ay>>8; tx_data[3] = ay;
      tx_data[4] = az>>8; tx_data[5] = az;

      // Read speed safely
      uint8_t spd = 0;
      if(xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        spd = control.speed;
        xSemaphoreGive(controlMutex);
      }
      tx_data[6] = spd;
      tx_data[7] = faultStatus;

      can_send(CAN_ID_FEEDBACK, tx_data);
      xSemaphoreGive(accelMutex);
      lastTx = millis();
    }
  }
}

/* ================= FAULT TX TASK ================= */
void fault_tx_task(void *arg) {
  uint8_t tx_data[8] = {0};
  while(1) {
    tx_data[0] = faultStatus;
    can_send(CAN_ID_FAULT, tx_data);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/* ================= ENCODER TX TASK ================= */
// Sends M1 and M4 tick counts over CAN ID 0x201 at 20Hz
// Frame layout (8 bytes):
//   byte[0..3] = enc_m1 (rear right, int32_t big-endian)
//   byte[4..7] = enc_m4 (rear left,  int32_t big-endian)
void encoder_tx_task(void *arg) {
  while(1) {
    int32_t m1 = get_encoder(PCNT_M1, enc_m1);
    int32_t m4 = get_encoder(PCNT_M4, enc_m4);

    Serial.printf("[ENC] M1=%d  M4=%d\n", m1, m4);

    uint8_t tx_data[8];
    tx_data[0] = (m1 >> 24) & 0xFF;
    tx_data[1] = (m1 >> 16) & 0xFF;
    tx_data[2] = (m1 >>  8) & 0xFF;
    tx_data[3] =  m1        & 0xFF;
    tx_data[4] = (m4 >> 24) & 0xFF;
    tx_data[5] = (m4 >> 16) & 0xFF;
    tx_data[6] = (m4 >>  8) & 0xFF;
    tx_data[7] =  m4        & 0xFF;

    can_send(CAN_ID_ENCODER, tx_data);
    vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Serial.println("[BOOT] Starting...");

  Wire.begin();
  Serial.println("[BOOT] Wire OK");

  motor_init();
  Serial.println("[BOOT] Motors OK");

  can_init();
  Serial.println("[BOOT] CAN OK");

  if(!adxl.begin()) {
    faultStatus |= (1 << FAULT_ADXL_ERROR);
    Serial.println("[BOOT] ADXL345 FAILED");
  } else {
    Serial.println("[BOOT] ADXL345 OK");
  }

  Serial.println("[BOOT] Init PCNT...");
  pcnt_init_encoder(PCNT_M1, M1_ENA, M1_ENB);
  Serial.println("[BOOT] PCNT M1 encoder init OK");
  pcnt_init_encoder(PCNT_M4, M4_ENA, M4_ENB);
  Serial.println("[BOOT] PCNT M4 encoder init OK");
  // Register single ISR for both units after both are configured
  pcnt_isr_register(pcnt_isr, NULL, 0, NULL);
  pcnt_intr_enable(PCNT_M1);
  pcnt_intr_enable(PCNT_M4);
  Serial.println("[BOOT] PCNT ISR registered");

  accelMutex   = xSemaphoreCreateMutex();
  controlMutex = xSemaphoreCreateMutex();
  Serial.println("[BOOT] Mutexes created");

  xTaskCreatePinnedToCore(can_rx_task,    "rx",   2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(control_task,   "ctrl", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(accel_task,     "acl",  2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(can_tx_task,    "tx",   2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(fault_tx_task,  "flt",  1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(encoder_tx_task,"enc",  2048, NULL, 2, NULL, 0);
  Serial.println("[BOOT] All tasks started");
}

void loop() {
  vTaskDelete(NULL);
}

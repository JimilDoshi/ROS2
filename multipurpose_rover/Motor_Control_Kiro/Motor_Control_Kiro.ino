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
#define MOTOR1_RPWM 27
#define MOTOR1_LPWM 26
#define MOTOR2_RPWM 33
#define MOTOR2_LPWM 32
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
#define M1_ENA 23
#define M1_ENB 15
#define M4_ENA 19
#define M4_ENB 18

#define ENC_M1_UNIT PCNT_UNIT_0
#define ENC_M4_UNIT PCNT_UNIT_1

/* ================= CAN ================= */
#define CAN_TX GPIO_NUM_5
#define CAN_RX GPIO_NUM_4

#define CAN_ID_RECEIVER_DATA 0x101
#define CAN_ID_FEEDBACK      0x200
#define CAN_ID_ENCODER       0x201
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
uint32_t last_rx_time = 0;
Adafruit_ADXL345_Unified adxl(12345);
uint8_t faultStatus = 0;

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
  twai_transmit(&tx, 0);
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
    if(twai_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK) {
      if(rx.identifier == CAN_ID_RECEIVER_DATA && rx.data_length_code == 8) {
        control.y = (int16_t)((rx.data[0]<<8)|rx.data[1]);
        control.x = (int16_t)((rx.data[2]<<8)|rx.data[3]);
        control.speed = rx.data[4];
        control.enable = rx.data[5];
        control.mode = rx.data[6];
        last_rx_time = millis();
        faultStatus &= ~(1 << FAULT_CAN_RX_TIMEOUT);
      }
    } else {
      if(millis() - last_rx_time > 500) {
        faultStatus |= (1 << FAULT_CAN_RX_TIMEOUT);
      }
    }
  }
}

/* ================= CONTROL TASK ================= */
void control_task(void *arg) {
  while(1) {
    if(millis() - last_rx_time > 500) {
      control.enable = 0;
      faultStatus |= (1 << FAULT_CAN_RX_TIMEOUT);
    }

    if(!control.enable) {
      for(int i=1; i<=4; i++) set_motor(i, 0, 0);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    float mode_scale = (control.mode==0) ? 0.3f : (control.mode==1) ? 0.6f : 1.0f;
    float throttle = (control.x / 100.0f) * control.speed * mode_scale;
    float turn = -(control.y / 100.0f) * control.speed * mode_scale;

    int16_t left = (int16_t)constrain(throttle + turn, -100, 100);
    int16_t right = (int16_t)constrain(throttle - turn, -100, 100);

    left = (int16_t)((left / 100.0f) * MAX_PWM);
    right = (int16_t)((right / 100.0f) * MAX_PWM);

    if(left > 0) left = (left < MIN_PWM) ? MIN_PWM : left;
    if(left < 0) left = (left > -MIN_PWM) ? -MIN_PWM : left;
    if(right > 0) right = (right < MIN_PWM) ? MIN_PWM : right;
    if(right < 0) right = (right > -MIN_PWM) ? -MIN_PWM : right;

    if(abs(left) < 10) left = 0;
    if(abs(right) < 10) right = 0;

    // M1=RR, M2=FR (right side), M3=FL, M4=RL (left side)
    set_motor(1, left>=0?1:-1,  abs(left));   // RR - right side
    set_motor(2, right>=0?1:-1, abs(right));  // FR - right side
    set_motor(3, left>=0?1:-1,  abs(left));   // FL - left side
    set_motor(4, right>=0?1:-1, abs(right));  // RL - left side

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
    vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz — matches encoder rate
  }
}

/* ================= CAN TX TASK ================= */
void can_tx_task(void *arg) {
  while(1) {
    if(xSemaphoreTake(accelMutex, pdMS_TO_TICKS(5))) {
      uint8_t tx_data[8];
      int16_t ax = accel.ax * 100;
      int16_t ay = accel.ay * 100;
      int16_t az = accel.az * 100;

      tx_data[0] = ax>>8; tx_data[1] = ax;
      tx_data[2] = ay>>8; tx_data[3] = ay;
      tx_data[4] = az>>8; tx_data[5] = az;
      tx_data[6] = control.speed;
      tx_data[7] = faultStatus;

      can_send(CAN_ID_FEEDBACK, tx_data);
      xSemaphoreGive(accelMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz
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

/* ================= ENCODER FUNCTIONS ================= */
void encoder_init(pcnt_unit_t unit, int pin_a, int pin_b) {
  pcnt_config_t cfg = {
    .pulse_gpio_num = pin_a,
    .ctrl_gpio_num  = pin_b,
    .lctrl_mode     = PCNT_MODE_REVERSE,
    .hctrl_mode     = PCNT_MODE_KEEP,
    .pos_mode       = PCNT_COUNT_INC,
    .neg_mode       = PCNT_COUNT_DEC,
    .counter_h_lim  = 32767,
    .counter_l_lim  = -32768,
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

/* ================= ENCODER TX TASK ================= */
void encoder_tx_task(void *arg) {
  while(1) {
    int16_t cnt_m1 = 0, cnt_m4 = 0;
    pcnt_get_counter_value(ENC_M1_UNIT, &cnt_m1);
    pcnt_get_counter_value(ENC_M4_UNIT, &cnt_m4);

    uint8_t tx_data[8] = {0};
    tx_data[0] = (cnt_m1 >> 8) & 0xFF;
    tx_data[1] =  cnt_m1       & 0xFF;
    tx_data[2] = (cnt_m4 >> 8) & 0xFF;
    tx_data[3] =  cnt_m4       & 0xFF;

    can_send(CAN_ID_ENCODER, tx_data);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* ================= SETUP ================= */
void setup() {
  Wire.begin();
  motor_init();
  can_init();

  if(!adxl.begin()) {
    faultStatus |= (1 << FAULT_ADXL_ERROR);
  }

  encoder_init(ENC_M1_UNIT, M1_ENA, M1_ENB);
  encoder_init(ENC_M4_UNIT, M4_ENA, M4_ENB);

  accelMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(can_rx_task,     "rx",  2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(control_task,    "ctrl",2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(accel_task,      "acl", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(can_tx_task,     "tx",  2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(fault_tx_task,   "flt", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(encoder_tx_task, "enc", 2048, NULL, 2, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);
}
/************************************************************
 * ESP32 Motor Driver ECU — FINAL STABLE VERSION (v3.x core)
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
#define MOTOR1_RPWM 26   // rear right
#define MOTOR1_LPWM 27
#define MOTOR2_RPWM 32   // front right
#define MOTOR2_LPWM 33
#define MOTOR3_RPWM 25   // front left
#define MOTOR3_LPWM 14
#define MOTOR4_RPWM 12   // rear left
#define MOTOR4_LPWM 13

#define PWM_FREQ 5000
#define PWM_RES  8
#define MAX_PWM  255

/* ================= ENCODERS ================= */
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
#define CAN_ID_ENCODER  0x201

/* ================= CONTROL STRUCT ================= */
typedef struct {
  int16_t x;
  int16_t y;
  uint8_t speed;
  uint8_t enable;
  uint8_t mode;
} ControlData_t;

ControlData_t ctrl = {0};
SemaphoreHandle_t ctrlMutex;
uint32_t last_rx_time = 0;

/* ================= ACCEL ================= */
Adafruit_ADXL345_Unified adxl(12345);

/* ================= ENCODERS ================= */
volatile int32_t enc_m1 = 0;
volatile int32_t enc_m4 = 0;

static void IRAM_ATTR pcnt_isr(void *arg) {
  uint32_t s0, s1;
  pcnt_get_event_status(PCNT_M1, &s0);
  pcnt_get_event_status(PCNT_M4, &s1);

  if (s0 & PCNT_EVT_H_LIM) enc_m1 += 32767;
  if (s0 & PCNT_EVT_L_LIM) enc_m1 -= 32767;
  if (s1 & PCNT_EVT_H_LIM) enc_m4 += 32767;
  if (s1 & PCNT_EVT_L_LIM) enc_m4 -= 32767;

  pcnt_counter_clear(PCNT_M1);
  pcnt_counter_clear(PCNT_M4);
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
  pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit, PCNT_EVT_L_LIM);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

int32_t get_enc(pcnt_unit_t unit, volatile int32_t &acc) {
  int16_t v;
  pcnt_get_counter_value(unit, &v);
  return acc + v;
}

/* ================= PWM MOTOR (ESP32 v3 API) ================= */
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

void set_motor(uint8_t rpwm, uint8_t lpwm, int16_t pwm) {
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (pwm > 0) { ledcWrite(rpwm, pwm); ledcWrite(lpwm, 0); }
  else if (pwm < 0) { ledcWrite(rpwm, 0); ledcWrite(lpwm, -pwm); }
  else { ledcWrite(rpwm, 0); ledcWrite(lpwm, 0); }
}

void stop_all() {
  ledcWrite(MOTOR1_RPWM,0); ledcWrite(MOTOR1_LPWM,0);
  ledcWrite(MOTOR2_RPWM,0); ledcWrite(MOTOR2_LPWM,0);
  ledcWrite(MOTOR3_RPWM,0); ledcWrite(MOTOR3_LPWM,0);
  ledcWrite(MOTOR4_RPWM,0); ledcWrite(MOTOR4_LPWM,0);
}

/* ================= MOTOR RAMP ================= */
int16_t ramp(int16_t target, int16_t current, int16_t step = 6) {
  if (target > current) {
    int16_t next = current + step;
    return (next > target) ? target : next;
  }
  if (target < current) {
    int16_t next = current - step;
    return (next < target) ? target : next;
  }
  return current;
}

/* ================= CAN ================= */
void can_send(uint32_t id, uint8_t *data) {
  twai_message_t tx = {.identifier=id,.data_length_code=8};
  memcpy(tx.data,data,8);
  twai_transmit(&tx,pdMS_TO_TICKS(10));
}

void can_init() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX,CAN_RX,TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g,&t,&f);
  twai_start();
}

/* ================= CAN RX ================= */
void can_rx_task(void *arg) {
  twai_message_t rx;
  while (1) {
    if (twai_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK) {
      if (rx.identifier == CAN_ID_CMD && rx.data_length_code == 8) {
        if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) {
          ctrl.y=(rx.data[0]<<8)|rx.data[1];
          ctrl.x=(rx.data[2]<<8)|rx.data[3];
          ctrl.enable=rx.data[5];
          xSemaphoreGive(ctrlMutex);
        }
        last_rx_time = millis();
      }
    }

    if (millis() - last_rx_time > 500) {
      if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) {
        ctrl.enable = 0;
        ctrl.x = 0;
        ctrl.y = 0;
        xSemaphoreGive(ctrlMutex);
      }
    }
  }
}

/* ================= CONTROL TASK ================= */
void control_task(void *arg) {
  int16_t left_pwm=0, right_pwm=0;

  while (1) {
    ControlData_t c;
    if (!xSemaphoreTake(ctrlMutex,pdMS_TO_TICKS(5))) continue;
    c=ctrl;
    xSemaphoreGive(ctrlMutex);

    if (!c.enable) { stop_all(); vTaskDelay(pdMS_TO_TICKS(20)); continue; }

    int16_t left_target  = ((c.x - c.y) * MAX_PWM) / 100;
    int16_t right_target = ((c.x + c.y) * MAX_PWM) / 100;

    left_pwm  = ramp(left_target, left_pwm);
    right_pwm = ramp(right_target, right_pwm);

    set_motor(MOTOR1_RPWM, MOTOR1_LPWM, right_pwm);
    set_motor(MOTOR2_RPWM, MOTOR2_LPWM, right_pwm);
    set_motor(MOTOR3_RPWM, MOTOR3_LPWM, -left_pwm);
    set_motor(MOTOR4_RPWM, MOTOR4_LPWM, -left_pwm);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  motor_init();
  can_init();

  pcnt_init(PCNT_M1,M1_ENA,M1_ENB);
  pcnt_init(PCNT_M4,M4_ENA,M4_ENB);
  pcnt_isr_register(pcnt_isr,NULL,0,NULL);
  pcnt_intr_enable(PCNT_M1);
  pcnt_intr_enable(PCNT_M4);

  ctrlMutex=xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(can_rx_task,"rx",2048,NULL,3,NULL,0);
  xTaskCreatePinnedToCore(control_task,"ctrl",2048,NULL,3,NULL,1);

  Serial.println("Motor ECU READY");
}

void loop(){ vTaskDelete(NULL); }

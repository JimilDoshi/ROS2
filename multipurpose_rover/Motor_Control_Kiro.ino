/************************************************************
 * ESP32 Motor Driver ECU
 * Using Arduino ESP32 Core APIs (ESP32 Arduino Core v3.x)
 * - LEDC via ledcAttach/ledcWrite
 * - CAN via ESP32Can library
 * - PCNT via ESP32Encoder library
 ************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Adafruit_ADXL345_U.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <ESP32Encoder.h>

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
#define MAX_PWM   255

/* ================= ENCODER PINS ================= */
#define M1_ENA 23
#define M1_ENB 15
#define M4_ENA 19
#define M4_ENB 18

/* ================= CAN ================= */
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

#define CAN_ID_CMD      0x101
#define CAN_ID_FEEDBACK 0x200
#define CAN_ID_ENCODER  0x201
#define CAN_ID_FAULT    0x300

/* ================= CONTROL DATA ================= */
typedef struct {
  int16_t x;
  int16_t y;
  uint8_t speed;
  uint8_t enable;
  uint8_t mode;
} ControlData_t;

/* ================= GLOBALS ================= */
ControlData_t ctrl = {0};
SemaphoreHandle_t ctrlMutex;
uint32_t last_rx_time = 0;

Adafruit_ADXL345_Unified adxl(12345);
SemaphoreHandle_t accelMutex;
float ax_ = 0, ay_ = 0, az_ = 0;
uint8_t faultStatus = 0;

ESP32Encoder enc_m1;
ESP32Encoder enc_m4;

CAN_device_t CAN_cfg;

/* ================= MOTOR ================= */
void motor_init() {
  pinMode(MOTOR1_RPWM, OUTPUT); pinMode(MOTOR1_LPWM, OUTPUT);
  pinMode(MOTOR2_RPWM, OUTPUT); pinMode(MOTOR2_LPWM, OUTPUT);
  pinMode(MOTOR3_RPWM, OUTPUT); pinMode(MOTOR3_LPWM, OUTPUT);
  pinMode(MOTOR4_RPWM, OUTPUT); pinMode(MOTOR4_LPWM, OUTPUT);
}

void set_motor(uint8_t rpwm, uint8_t lpwm, int16_t pwm) {
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (pwm > 0)      { analogWrite(rpwm, pwm);  analogWrite(lpwm, 0);    }
  else if (pwm < 0) { analogWrite(rpwm, 0);    analogWrite(lpwm, -pwm); }
  else              { analogWrite(rpwm, 0);    analogWrite(lpwm, 0);    }
}

void stop_all() {
  set_motor(MOTOR1_RPWM, MOTOR1_LPWM, 0);
  set_motor(MOTOR2_RPWM, MOTOR2_LPWM, 0);
  set_motor(MOTOR3_RPWM, MOTOR3_LPWM, 0);
  set_motor(MOTOR4_RPWM, MOTOR4_LPWM, 0);
}

/* ================= CAN ================= */
void can_init() {
  CAN_cfg.speed     = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = CAN_TX_PIN;
  CAN_cfg.rx_pin_id = CAN_RX_PIN;
  CAN_cfg.rx_queue  = xQueueCreate(10, sizeof(CAN_frame_t));
  ESP32Can.CANInit();
}

void can_send(uint32_t id, uint8_t *data) {
  CAN_frame_t frame;
  frame.FIR.B.FF  = CAN_frame_std;
  frame.MsgID     = id;
  frame.FIR.B.DLC = 8;
  memcpy(frame.data.u8, data, 8);
  ESP32Can.CANWriteFrame(&frame);
}

/* ================= CAN RX TASK ================= */
void can_rx_task(void *arg) {
  CAN_frame_t rx;
  while (1) {
    if (xQueueReceive(CAN_cfg.rx_queue, &rx, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (rx.MsgID == CAN_ID_CMD && rx.FIR.B.DLC == 8) {
        if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5))) {
          ctrl.y      = (int16_t)((rx.data.u8[0] << 8) | rx.data.u8[1]);
          ctrl.x      = (int16_t)((rx.data.u8[2] << 8) | rx.data.u8[3]);
          ctrl.speed  = rx.data.u8[4];
          ctrl.enable = rx.data.u8[5];
          ctrl.mode   = rx.data.u8[6];
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

    int16_t left  = (int16_t)(((float)(c.x - c.y) / 100.0f) * MAX_PWM);
    int16_t right = (int16_t)(((float)(c.x + c.y) / 100.0f) * MAX_PWM);

    left  = constrain(left,  -MAX_PWM, MAX_PWM);
    right = constrain(right, -MAX_PWM, MAX_PWM);

    set_motor(MOTOR1_RPWM, MOTOR1_LPWM,  right);
    set_motor(MOTOR2_RPWM, MOTOR2_LPWM,  right);
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
    int32_t m1 = (int32_t)enc_m1.getCount();
    int32_t m4 = (int32_t)enc_m4.getCount();

    uint8_t d[8];
    d[0]=(m1>>24)&0xFF; d[1]=(m1>>16)&0xFF; d[2]=(m1>>8)&0xFF; d[3]=m1&0xFF;
    d[4]=(m4>>24)&0xFF; d[5]=(m4>>16)&0xFF; d[6]=(m4>>8)&0xFF; d[7]=m4&0xFF;

    can_send(CAN_ID_ENCODER, d);
    vTaskDelay(pdMS_TO_TICKS(50));
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

  // ESP32Encoder handles PCNT internally
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_m1.attachHalfQuad(M1_ENA, M1_ENB);
  enc_m4.attachHalfQuad(M4_ENA, M4_ENB);
  enc_m1.clearCount();
  enc_m4.clearCount();

  ctrlMutex  = xSemaphoreCreateMutex();
  accelMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(can_rx_task,     "rx",   2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(control_task,    "ctrl", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(accel_task,      "acl",  2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(can_tx_task,     "tx",   2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(fault_tx_task,   "flt",  1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(encoder_tx_task, "enc",  2048, NULL, 2, NULL, 0);

  Serial.println("[BOOT] Ready");
}

void loop() { vTaskDelete(NULL); }

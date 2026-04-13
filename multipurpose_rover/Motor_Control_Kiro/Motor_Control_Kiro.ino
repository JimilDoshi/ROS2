/************************************************************
 * ESP32 Motor Driver ECU
 * MPU6050 IMU (accel + gyro) + Encoders + CAN
 *
 * CAN frames sent:
 *   0x101 ← received cmd (x, y, speed, enable, mode)
 *   0x200 → IMU accel  (ax, ay, az as int16 × 100)
 *   0x201 → encoders   (m1, m4 as int16)
 *   0x202 → IMU gyro   (gx, gy, gz as int16 × 100, rad/s)
 *   0x300 → fault status
 ************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <MPU6050_light.h>
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

#define CAN_ID_CMD      0x101
#define CAN_ID_ACCEL    0x200   // ax, ay, az
#define CAN_ID_ENCODER  0x201   // m1, m4 ticks
#define CAN_ID_GYRO     0x202   // gx, gy, gz
#define CAN_ID_FAULT    0x300

/* ================= FAULT FLAGS ================= */
#define FAULT_CAN_TIMEOUT  0
#define FAULT_IMU_ERROR    1

/* ================= STRUCTS ================= */
typedef struct {
  int16_t x, y;
  uint8_t speed, enable, mode;
} ControlData_t;

typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
} ImuData_t;

/* ================= GLOBALS ================= */
ControlData_t ctrl = {0};
ImuData_t imu_data = {0};
SemaphoreHandle_t imuMutex;
uint32_t last_rx_time = 0;
uint8_t faultStatus = 0;

MPU6050 mpu(Wire);

// Low-pass filter alpha — 0.2 = smooth, 0.5 = responsive
#define LPF_A 0.2f
float lpf_ax=0, lpf_ay=0, lpf_az=0;
float lpf_gx=0, lpf_gy=0, lpf_gz=0;

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

/* ================= CAN ================= */
void can_send(uint32_t id, uint8_t *data, uint8_t len=8) {
  twai_message_t tx = {.identifier = id, .data_length_code = len};
  memcpy(tx.data, data, len);
  twai_transmit(&tx, pdMS_TO_TICKS(5));
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
  while(1) {
    if(twai_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK) {
      if(rx.identifier == CAN_ID_CMD && rx.data_length_code == 8) {
        ctrl.y      = (int16_t)((rx.data[0]<<8)|rx.data[1]);
        ctrl.x      = (int16_t)((rx.data[2]<<8)|rx.data[3]);
        ctrl.speed  = rx.data[4];
        ctrl.enable = rx.data[5];
        ctrl.mode   = rx.data[6];
        last_rx_time = millis();
        faultStatus &= ~(1 << FAULT_CAN_TIMEOUT);
      }
    } else {
      if(millis() - last_rx_time > 500)
        faultStatus |= (1 << FAULT_CAN_TIMEOUT);
    }
  }
}

/* ================= CONTROL TASK ================= */
void control_task(void *arg) {
  while(1) {
    if(millis() - last_rx_time > 500) ctrl.enable = 0;

    if(!ctrl.enable) {
      for(int i=1;i<=4;i++) set_motor(i,0,0);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    float mode_scale = (ctrl.mode==0)?0.3f:(ctrl.mode==1)?0.6f:1.0f;
    float throttle = (ctrl.x/100.0f)*ctrl.speed*mode_scale;
    float turn     = -(ctrl.y/100.0f)*ctrl.speed*mode_scale;

    int16_t left  = (int16_t)constrain(throttle+turn, -100, 100);
    int16_t right = (int16_t)constrain(throttle-turn, -100, 100);

    left  = (int16_t)((left /100.0f)*MAX_PWM);
    right = (int16_t)((right/100.0f)*MAX_PWM);

    if(left >0) left  = (left  < MIN_PWM)? MIN_PWM:left;
    if(left <0) left  = (left  >-MIN_PWM)?-MIN_PWM:left;
    if(right>0) right = (right < MIN_PWM)? MIN_PWM:right;
    if(right<0) right = (right >-MIN_PWM)?-MIN_PWM:right;
    if(abs(left) <10) left =0;
    if(abs(right)<10) right=0;

    set_motor(1, left >=0?1:-1, abs(left));
    set_motor(2, right>=0?1:-1, abs(right));
    set_motor(3, left >=0?1:-1, abs(left));
    set_motor(4, right>=0?1:-1, abs(right));

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ================= IMU TASK ================= */
void imu_task(void *arg) {
  while(1) {
    mpu.update();

    // Low-pass filter on all 6 axes
    lpf_ax = LPF_A*mpu.getAccX() + (1-LPF_A)*lpf_ax;
    lpf_ay = LPF_A*mpu.getAccY() + (1-LPF_A)*lpf_ay;
    lpf_az = LPF_A*mpu.getAccZ() + (1-LPF_A)*lpf_az;
    lpf_gx = LPF_A*mpu.getGyroX() + (1-LPF_A)*lpf_gx;
    lpf_gy = LPF_A*mpu.getGyroY() + (1-LPF_A)*lpf_gy;
    lpf_gz = LPF_A*mpu.getGyroZ() + (1-LPF_A)*lpf_gz;

    if(xSemaphoreTake(imuMutex, pdMS_TO_TICKS(5))) {
      imu_data.ax = lpf_ax; imu_data.ay = lpf_ay; imu_data.az = lpf_az;
      imu_data.gx = lpf_gx; imu_data.gy = lpf_gy; imu_data.gz = lpf_gz;
      xSemaphoreGive(imuMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz
  }
}

/* ================= IMU TX TASK ================= */
// Sends accel on 0x200 and gyro on 0x202 at 50Hz
void imu_tx_task(void *arg) {
  while(1) {
    if(xSemaphoreTake(imuMutex, pdMS_TO_TICKS(5))) {
      // Accel frame 0x200 — ax, ay, az as int16 × 100 (m/s²)
      uint8_t accel_data[8] = {0};
      int16_t ax = imu_data.ax * 100;
      int16_t ay = imu_data.ay * 100;
      int16_t az = imu_data.az * 100;
      accel_data[0]=ax>>8; accel_data[1]=ax;
      accel_data[2]=ay>>8; accel_data[3]=ay;
      accel_data[4]=az>>8; accel_data[5]=az;
      accel_data[6]=ctrl.speed;
      accel_data[7]=faultStatus;
      can_send(CAN_ID_ACCEL, accel_data);

      // Gyro frame 0x202 — gx, gy, gz as int16 × 100 (rad/s)
      uint8_t gyro_data[8] = {0};
      int16_t gx = imu_data.gx * 100;
      int16_t gy = imu_data.gy * 100;
      int16_t gz = imu_data.gz * 100;
      gyro_data[0]=gx>>8; gyro_data[1]=gx;
      gyro_data[2]=gy>>8; gyro_data[3]=gy;
      gyro_data[4]=gz>>8; gyro_data[5]=gz;
      can_send(CAN_ID_GYRO, gyro_data);

      xSemaphoreGive(imuMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz
  }
}

/* ================= FAULT TX TASK ================= */
void fault_tx_task(void *arg) {
  uint8_t d[8] = {0};
  while(1) {
    d[0] = faultStatus;
    can_send(CAN_ID_FAULT, d);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/* ================= ENCODER INIT ================= */
void encoder_init(pcnt_unit_t unit, int pin_a, int pin_b) {
  pcnt_config_t cfg = {
    .pulse_gpio_num = pin_a, .ctrl_gpio_num = pin_b,
    .lctrl_mode = PCNT_MODE_REVERSE, .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC, .neg_mode = PCNT_COUNT_DEC,
    .counter_h_lim = 32767, .counter_l_lim = -32768,
    .unit = unit, .channel = PCNT_CHANNEL_0,
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
    int16_t m1=0, m4=0;
    pcnt_get_counter_value(ENC_M1_UNIT, &m1);
    pcnt_get_counter_value(ENC_M4_UNIT, &m4);

    uint8_t d[8] = {0};
    d[0]=(m1>>8)&0xFF; d[1]=m1&0xFF;
    d[2]=(m4>>8)&0xFF; d[3]=m4&0xFF;
    can_send(CAN_ID_ENCODER, d);
    vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  motor_init();
  can_init();

  // MPU6050 init and calibration
  byte status = mpu.begin();
  if(status != 0) {
    faultStatus |= (1 << FAULT_IMU_ERROR);
    Serial.println("[IMU] MPU6050 init failed!");
  } else {
    Serial.println("[IMU] Calibrating — keep rover still...");
    mpu.calcOffsets(true, true);  // gyro + accel calibration
    Serial.println("[IMU] Calibration done");
  }

  encoder_init(ENC_M1_UNIT, M1_ENA, M1_ENB);
  encoder_init(ENC_M4_UNIT, M4_ENA, M4_ENB);

  imuMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(can_rx_task,     "rx",   2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(control_task,    "ctrl", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(imu_task,        "imu",  2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(imu_tx_task,     "itx",  2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(fault_tx_task,   "flt",  1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(encoder_tx_task, "enc",  2048, NULL, 2, NULL, 0);

  Serial.println("[BOOT] Ready");
}

void loop() { vTaskDelete(NULL); }

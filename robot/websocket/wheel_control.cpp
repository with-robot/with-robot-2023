/*
  ESP32CAM Robot Car
  app_httpd.cpp (requires esp32cam-robot.ino)
  Based upon Espressif ESP32CAM Examples
  Uses TBA6612FNG H-Bridge Controller

  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/
#include <map>
#include "Arduino.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "wheel_control.h"

std::map<String, int> directionMap;
WheelControl cntl;
// Define Speed variables
int speed = 255;
int noStop = 0;

volatile unsigned int motor_speed = 50;
volatile unsigned long move_interval = 1000;
unsigned int _map_range_max(unsigned char val, int in_range, int out_max);

void _bind_channel(uint8_t channel, uint8_t pin, double freq, uint8_t res, uint32_t val);

esp_err_t cmd_handler(CommandCode code, unsigned char value, int speed_delta)
{
  int val = _map_range_max(value, 100, 255);
  int sp = 30 + speed_delta;
  int res = 0;

  Serial.printf("commandCode=%d, value=%x, speed=%d\n", code, value, speed);
  // previous_time = millis(); //요청받은 시간을 기준으로 한다.

  // Look at values within URL to determine function
  // if (!strcmp(variable, "framesize"))
  if (code == __framesize)
  {
    sensor_t *s = esp_camera_sensor_get();
    if (s->pixformat == PIXFORMAT_JPEG)
    {
      res = s->set_framesize(s, (framesize_t)val);
    }

    Serial.printf("framesize[%d]\n", res);
  }
  else if (code == __quality)
  {
    sensor_t *s = esp_camera_sensor_get();
    res = s->set_quality(s, val);
    Serial.printf("quality: [%d]\n", res);
  }
  else if (code == __flash)
  {
    ledcWrite(7, val);
  }
  else if (code == __flashoff)
  {
    ledcWrite(7, val);
  }
  else if (code == __speed)
  {
    ledcWrite(8, val); // #8
  }
  else if (code == __nostop)
  {
    noStop = val;
  }
  else if (code == __direction)
  {
    ledcWrite(8, sp); // #8
    cntl.operate(static_cast<int>(value));

    // if (value == 1)
    // {
    //   robot_fwd();
    // }
    // else if (value == 2)
    // {
    //   robot_left();
    // }
    // else if (value == 0)
    // {
    //   robot_stop();
    // }
    // else if (value == 3)
    // {
    //   robot_right();
    // }
    // else if (value == 4)
    // {
    //   robot_back();
    // }
    // if (noStop != 1)
    // {
    // }
  }
  else
  {
    Serial.println("variable");
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

void robot_setup()
{
  // Configure Control keys
  directionMap["forward"] = 1;
  directionMap["backward"] = 4;
  directionMap["left"] = 2;
  directionMap["right"] = 3;
  directionMap["stop"] = 0;

  // configure GPIO4 LED PWM functionalitites
  // channel, pin, freq, val
  uint8_t channel = 7;
  uint8_t pin = 4;
  uint8_t resolution = 8;
  _bind_channel(channel, pin, 1000, resolution, 0);

  // LED blinking for some time when it boots.
  for (int i = 0; i < 5; i++)
  {
    ledcWrite(channel, 10); // flash led
    delay(50);
    ledcWrite(channel, 0);
    delay(50);
  }
  ledcWrite(channel, 0);

  // Motor uses PWM Channel 8
  // channel, pin, freq, val
  // 1 : forward, 2:backward, 3:left, 4:right, 0:stop
  channel = 8;
  pin = 12;
  uint32_t default_speed = 30;
  // uint8_t timer=((chan/2)%4);
  _bind_channel(channel, pin, 2000, resolution, default_speed);

  // Pins for Motor Controller
  pinMode(LEFT_M0, OUTPUT);
  pinMode(LEFT_M1, OUTPUT);
  pinMode(RIGHT_M0, OUTPUT);
  pinMode(RIGHT_M1, OUTPUT);

  // Make sure we are stopped
  robot_stop();

  Serial.println("robot wheels setup completed.");
}

// channel 연결
void _bind_channel(uint8_t channel, uint8_t pin, double freq, uint8_t res, uint32_t val)
{
  // ESP채널번호, PWM신호의 주파수, 듀티사이클 해상도(8비트;0~255)
  ledcSetup(channel, freq, res);
  // ESP채널8을 PWM(LED제어)핀에 연결한다.
  ledcAttachPin(pin, channel);
  // ESP채널번호, 듀티사이클 크기(클수록 속도가 빨라진다)
  ledcWrite(channel, val);
}

void robot_stop()
{
  Serial.println("stop");
  digitalWrite(LEFT_M0, LOW);
  digitalWrite(LEFT_M1, LOW);
  digitalWrite(RIGHT_M0, LOW);
  digitalWrite(RIGHT_M1, LOW);
}

void robot_fwd()
{
  Serial.println("Forward");
  digitalWrite(LEFT_M0, HIGH);
  digitalWrite(LEFT_M1, LOW);
  digitalWrite(RIGHT_M0, HIGH);
  digitalWrite(RIGHT_M1, LOW);
}

void robot_back()
{
  Serial.println("Backward");
  digitalWrite(LEFT_M0, LOW);
  digitalWrite(LEFT_M1, HIGH);
  digitalWrite(RIGHT_M0, LOW);
  digitalWrite(RIGHT_M1, HIGH);
}

// RIGHT_M0를 HIGH, RIGHT_M1를 LOW로 설정하면 정방향 회전
// RIGHT_M0를 LOW, RIGHT_M1를 HIGH로 설정하면 역방향 회전
// RIGHT_M0와 RIGHT_M1을 동일하게 설정하면(모두 HIGH 혹은 LOW) 정지
void robot_right()
{
  Serial.println("TurnRight");
  digitalWrite(LEFT_M0, HIGH);
  digitalWrite(LEFT_M1, LOW);
  digitalWrite(RIGHT_M0, LOW);
  digitalWrite(RIGHT_M1, HIGH);
}

void robot_left()
{
  Serial.println("TurnLeft");
  digitalWrite(LEFT_M0, LOW);
  digitalWrite(LEFT_M1, HIGH);
  digitalWrite(RIGHT_M0, HIGH);
  digitalWrite(RIGHT_M1, LOW);
}

// 범위변환 0~100 => 0~255
unsigned int _map_range_max(unsigned char val, int in_range, int out_max)
{
  if (val > in_range)
    val = in_range;
  else if (val < 0)
    val = 0;
  return map(val, 0, in_range, 0, out_max);
}

WheelControl::WheelControl()
{
  // 배열 초기화
  op_cntrl[0] = {LOW, LOW, LOW, LOW};   // stop
  op_cntrl[1] = {HIGH, LOW, HIGH, LOW}; // forward
  op_cntrl[2] = {LOW, HIGH, HIGH, LOW}; // left
  op_cntrl[3] = {HIGH, LOW, LOW, HIGH}; // right
  op_cntrl[4] = {LOW, HIGH, LOW, HIGH}; // backward

  // 문자열 배열 초기화
  op_nm[0] = "stop";
  op_nm[1] = "forward";
  op_nm[2] = "left";
  op_nm[3] = "right";
  op_nm[4] = "backward";
}
// 모터 동작
void WheelControl::operate(int op_cmd)
{
  Serial.println(op_nm[op_cmd]);

  std::array<byte, 4> pos = op_cntrl[op_cmd];

  digitalWrite(LEFT_M0, pos[0]);
  digitalWrite(LEFT_M1, pos[1]);
  digitalWrite(RIGHT_M0, pos[2]);
  digitalWrite(RIGHT_M1, pos[3]);
}
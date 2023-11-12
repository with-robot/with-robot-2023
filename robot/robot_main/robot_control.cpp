/*
  ESP32CAM Robot Car
  app_httpd.cpp (requires esp32cam-robot.ino)
  Based upon Espressif ESP32CAM Examples
  Uses TBA6612FNG H-Bridge Controller

  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/

// #include <esp32-hal-ledc.h>
#include "esp_timer.h"
#include "esp_camera.h"
#include "Arduino.h"
#include "robot_control.h"

// TB6612FNG H-Bridge Connections (both PWM inputs driven by GPIO 12)
#define MTR_PWM 12
#define LEFT_M0 15
#define LEFT_M1 13
#define RIGHT_M0 14
#define RIGHT_M1 2

// Define Speed variables
int speed = 255;
int noStop = 0;

// Setting Motor PWM properties
const int freq = 2000;
const int motorPWMChannnel = 8;
const int lresolution = 8;

volatile unsigned int motor_speed = 200;
volatile unsigned long previous_time = 0;
volatile unsigned long move_interval = 250;

// Placeholder for functions
void robot_setup();
void robot_stop();
void robot_fwd();
void robot_back();
void robot_left();
void robot_right();
unsigned int map_range_255(unsigned int val);
uint8_t robo = 0;

enum ststate
{
  fwd,
  rev,
  stp //stop
};
ststate actstate = stp;

esp_err_t cmd_handler(CommandCode code, unsigned int value)
{  
  int val = map_range_255(value);
  int res = 0;
  previous_time = millis(); //요청받은 시간을 기준으로 한다.

  robot_setup();

  // Look at values within URL to determine function
  // if (!strcmp(variable, "framesize"))
  if (code == __framesize)
  {
    sensor_t *s = esp_camera_sensor_get();    
    if (s->pixformat == PIXFORMAT_JPEG) {
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
  else if (code == speed)
  {
    ledcWrite(motorPWMChannnel, val); // #8
  }
  else if (code == __nostop)
  {
    noStop = val;
  }
  else if (code == __direction)
  {
    if (value == 1)
    {
      Serial.println("Forward");
      robot_fwd();
      
      robo = 1;
    }
    else if (value == 2)
    {
      Serial.println("TurnLeft");
      robot_left();
      robo = 1;
    }
    else if (value == 3)
    {
      Serial.println("Stop");
      robot_stop();
    }
    else if (value == 4)
    {
      Serial.println("TurnRight");
      robot_right();
      robo = 1;
    }
    else if (value == 5)
    {
      Serial.println("Backward");
      robot_back();
      robo = 1;
    }
    if (noStop != 1)
    {
    }
  }
  else
  {
    Serial.println("variable");
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

// 범위변환 0~100 => 0~255
unsigned int map_range_255(unsigned int val)
{
  if (val > 100)
    val = 100;
  else if (val < 0)
    val = 0;
  return map(val, 0, 100, 0, 255);
}

void robot_setup()
{
  // Pins for Motor Controller
  pinMode(LEFT_M0, OUTPUT);
  pinMode(LEFT_M1, OUTPUT);
  pinMode(RIGHT_M0, OUTPUT);
  pinMode(RIGHT_M1, OUTPUT);

  // Make sure we are stopped
  robot_stop();

  // Motor uses PWM Channel 8
  ledcAttachPin(MTR_PWM, 8);
  ledcSetup(8, 2000, 8);
  ledcWrite(8, 130);
}

void robot_stop()
{
  digitalWrite(LEFT_M0, LOW);
  digitalWrite(LEFT_M1, LOW);
  digitalWrite(RIGHT_M0, LOW);
  digitalWrite(RIGHT_M1, LOW);
}

void robot_fwd()
{
  digitalWrite(LEFT_M0, HIGH);
  digitalWrite(LEFT_M1, LOW);
  digitalWrite(RIGHT_M0, HIGH);
  digitalWrite(RIGHT_M1, LOW);
  
  if(previous_time - millis() > move_interval) {
    robot_stop();
  }
  previous_time = millis();
}

void robot_back()
{
  digitalWrite(LEFT_M0, LOW);
  digitalWrite(LEFT_M1, HIGH);
  digitalWrite(RIGHT_M0, LOW);
  digitalWrite(RIGHT_M1, HIGH);
  move_interval = 250;
  previous_time = millis();
}

void robot_right()
{
  digitalWrite(LEFT_M0, HIGH);
  digitalWrite(LEFT_M1, LOW);
  digitalWrite(RIGHT_M0, LOW);
  digitalWrite(RIGHT_M1, HIGH);
  move_interval = 100;
  previous_time = millis();
}

void robot_left()
{
  digitalWrite(LEFT_M0, LOW);
  digitalWrite(LEFT_M1, HIGH);
  digitalWrite(RIGHT_M0, HIGH);
  digitalWrite(RIGHT_M1, LOW);
  move_interval = 100;
  previous_time = millis();
}
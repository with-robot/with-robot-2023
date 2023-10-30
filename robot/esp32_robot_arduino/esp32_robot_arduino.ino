#include <stdio.h>
#include <string.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"

#define __WIFI_CLIENT__1
#ifdef __WIFI_CLIENT__
const char* ssid = "LEO24";
const char* password = "12911990";
#else
const char* ssid = "robot";
const char* password = "1234";
#endif 

#define LED_PIN 4

// command
byte CMD_CFG = 0x01;
byte NOTI_CAM = 0x81;
// error
byte REQUEST = 0x00;
byte RES_OK = 0x01;
byte UNK_CMD = 0xEE;

WiFiServer tcpServer(10000);
WiFiClient tcpCleint;

byte read_buf[256] = {0xFF, 0, };
byte send_buf[65536] = {0xFF, 0, };

esp_err_t initSerial() {
  Serial.begin(115200);
  while(!Serial) delay(500);
  Serial.setDebugOutput(true);
  Serial.println("starts and complete serial comm.");
  return ESP_OK;
}

esp_err_t initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  return esp_camera_init(&config);
}

esp_err_t initWiFi() {
#ifdef __WIFI_CLIENT__
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi IP address: ");
  Serial.println(WiFi.localIP());
#else
  WiFi.mode(WIFI_AP);
  // WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
#endif
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  return ESP_OK;
}

void setup() {
  esp_err_t err = initSerial();
  if (err != ESP_OK) {
    Serial.printf("Serial init failed with error 0x%x", err);
    return;
  }

  err = initCamera();
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  s->set_framesize(s, FRAMESIZE_SVGA);
  
  //전면 LED를 점등한다.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(1000);
  digitalWrite(LED_PIN, LOW);

  err = initWiFi();
  if (err != ESP_OK) {
    Serial.printf("WiFi init failed with error 0x%x", err);
    return;
  }
}

void sendTCP(byte cmd, byte seq, byte rsp, byte* buf, int len) {
  send_buf[0] = 0xFF;
  send_buf[1] = cmd;
  send_buf[2] = seq;
  send_buf[3] = rsp;
  for (int i = 0; i < 4; i++) {
    send_buf[i + 4] = 0xFF & (len >>  i * 8);
  }
  memcpy(&send_buf[8], buf, len);
  int checksum = 0;
  for (int i = 0; i < len + 8; i++) {
    checksum += send_buf[i];
  }
  send_buf[8 + len] = checksum & 0xFF;
  tcpCleint.write(send_buf, len + 9);
}

void setConfig(int len) {
  memcpy(send_buf, read_buf, len + 8);
  send_buf[3] = RES_OK;
}

void loop() {
  if (tcpServer.hasClient()) {
    if (!tcpCleint || !tcpCleint.connected()) {
      tcpCleint = tcpServer.available();
      Serial.println("accept new Connection ...");
    } else {
      WiFiClient temp = tcpServer.available();
      temp.stop();
      Serial.println("reject new Connection ...");
    }
  }

  if (!tcpCleint || !tcpCleint.connected()) {
    delay(100);
    return;
  }

  int count = 0;
  while(tcpCleint.connected()) {
    // recv command msg and send response
    if (tcpCleint.available()) {
      // recv command
      tcpCleint.read(read_buf, 8);
      if (read_buf[0] == 0xFF) {
        int len = 0;
        for (int i = 0; i < 4; i++) {
          len += (read_buf[i + 4] <<  i * 8);
        }
        tcpCleint.read(&read_buf[8], len + 1);
        int checksum = 0;
        for (int i = 0; i < len + 8; i++) {
          checksum += read_buf[i];
        }
        if ((0xFF & checksum) == read_buf[8 + len]) {
          if (read_buf[1] == CMD_CFG) {
            setConfig(len);
          } else {
            memcpy(send_buf, read_buf, len + 8);
            send_buf[3] = UNK_CMD;
          }
          // make response and send
          len = 0;
          for (int i = 0; i < 4; i++) {
            len += (send_buf[i + 4] <<  i * 8);
          }
          checksum = 0;
          for (int i = 0; i < len + 8; i++) {
            checksum += send_buf[i];
          }
          send_buf[8 + len] = checksum & 0xFF;
          tcpCleint.write(send_buf, len + 9);
          // Serial.print("recv data: ");
          // Serial.println(len);
        }
      }
    }

    // send camera image
    if (count == 0) { // count를 조절하면 이미지 셈플링 주기 조절 가능 (1초에 한번)
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
      } else {
        sendTCP(NOTI_CAM, 0, RES_OK, fb->buf, fb->len);
        // Serial.print("send cam: ");
        // Serial.println(fb->len);
      }

      if (fb) {
        esp_camera_fb_return(fb);
        fb = NULL;
      }  
    }
    count++;
    if (count == 100) {
      count = 0;
    }
    delay(10);
  }  

}

#include <cstdio>
#include <cstdint>
#include <string.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>

#include "Image_s.h"
#include "user_func.h"

#define __WIFI_CLIENT__1
#ifdef __WIFI_CLIENT__
const char *ssid = "LEO24";
const char *password = "12911990";
#else
const char *ssid = "robot";
const char *password = "1234";
#endif

#define LED_PIN 4
// user_func list
void readTCP();
void sendTCP(byte cmd, byte seq, byte rsp, byte* buf, int len);
void setConfig(int len);
void loop_delay(int *count, int limit);

// command
byte CMD_CFG = 0x01;
byte NOTI_CAM = 0x81;
// error
byte REQUEST = 0x00;
byte RES_OK = 0x01;
byte UNK_CMD = 0xEE;

WiFiServer tcpServer(10000);
WiFiClient tcpClient;
esp_err_t err;

byte _buf[256] = {
    0xFF,
    0,
};
byte send_buf[65536] = {
    0xFF,
    0,
};

esp_err_t initSerial()
{
    Serial.begin(115200);
    while (!Serial)
        delay(500);
    Serial.setDebugOutput(true);    
    return ESP_OK;
}

esp_err_t initWiFi()
{
#ifdef __WIFI_CLIENT__
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
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

void setup()
{
    Serial.println("initSerial starts...........");
    err = initSerial();
    if (err!= ESP_OK) {
        Serial.printf("Serial init failed with error 0x%x\n", err);
        return;
    }
    Serial.println("setUpCamera starts...........");
    err = setUpCamera();
    if (err!= ESP_OK) {
        Serial.printf("set-Up camera failed with error 0x%x\n", err);
        return;
    }
    Serial.println("initWiFi starts...........");
    esp_err_t err = initWiFi();
    if (err!= ESP_OK)
    {
        Serial.printf("WiFi init failed with error 0x%x\n", err);
        return;
    }
    // 전면 LED를 점등한다.
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    delay(1000);
    digitalWrite(LED_PIN, LOW);
    Serial.println("setup completed......");
}

void loop()
{    
    if (tcpServer.hasClient())    
    {
        if (!tcpClient || !tcpClient.connected())
        {
            tcpClient = tcpServer.available();
            Serial.println("accept new Connection ...");
        }
        else
        {
            WiFiClient temp = tcpServer.available();
            temp.stop();
            Serial.println("reject new Connection ...");
        }
    }

    if (!tcpClient || !tcpClient.connected())
    {
        delay(100);
        return;
    }

    int count = 0;
    while (tcpClient.connected())
    {
        // recv command msg and send response
        if (tcpClient.available())
        {
            byte _buf[256] = {
                0xFF,
                0,
            };
           Serial.println("tcpClient.available().....");
           readTCP(_buf);
           continue;
        }

        // send camera image
        if (count == -1)
        { // count를 조절하면 이미지 셈플링 주기 조절 가능 (1초에 한번)            
            Image_st *data = capture_image_();
            if (data->size > 0) {
                sendTCP(NOTI_CAM, 0, RES_OK, data->buf, data->size);                
            }
            free(data);
            
            // camera_fb_t *fb = esp_camera_fb_get();
            // if (fb) {
            //     sendTCP(NOTI_CAM, 0, RES_OK, fb->buf, fb->len);
                
            //     esp_camera_fb_return(fb);
            //     fb = NULL;
            // } else {
            //     Serial.println("Camera capture failed");
            //     // Serial.print("send cam: ");
            //     // Serial.println(fb->len);
            // }
        }

        loop_delay(&count, 100);        
    } 
}

void readTCP(byte *_buf)
{
    // recv command
    // len이 12468인 겨우, _buf = [x,x,x,x,108,48,0,0]
    tcpClient.read(_buf, 8);
    if (_buf[0] != 0xFF) {
        return;
    }
    
    int len = 0;
    for (int i = 0; i < 4; i++)
    {
        len += (_buf[i + 4] << i * 8);
    }
    tcpClient.read(&_buf[8], len + 1);

    int checksum = 0;
    for (int i = 0; i < len + 8; i++)
    {
        checksum += _buf[i];
    }
    if ((0xFF & checksum) == _buf[8 + len])
    {
        // 메시지 종류에 따라 로봇 조작한다.
        #define WHL_LFT 1
        #define WHL_RGT 2
        #define WHL_FWD 3
        #define WHL_BWD 4
        
        err = ESP_OK;
        if (_buf[1] == CMD_CFG)
        {
            setConfig(len);
        } else {
            err = ESP_ERR_FLASH_OP_FAIL;
        }

        byte *output = (byte *)(malloc((sizeof(byte)*(8+len))));

        // 결과를 반환한다. 
        output[0] = 0xFF;
        output[1] = NOTI_CAM;
        output[2] = 0;
        if (err !=ESP_OK) {
            output[3] = UNK_CMD; //OR ESP_ERR
        } else {
            output[3] = RES_OK;
        }
        
        int message = 1;
        for (int i = 0; i < 4; i++) {
            output[i + 4] = 0xFF & (message >>  i * 8);
        }
        output[8] = 134;
        byte message_len = 1;        
        int checksum = 0;
        for (int i = 0; i < message_len + 8; i++) {
            checksum += output[i];
        }       
        
        output[8 + message_len] = checksum & 0xFF;
        
        tcpClient.write(output, message_len + 9);

        array_print("received data", _buf, len + 9);
        array_print("sent data", output, message_len + 9);
    }

}

void array_print(char *title, byte *arr, int len) {
    Serial.printf("%s : ", title);
    for(int i=0; i < len; i++) {
        Serial.printf("%02X ", arr[i]);
    }
    Serial.printf("\n");
}

void sendTCP(byte cmd, byte seq, byte rsp, byte* buf, int len) {
  // 헤더 8bytes, body: len's bytes, checksum: 1byte
  byte *send_buf = (byte *)(malloc(sizeof(byte)*(1+8+len)));
    
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
  tcpClient.write(send_buf, len + 9);
}

void setConfig(int len) {
  memcpy(send_buf, _buf, len + 8);
  send_buf[3] = RES_OK;
}

// 카운터
void loop_delay(int *count, int limit) {
        (*count)++;
        if (*count == limit)
        {
            *count = 0;
        }
        delay(10);
    }

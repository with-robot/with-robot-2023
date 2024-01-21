/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-websocket-server-arduino/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
#define ESP32
#define __cplusplus 201103L

// Import required libraries
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdio.h>
#include "esp_err.h"
// #include "esp_log.h"
#include "esp_spiffs.h"
#include "websocket.h"

// Replace with your network credentials
const char *ssid = "LEO24";
const char *password = "12911990";

int freq = 1000;
int ledChannel = 7; // ledc channel(PWM control unit) number 7
int resolution = 8; // ledc pwm resolution
int ledPin = 4;

bool ledState = 0;
bool GPIO0_State = 0;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void notifyClients()
{
  ws.textAll(String(ledState));
}

void handleWebSocketMessage(AwsFrameInfo *info, uint8_t *data, size_t len)
{
  // AwsFrameInfo *info = (AwsFrameInfo*)arg;
  Serial.println("handleWebSocketMessage1");
  if (is_ready(info, len) == true)
  {
    Serial.println("handleWebSocketMessage2");
    // data[len] = 0;
    if (info->opcode == WS_TEXT && convert2string(data, len) == "toggle")
    {
      Serial.println("handleWebSocketMessage3");
      // if (info->opcode == WS_TEXT && strcmp((char *)data, "toggle") == 0)
      // {
      change_state(!ledState);
      notifyClients();
    }
  }
}

void change_state(bool v)
{
  ledState = v;
}

bool is_ready(AwsFrameInfo *info, size_t len)
{
  return info->final && info->index == 0 && info->len == len;
}

// 문자열이 들어있는 buffer를 string타입으로 변환시킨다.
std::string convert2string(const uint8_t *data, size_t len)
{
  //   std::stringstream ss;
  //   // 16진수 0으로 초기화시킨다.
  //   ss << std::hex << std::setfill('0');

  //   for (size_t i = 0; i < len; ++i)
  //   {
  //     ss << std::setw(2) << static_cast<int>(data[i]);
  //   }

  //   Serial.printf("input string: %s\n", ss.str().c_str());

  //   return ss.str();

  // 입력 데이터가 유효한지 확인
  if (data == nullptr || len == 0)
  {
    // 유효하지 않은 입력 데이터 처리
    return std::string();
  }

  // std::stringstream을 사용하여 uint8_t 배열을 std::string으로 변환
  std::stringstream ss;
  for (size_t i = 0; i < len; ++i)
  {
    ss << static_cast<char>(data[i]);
  }

  // uint8_t 배열을 std::string으로 변환
  // return std::string(reinterpret_cast<const char *>(data), len);
  return ss.str();
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
  // Serial.printf("event_type:%d\n", (int)type);
  // Serial.printf("data:%s\n", (char *)data);
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage((AwsFrameInfo *)arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void initSPIFFS()
{
  esp_vfs_spiffs_conf_t conf = {
      .base_path = "/data",
      .partition_label = NULL,
      .max_files = 2,
      .format_if_mount_failed = true};

  esp_err_t ret = esp_vfs_spiffs_register(&conf);

  if (ret != ESP_OK)
  {
    if (ret == ESP_FAIL)
    {
      ESP_LOGE(TAG, "Failed to mount or format filesystem");
    }
    else if (ret == ESP_ERR_NOT_FOUND)
    {
      ESP_LOGE(TAG, "Failed  to find SPIFFS partition");
    }
    else
    {
      ESP_LOGE(TAG, "Failed to initialize SPIFFS (%S)", esp_err_to_name(ret));
    }
    return;
  }

  size_t total = 0, used = 0;
  ret = esp_spiffs_info(NULL, &total, &used);
  if (ret != ESP_OK)
  {
    Serial.printf("Failed to get SPIFFS partition information (%s)\n", esp_err_to_name(ret));
  }
  else
  {
    Serial.printf("partition size: total: %d, used: %d\n", total, used);
  }
}
void initWifi()
{
  // STA모드
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());
}

void initWebSocket()
{
  Serial.println("initalize websocket");
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

String processor(const String &var)
{
  if (var == "STATE")
  {
    Serial.println(var);
    if (ledState)
    {
      return "ON";
    }
    else
    {
      return "OFF";
    }
  }
  return String();
}

void setup()
{
  Serial.println('setup starts');
  // Serial port for debugging purposes
  Serial.begin(115200);

  // configure GPIO4 LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 0);

  // configure GPIO0 for LED toggling
  pinMode(0, INPUT);

  // mount SPIFFS
  initSPIFFS();

  // Connect to Wi-Fi
  initWifi();

  initWebSocket();

  // Route for root / web page
  server.on("/tt", HTTP_GET, [](AsyncWebServerRequest *request)
            { char *index_html = getIndexHtml();
              request->send_P(200, "text/html", index_html, processor); 
              delete[] index_html;
              index_html =nullptr;
              esp_vfs_spiffs_unregister(NULL); });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              Serial.println("Opening file");
              std::ifstream file("/data/index.html");

              if (!file.is_open())
              {
                Serial.println("Failed to open file for reading");
                return;
              }
              std::ostringstream buffer;
              buffer << file.rdbuf();

              file.close();

              const char *index_html = buffer.str().c_str();

              Serial.printf("index file loaded: %s\n", index_html);

              request->send_P(200, "text/html", index_html, processor);

              index_html = nullptr; });

  // Start server
  server.begin();
}

void loop()
{
  ws.cleanupClients();

  ledcWrite(ledChannel, (int)(ledState));

  // CSI_MCLK
  GPIO0_State = digitalRead(0);
  if (GPIO0_State == 0)
  {
    ledState = !ledState;
    notifyClients();
    delay(300);
  }
}

/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-websocket-server-arduino/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
// #include <iostream>
#include <sstream>
// #include <istream>
#include <vector>
#include <map>
#include "websocket.h"
// Import required libraries
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_err.h"
// #include "esp_log.h"
#include "esp_spiffs.h"
#include "wheel_control.h"
#include <cstdlib>

extern std::map<String, int> directionMap;
bool ledState = false;

// for string delimiter
std::vector<std::string> split(std::string s, std::string delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

// 바이트 데이터를 STL을 이용해 string타입으로 전환.
String _to_string(const uint8_t *data, size_t len)
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
    return String();
  }

  // std::stringstream을 사용하여 uint8_t 배열을 std::string으로 변환
  std::stringstream ss;
  for (size_t i = 0; i < len; ++i)
  {
    ss << static_cast<char>(data[i]);
  }
  // uint8_t 배열을 std::string으로 변환
  // return std::string(reinterpret_cast<const char *>(data), len);
  return String(ss.str().c_str());
}

bool _is_ready(AwsFrameInfo *info, size_t len)
{
  return info->final && info->index == 0 && info->len == len;
}

esp_err_t handleWebSocketMessage(AwsFrameInfo *info, uint8_t *data, size_t len)
{

  esp_err_t result = ESP_FAIL;

  if (_is_ready(info, len) != true || info->opcode != WS_TEXT)
  {
    return result;
  }

  String msg = _to_string(data, len);
  std::vector<std::string> tokens = split(msg.begin(), ",");
  float fb, lf;
  int cmd = std::strtol(tokens[0].c_str(), nullptr, 10);

  Serial.printf("cmd=%d, msg=%s, token size=%d\n", cmd, msg.c_str(), tokens.size());

  if (tokens.size() != 3)
  {
    if (cmd == 0x82)
    {
      return ESP_OK; // health check
    }
    else
    {
      Serial.println("Invalid message format");
      return ESP_FAIL;
    }
    return ESP_FAIL;
  }
  else
  {
    fb = std::strtod(tokens[1].c_str(), nullptr); // 앞뒤
    lf = std::strtod(tokens[2].c_str(), nullptr); // 좌우

    Serial.printf("cmd=%d, val1=%f, val2=%f\n", cmd, fb, lf);

    // 로봇제어 호출
    CommandCode cmd_code;
    uint8_t message;

    if (cmd == 0x81)
    {
      cmd_code = __direction;
    }
    else
    {
      Serial.println("Invalid message data");
      return ESP_FAIL;
    }

    uint8_t direction = 0;
    uint8_t speed_delta = 0;
    if (fb > 0.001)
    {
      direction = 1; // 전진
      speed_delta = (fb - 1) * 10;
    }
    else if (fb < -0.001)
    {
      direction = 4; // 후진
      speed_delta = (1 - fb) * 10;
    }
    else if (lf > 0.01)
    {
      direction = 3; // right
      speed_delta = (lf - 1) * 10;
    }
    else if (lf < -0.01)
    {
      direction = 2; // left
      speed_delta = (1 - lf) * 10;
    };

    result = cmd_handler(cmd_code, direction, speed_delta);
    Serial.printf("cmd_handler called %d\n", result);
  }

  // auto it = directionMap.find(msg);

  // Serial.println(msg);
  // Serial.printf("direction: %s,", (char *)msg.c_str());
  // Serial.printf("map key: %s,", it->first);
  // Serial.printf("map val: %d\n", (int)it->second);
  // if (it != directionMap.end())
  // {
  //   // 로봇제어 호출
  // result = cmd_handler(__direction, it->second);
  //   Serial.printf("cmd_handler called %d\n", result);
  //   result = ESP_OK;
  // }
  return result;
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

String processor(const String &var)
{
  String return_msg{""};
  if (var == "STATE")
  {
    return_msg = (ledState) ? "ON" : "OFF";
    ledState = !ledState;
  }

  Serial.printf("var=%s, msg=%s", var, return_msg);
  return return_msg;
}

// 가상폴더에 public으로 마운트하도록 설정한다.
// void initSPIFFS(String base_path = "./public")
// {
//   esp_vfs_spiffs_conf_t conf = {
//       .base_path = base_path,
//       .partition_label = NULL,
//       .max_files = 10,
//       .format_if_mount_failed = true};

//   esp_err_t ret = esp_vfs_spiffs_register(&conf);

//   if (ret != ESP_OK)
//   {
//     if (ret == ESP_FAIL)
//     {
//       Serial.println("Failed to mount or format filesystem");
//     }
//     else if (ret == ESP_ERR_NOT_FOUND)
//     {
//       Serial.println("Failed  to find SPIFFS partition");
//     }
//     else
//     {
//       Serial.printf("Failed to initialize SPIFFS (%S)\n", esp_err_to_name(ret));
//     }
//     return;
//   }

//   size_t total = 0, used = 0;
//   ret = esp_spiffs_info(NULL, &total, &used);
//   if (ret != ESP_OK)
//   {
//     Serial.printf("Failed to get SPIFFS partition information (%s)\n", esp_err_to_name(ret));
//   }
//   else
//   {
//     Serial.printf("partition size: total: %d, used: %d\n", total, used);
//   }
// }

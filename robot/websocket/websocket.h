// tools for websocket connection
// 2024-02-08
#define ESP32

#define __cplusplus 201103L
#include <Arduino.h>
#include "esp_err.h"
#include <ESPAsyncWebServer.h>

// index 불러오기
char *getIndexHtml();

// 웹소켓 연결 처리
esp_err_t handleWebSocketMessage(AwsFrameInfo *info, uint8_t *data, size_t len);

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len);
String processor(const String &var);

// void initSPIFFS();

// 장치 초기화
void initWifi();
void initWebSocket();
// websocket 메시지 전송
void notifyClients();
void route();
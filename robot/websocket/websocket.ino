/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-websocket-server-arduino/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
#define ESP32
// Import required libraries
// #include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h>
// #include <ESPAsyncWebServer.h>
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

PGM_P index_html = getIndexHtml();

void notifyClients()
{
  ws.textAll(String(ledState));
}

void handleWebSocketMessage(AwsFrameInfo *info, uint8_t *data, size_t len)
{
  // AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    // data[len] = 0;
    if (convert2string(data, len) == "toggle")
    {
      // if (strcmp((char*)data, "toggle") == 0) {
      ledState = !ledState;
      notifyClients();
    }
  }
}
// 문자열이 들어있는 buffer를 string타입으로 변환시킨다.
string convert2string(const uint_t *data, size_t len)
{
  std::stringstream ss;
  // 16진수 0으로 초기화시킨다.
  ss << std::hex << std::setfill('0');

  for (size_t i = 0; i < len; ++i)
  {
    ss << std::setw(2) << static_cast<int>(data[i])
  }

  return ss.str();
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
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

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

String processor(const String& var)
{
  Serial.println(var);
  if (var == "STATE")
  {
    if (ledState)
    {
      return "ON";
    }
    else
    {
      return "OFF";
    }
  }
  return var;
}

void setup()
{
  // Serial port for debugging purposes
  Serial.begin(115200);

  //  pinMode(ledPin, OUTPUT);
  //  digitalWrite(ledPin, LOW);

  // configure GPIO4 LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 0);

  // configure GPIO0 for LED toggling
  pinMode(0, INPUT);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());

  initWebSocket();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html, processor); });

  server.on("/action", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", "OK"); });

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

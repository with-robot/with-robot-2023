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
#include "esp_err.h"
#include "wheel_control.h"
#include "websocket.h"
#include "camera_image.h"
#include <map>
// Replace with your network credentials
const char *ssid = "LEO24";
const char *password = "12911990";

uint8_t ledPin = 4; // Camera FLASH
bool GPIO0_State = 0;
extern bool ledState;
Image_st data;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setup()
{
  // Serial port for debugging purposes
  Serial.begin(115200);
  Serial.println("setup starts");

  // Connect to Wi-Fi
  initWifi();
  // 구동장치 초기설정
  robot_setup();

  // mount SPIFFS
  // initSPIFFS();

  // Route for root / web page
  add_route();

  // Start server
  start_server();

  // 카메라설정
  setup_camera();
}

void loop()
{
  ws.cleanupClients();

  ledcWrite(ledPin, (uint8_t)(ledState));

  esp_err_t result = capture_image(data);

  if (result == ESP_OK)
  {
    // ws.binaryAll(reinterpret_cast<char *>(data.buf.get()), data.size);
    ws.binaryAll(data.buf, data.size);
    data.destroy();
    delay(100);
  }

  // CSI_MCLK
  GPIO0_State = digitalRead(0);
  if (GPIO0_State == 0)
  {
    ledState = !ledState;
    // notifyClients();
    delay(100);
  }
}

void initWifi()
{
  // STA모드
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(800);
    Serial.print(".");
  }
  // Print ESP Local IP Address
  Serial.printf("\nConnecting to WiFi with %s\n", WiFi.localIP().toString().c_str());
}

void start_server()
{
  Serial.println("initalize a websocket");
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.begin();
  Serial.println("webserver has started");
}

void add_route()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              char *index_html = getIndexHtml();
              request->send_P(200, "text/html", index_html, processor);
              delete[] index_html;
              index_html = nullptr;

              // struct stat st;
              // if (stat("/public/index.htm", &st) == 0)
              // {
              //   // Delete it if it exists
              //   unlink("/public/index.htm");
              // }
              // esp_vfs_spiffs_unregister(NULL);
            });

  // server.on("/bak", HTTP_GET, [](AsyncWebServerRequest *request)
  //           {
  //           Serial.println("start to open the index file");
  //           std::ifstream file("/public/index.htm");

  //           if (!file.is_open())
  //           {
  //             Serial.println("Failed to open file for reading");
  //             return;
  //           }
  //           std::ostringstream buffer;
  //           buffer << file.rdbuf();

  //           file.close();

  //           const char *index_html = buffer.str().c_str();

  //           Serial.printf("index file loaded: %s\n", index_html);

  //           request->send_P(200, "text/html", index_html, processor); });
}

void notifyClients()
{
  ws.textAll(String(ledState));
}

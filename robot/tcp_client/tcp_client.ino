#include <cstdio>
#include <cstdint>
#include <string.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>
// #include <esp32-hal-ledc.h>
#include "camera_image.h"
#include "wheel_control.h"

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
void sendTCP(byte cmd, byte seq, byte rsp, byte *buf, int len);
void setConfig(int len);
void loop_delay(int *count, int limit);
void timerCallback(void *arg);

// command
byte CMD_CFG = 0x01;  // 구동명령
byte CMD_DCF = 0x02;  // 구독중지
byte NOTI_CAM = 0x81; // 통지_이미지
byte NOTI_IMU = 0x82; // 통지_위치정보
// error
byte REQUEST = 0x00;
byte RES_OK = 0x01;
byte RES_NK = 0x02;
byte UNK_CMD = 0xEE;

#define MAX_CLIENTS 50

WiFiServer tcpServer(10000);
WiFiClient *clients[MAX_CLIENTS] = {NULL};
WiFiClient tcpClient;
esp_err_t result;

extern uint8_t robo;
extern volatile unsigned long move_interval;
extern volatile unsigned int motor_speed;
unsigned long previous_time;

byte _buf[256] = {
    0xFF,
    0,
};
byte send_buf[65536] = {
    0xFF,
    0,
};

Image_st *data;
CmdReq_st *cmd_req_s;

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

// esp_timer_handle_t timer;
// esp_timer_create_args_t timerArgs = {
//     .callback = &timerCallback,
//     .arg = nullptr,
//     .name = "stop_car"};

void setup()
{
    Serial.println("initSerial starts...........");
    result = initSerial();
    if (result != ESP_OK)
    {
        Serial.printf("Serial init failed with error 0x%x\n", result);
        return;
    }
    Serial.println("setUpCamera starts...........");
    result = setUp_camera();
    if (result != ESP_OK)
    {
        Serial.printf("set-Up camera failed with error 0x%x\n", result);
        return;
    }
    Serial.println("initWiFi starts...........");
    esp_err_t err = initWiFi();
    if (err != ESP_OK)
    {
        Serial.printf("WiFi init failed with error 0x%x\n", err);
        return;
    }
    // 전면 LED를 점등한다.

    digitalWrite(LED_PIN, HIGH);
    pinMode(LED_PIN, OUTPUT);

    delay(1000);
    digitalWrite(LED_PIN, LOW);
    Serial.println("setup completed......");

    // 정지처리 등록
    // esp_timer_create(&timerArgs, &timer);

    data = (Image_st *)malloc(sizeof(Image_st));
    cmd_req_s = (CmdReq_st *)(malloc(sizeof(CmdReq_st)));

    // 휠 초기화
    robot_setup();

    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        clients[i] = new WiFiClient();
    }
}

void timerCallback(void *arg)
{
    robot_stop();
    Serial.println('robot stopped.');
}

void loop()
{
    WiFiClient client_;
    // 주기적으로 바퀴를 정지시키다.
    // esp_timer_start_periodic(timer, 250000);
    if (robo)
    {
        unsigned long currentMillis = millis();
        if (currentMillis - previous_time >= move_interval)
        {
            Serial.printf("로봇을 정지시켰습니다. %u - %u > %u\n", currentMillis, previous_time, move_interval);
            robot_stop();
            // char rsp[32];
            // sprintf(rsp, "SPPED: %d", motor_speed);
            Serial.println("Stop");
            robo = 0;
            // byte *rsp_ = (byte *)malloc(sizeof(byte) * 255)
            // mcmcpy(rsp_, rsp, 32);
            // sendTCP(CMD_CFG, 0, result, rsp, 32);
        }
    }

    if (tcpServer.hasClient())
    {
        // if (!tcpClient || !tcpClient.connected())
        // {
        WiFiClient newClient = tcpServer.available();
        for (int i = 0; i < MAX_CLIENTS; ++i)
        {
            if (!clients[i]->connected())
            {
                clients[i] = new WiFiClient(newClient);
                break;
            }
        }
        Serial.println("accept new Connection ...");
    }

    // 클라이언트가 사용 중이고 데이터를 가지고 있는 경우 해당 클라이언트를 선택합니다.
    for (int i = 0; i < MAX_CLIENTS; ++i)
    {
        // if (NULL !=clients[i])
        // {
        //     continue;
        // }
        if (!clients[i]->connected())
        {
            continue;
        }
        if (clients[i]->available())
        {
            client_ = *clients[i];
            Serial.printf("found tcpClient at %d\n", i);
            break;
        }
    }

    // 클라이언트가 없거나 연결이 끊어졌거나 데이터를 가지고 있지 않으면 함수를 종료합니다.
    // if (!tcpClient || !tcpClient.connected() || !tcpClient.available())
    if (!client_.available())
    {
        return;
    }

    tcpClient = client_;
    Serial.println("tcpClient.available.....");

    result = readTCP(cmd_req_s);
    if (result != ESP_OK)
    {
        Serial.println("readTCP return Fail.....");
        // 실패
        byte cmd;
        if (cmd_req_s->buf[1] == 0x53)
        {
            cmd = CMD_CFG;
        }
        else if (cmd_req_s->buf[1] == 0x81)
        {
            cmd = NOTI_CAM;
        }

        sendTCP(cmd, 0, result, NULL, 0);
        free(cmd_req_s);
        return;
    }

    if (cmd_req_s->buf[1] == 0x53)
    {
        previous_time = millis();
        // 로봇제어 호출
        result = cmd_handler(cmd_req_s->cmd, cmd_req_s->val);
        if (result == ESP_OK)
        {
            sendTCP(CMD_CFG, 0, RES_OK, NULL, 0);
        }
        else
        {
            Serial.printf("CMD Handler err %x\n", result);
            sendTCP(CMD_CFG, 0, RES_NK, NULL, 0);
        }
    }
    else if (cmd_req_s->buf[1] == 0x81)
    {
        // 사진 전송
        result = capture_image(data);
        if (result == ESP_OK)
        {
            sendTCP(NOTI_CAM, 0, RES_OK, data->buf, data->size);
            free(data->buf);
        }
        else
        {
            sendTCP(NOTI_CAM, 0, RES_NK, NULL, 0);
        }
    }
}

esp_err_t readTCP(CmdReq_st *cmd_req_s)
{
    byte *_buf = cmd_req_s->buf;
    // recv command
    // len이 12468인 겨우, _buf = [x,x,x,x,108,48,0,0]
    tcpClient.read(_buf, 8);
    if (_buf[0] != 0xFF)
    {
        Serial.println("_buf[0] != 0xFF");
        return ESP_ERR_INVALID_ARG;
    }
    array_print("입력값", _buf, 8);

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
    Serial.printf("checksum: %x\n", checksum);

    if ((0xFF & checksum) != _buf[8 + len])
    {
        return ESP_ERR_INVALID_CRC;
    }

    cmd_req_s->cmd = (CommandCode)_buf[1];
    cmd_req_s->size = len;
    cmd_req_s->val = _buf[8];

    // 로그
    Serial.printf("cmd=%x, val=%d\n", cmd_req_s->cmd, cmd_req_s->val);
    if (_buf[1] == 0x53)
    {
        array_print("received data", _buf, len + 9);
    }

    return ESP_OK;
}

void sendTCP(byte cmd, byte seq, byte rsp, byte *data, int len)
{
    // 헤더 8bytes, body: len's bytes, checksum: 1byte
    byte *send_buf = (byte *)(malloc(sizeof(byte) * (1 + 8 + len)));
    send_buf[0] = 0xFF;
    send_buf[1] = cmd;
    send_buf[2] = seq;
    send_buf[3] = rsp;

    array_print("sendTCP", send_buf, 4);

    // 데이터 건수 기록
    for (int i = 0; i < 4; i++)
    {
        send_buf[i + 4] = 0xFF & (len >> i * 8);
    }

    if (data != nullptr && len > 0)
    {
        memcpy(&send_buf[8], data, len);
    }
    else
    {
        len = 0;
    }

    int checksum = 0;
    for (int i = 0; i < len + 8; i++)
    {
        checksum += send_buf[i];
    }
    send_buf[8 + len] = checksum & 0xFF;

    tcpClient.write(send_buf, len + 9);
    if (_buf[1] == 0x01)
    {
        array_print("sendTCP completed", send_buf, len + 9);
    }

    free(send_buf);
}

void setConfig(int len)
{
    memcpy(send_buf, _buf, len + 8);
    send_buf[3] = RES_OK;
}

// 카운터
void loop_delay(int *count, int limit)
{
    (*count)++;
    if (*count == limit)
    {
        *count = 0;
    }
    delay(10);
}

// 임시
esp_err_t temp(char *_buf, int len)
{
    byte _tmp[256];
    // 데이터부만을 추출한다.
    memcpy(_tmp, &_buf[8], len);
    _tmp[len] = '\0';
    char *message = (char *)malloc(sizeof(char) * (len + 1));
    memcpy(message, _tmp, len);
    message[len] = '\0';

    Serial.printf("message: %s\n", message);

    char *cmd = strtok(message, "/");
    char *val = strtok(NULL, "/");

    Serial.printf("%s = %s / %s : ", message, cmd, val);

    if (val == nullptr)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    try
    {
        int cmd_len = strlen(cmd);
        // cmd_req_s->cmd = (char *)malloc(sizeof(char) * (cmd_len + 1));
        // strcpy(cmd_req_s->cmd, cmd);
        // cmd_req_s->val = atoi(val);
    }
    catch (const std::exception &e)
    {
        return ESP_ERR_NO_MEM;
    }
}

void array_print(char *title, byte *arr, int len)
{
    Serial.printf("%s : ", title);
    for (int i = 0; i < len; i++)
    {
        Serial.printf("%02X ", arr[i]);
    }
    Serial.printf("\n");
}

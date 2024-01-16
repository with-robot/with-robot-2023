#include <stdio.h>
#include <micro_ros_arduino.h>
#include <arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <memory>
#include <sensor_msgs/msg/image.h>
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "esp_camera.h"
#include "camera_pins.h"
#include "img_converters.h"

#define SSID "LEO24"
#define SSID_PW "12911990"
#define AGENT_IP "192.168.55.4"
#define AGENT_PORT 2018

rcl_publisher_t publisher;
sensor_msgs__msg__Image msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

typedef struct camera_Image
{  
  size_t size;
  uint8_t* data; 
} CameraImage_st;

#define create_camera_image() { 0, NULL }

#define LED_PIN 4

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

void setup()
{
  // 시리얼통신 초기화
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("starts and complete serial comm.");
  //wifi접속
  set_microros_wifi_transports(SSID, SSID_PW, AGENT_IP, AGENT_PORT);
  //전면 LED를 점등한다.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(1000);
  digitalWrite(LED_PIN, LOW);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
      "camera_image"));

  // 카메라 설정
  camera_config_t camera_config = get_camera_config();

  // camera init
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

// #if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
//   s->set_vflip(s, 1);
//   s->set_hmirror(s, 1);
// #endif
}

void loop()
{  
  delay(1000);  
  CameraImage_st st_img_data = create_camera_image();
  // uint8_t* buffer=NULL;
  // size_t size=0;

  Serial.println("capture camera image....\n");
  
  //이미지를 취득한다.
  // take_image_to(&buffer, &size);
  get_image_data(&st_img_data);

  if (!st_img_data.data)
  {
    Serial.println("buffer is null..\n");
    return;
  }
  
  std::shared_ptr<sensor_msgs__msg__Image> msg = std::make_shared<sensor_msgs__msg__Image>();
  create_image_message(msg, st_img_data);

  Serial.printf("[MAIN] data size: %d\n", (int)msg->data.size);

  if (msg!=NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, msg.get(), NULL));
  }    
  
  
  free(st_img_data.data);

  Serial.println("restarting....");
  delay(10000);
}

// 이미지메시지를 작성하여 반환한다.
void create_image_message(std::shared_ptr<sensor_msgs__msg__Image> msg, CameraImage_st st_img_data) {  

  rosidl_runtime_c__uint8__Sequence uint8_seq;
  rosidl_runtime_c__String encoding;
  // uint8_seq.data = (uint8_t*)malloc(size);
  // if(uint8_seq.data == NULL) {
  //   Serial.println("Memory allocation failed");
  //   return NULL;
  // }   
  // memcpy(uint8_seq.data, buffer, size);  // 이미지 데이터 복사    
  Serial.printf("image size: %d\n", (int)st_img_data.size);

  uint8_seq.data = st_img_data.data;
  uint8_seq.size = st_img_data.size;
  uint8_seq.capacity = st_img_data.size;
  
  msg->data = uint8_seq;
  encoding.data="bgr8";
  
  // std::string encodingString(encoding.data);

  msg->encoding = encoding;
  msg->height=240;
  msg->width=320;
  msg->step=320*3;
}

// 카메라 초기 설정
camera_config_t get_camera_config()
{
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

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (psramFound())
  {
    // config.frame_size = FRAMESIZE_UXGA;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }  
  
  Serial.printf("end configuration %d\n", (int)config.frame_size);  
  return config;
}

// 카메라 이미지 취득
void take_image_to(uint8_t** _jpg_buf, size_t* _jpg_buf_len)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  // size_t _jpg_buf_len = 0;
  // uint8_t *_jpg_buf = NULL;
  char *part_buf[64];
  int64_t fr_start = 0;
  int64_t fr_ready = 0;
  int64_t fr_face = 0;
  int64_t fr_recognize = 0;
  int64_t fr_encode = 0;

  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    res = ESP_FAIL;
  }
  else
  {
    // fr_start = esp_timer_get_time();  

    if (fb->format != PIXFORMAT_JPEG)
    {
      if (!frame2jpg(fb, 80, _jpg_buf, _jpg_buf_len))
      {
        Serial.println("JPEG compression failed");
        res = ESP_FAIL;
      }
    }
    else
    {
      *_jpg_buf_len = fb->len;
      *_jpg_buf = fb->buf;
    }
  }
  Serial.printf("Camera capturing: %d\n", *_jpg_buf_len);
  
  if (fb)
  {
    esp_camera_fb_return(fb);
    fb = NULL;
  }  
  if (res == ESP_FAIL)
  {
    Serial.println("ESP_FAIL");
    return;
  }
  Serial.printf("Camera capture complete: %d\n", *_jpg_buf_len);
}

// 카메라 이미지 취득
void get_image_data(CameraImage_st *image_t)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  // size_t _jpg_buf_len = 0;
  // uint8_t *_jpg_buf = NULL;
  char *part_buf[64];
  int64_t fr_start = 0;
  int64_t fr_ready = 0;
  int64_t fr_face = 0;
  int64_t fr_recognize = 0;
  int64_t fr_encode = 0;

  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    res = ESP_FAIL;
  }
  else
  {
    // fr_start = esp_timer_get_time();  

    if (fb->format != PIXFORMAT_JPEG)
    {
      if (!frame2jpg(fb, 80, image_t->data, image_t->size))
      {
        Serial.println("JPEG compression failed");
        res = ESP_FAIL;
      }
    }
    else
    {
      image_t->data = fb->buf;
      image_t->size = fb->len;      
    }
  }
  Serial.printf("Camera capturing: %d\n", image_t->size);
  
  if (fb)
  {
    esp_camera_fb_return(fb);
    fb = NULL;
  }  
  if (res == ESP_FAIL)
  {
    Serial.println("ESP_FAIL");
  } else {
    Serial.printf("Camera capture complete: %d\n", image_t->size);
  }  
}

// 에러발생 시 점멸처리
void error_loop()
{
  int cnt=0;
  while (cnt <3)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
    cnt++;
  }
}


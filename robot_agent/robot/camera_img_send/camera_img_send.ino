#include <stdio.h>

#include <micro_ros_arduino.h>
#include <arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
// #include <rclc/executor.h>
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

// 카메라 서버 기동
// void startCameraServer();

void setup()
{
  camera_config_t camera_config = get_camera_config();

  set_microros_wifi_transports(SSID, SSID_PW, AGENT_IP, AGENT_PORT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);
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

  // camera init
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
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

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
}

void loop()
{
  rosidl_runtime_c__String encoding;
  uint8_t *_jpg_buf = get_camera_image();
  if (!_jpg_buf)
  {
    return;
  }

  std::shared_ptr<sensor_msgs__msg__Image> msg = std::make_shared<sensor_msgs__msg__Image>();
  rosidl_runtime_c__uint8__Sequence uint8_seq;
  uint8_seq.data = _jpg_buf;
  uint8_seq.size = sizeof(_jpg_buf);
  uint8_seq.capacity = sizeof(_jpg_buf);
    
  msg->data = uint8_seq;

  encoding.data = "jpg";
  msg->encoding = encoding;

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  free(_jpg_buf);
  _jpg_buf = NULL;
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
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  return config;
}

// 카메라 이미지 취득
uint8_t *get_camera_image()
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
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
    fr_start = esp_timer_get_time();
    fr_ready = fr_start;
    fr_face = fr_start;
    fr_encode = fr_start;
    fr_recognize = fr_start;

    if (fb->format != PIXFORMAT_JPEG)
    {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if (!jpeg_converted)
      {
        Serial.println("JPEG compression failed");
        res = ESP_FAIL;
      }
    }
    else
    {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    // dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

    // if (!image_matrix)
    // {
    //   Serial.println("dl_matrix3du_alloc failed");
    //   res = ESP_FAIL;
    // }
    // else
    // {
    //   if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item))
    //   {
    //     Serial.println("fmt2rgb888 failed");
    //     res = ESP_FAIL;
    //   }
    //   else
    //   {
    //     fr_ready = esp_timer_get_time();
    //     if (fb->format != PIXFORMAT_JPEG)
    //     {
    //       if (!fmt2jpg(image_matrix->item, fb->width * fb->height * 3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len))
    //       {
    //         Serial.println("fmt2jpg failed");
    //         res = ESP_FAIL;
    //       }
    //       esp_camera_fb_return(fb);
    //       fb = NULL;
    //     }
    //     else
    //     {
    //       _jpg_buf = fb->buf;
    //       _jpg_buf_len = fb->len;
    //     }
    //     fr_encode = esp_timer_get_time();
    //   }
    //   dl_matrix3du_free(image_matrix);
    // }
    // }
  }
  if (res == ESP_FAIL)
  {
    return NULL;
  }

  if (fb)
  {
    esp_camera_fb_return(fb);
    fb = NULL;
  }
  return _jpg_buf;
}
// 에러발생 시 점멸처리
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(3000);
  }
}

// void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
// {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL)
//   {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     msg.data++;
//   }
// }

// int main(int argc, char const *argv[])
// {
//   /* code */
//   setup()

//   return 0;
// }

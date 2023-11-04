#ifndef CameraControl2_H
#define CameraControl2_H

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include <string.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "user_func.h"

esp_err_t initCamera()
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
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    return esp_camera_init(&config);
}

esp_err_t setUpCamera()
{
    esp_err_t err = initCamera();
    if (err != ESP_OK)        
        return err;

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID)
    {
        s->set_vflip(s, 1);       // flip it back
        s->set_brightness(s, 1);  // up the brightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }
    s->set_framesize(s, FRAMESIZE_SVGA);

    return ESP_OK;
}

Image_st* capture_image_() {
  Image_st *data = (Image_st*)malloc(sizeof(Image_st));
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
      data->buf = (uint8_t*)malloc(sizeof(uint8_t) * fb->len);
      memcpy(data->buf, fb->buf, fb->len);
      data->size = fb->len;
      esp_camera_fb_return(fb);
      fb = NULL;
  } 

  return data;
}


#endif
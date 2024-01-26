#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include <memory>
#include <string.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "camera_pins.h"
#include "camera_image.h"

esp_err_t init_camera()
{
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_1;
    config.ledc_timer = LEDC_TIMER_1;
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
    if (psramFound())
    {
        config.frame_size = FRAMESIZE_XGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    }
    else
    {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    return esp_camera_init(&config);
}

esp_err_t setUp_camera()
{
    esp_err_t err = init_camera();
    if (err != ESP_OK)
        return err;

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID)
    {
        s->set_vflip(s, 1);       // flip it back
        s->set_brightness(s, 1);  // up the brightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }
    s->set_framesize(s, FRAMESIZE_QVGA);

    s->set_brightness(s, 0);                 // -2 to 2
    s->set_contrast(s, 0);                   // -2 to 2
    s->set_saturation(s, 0);                 // -2 to 2
    s->set_special_effect(s, 0);             // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);                   // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);                   // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);                    // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);              // 0 = disable , 1 = enable
    s->set_aec2(s, 0);                       // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);                   // -2 to 2
    s->set_aec_value(s, 300);                // 0 to 1200
    s->set_gain_ctrl(s, 1);                  // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);                   // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0); // 0 to 6
    s->set_bpc(s, 0);                        // 0 = disable , 1 = enable
    s->set_wpc(s, 1);                        // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);                    // 0 = disable , 1 = enable
    s->set_lenc(s, 1);                       // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);                    // 0 = disable , 1 = enable
    s->set_vflip(s, 0);                      // 0 = disable , 1 = enable
    s->set_dcw(s, 1);                        // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);                   // 0 = disable , 1 = enable

    return ESP_OK;
}

esp_err_t capture_image(Image_st &data)
{
    sensor_t *s = esp_camera_sensor_get();
    // PIXFORMAT_JPEG: 3
    // Serial.printf("s-state:%d\n", s->pixformat);

    camera_fb_t *fb = esp_camera_fb_get();
    esp_err_t result = ESP_OK;
    if (!fb)
    {
        Serial.println("사진이미지 취득 실패!!!\n");
        result = ESP_ERR_CAMERA_BASE;
    }
    else
    {
        data.buf = std::unique_ptr<uint8_t[]>(new uint8_t[fb->len]);
        //(uint8_t *)malloc(sizeof(uint8_t) * fb->len);
        if (data.buf == NULL)
        {
            Serial.println("이미지 저장용 메모리 할당오류!!!\n");
            result = ESP_ERR_NO_MEM;
        }
        else
        {
            std::copy(fb->buf, fb->buf + fb->len, data.buf.get());
            // memcpy(data.buf, fb->buf, fb->len);
            data.size = fb->len;
            // Serial.printf("이미지 취득 완료: %d\n", result);
        }

        esp_camera_fb_return(fb);
        fb = NULL;
    }

    return result;
}
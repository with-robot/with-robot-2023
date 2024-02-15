#include "esp_camera.h"
#include <iostream>
// #include <cstdio>
// #include <cstdint>
// #include <cstddef>
#include <memory>

struct Image_st
{
  uint8_t *buf;
  size_t size;

  void copy(uint8_t *img_cap_dt, size_t len)
  {
    size = len;
    buf = new uint8_t[len];
    std::move(img_cap_dt, img_cap_dt + len, buf);
  };
  void destroy()
  {
    if (buf != nullptr)
      delete buf;
  }
};

esp_err_t setup_camera();
esp_err_t capture_image(Image_st &data);
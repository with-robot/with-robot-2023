#ifndef CAMERA_TOOL_H
#define CAMERA_TOOL_H

#include <cstdio>
#include <cstdint>
#include <cstddef>
#include <memory>

typedef struct
{
  std::unique_ptr<uint8_t[]> buf;
  // uint8_t *buf;
  size_t size;
} Image_st;

esp_err_t setUp_camera();
esp_err_t capture_image(Image_st &data);

#endif

#ifndef CAMERA_TOOL_H
#define CAMERA_TOOL_H

#include <cstdio>
#include <cstdint>
#include <cstddef>

typedef struct {
  uint8_t *buf;
  size_t size;
} Image_st;

esp_err_t setUpCamera();
Image_st* capture_image();

#endif

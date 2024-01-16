#ifndef CAMERA_TOOL_H
#define CAMERA_TOOL_H

#include <cstdio>
#include <cstdint>
#include <cstddef>

typedef struct {
  uint8_t *buf;
  size_t size;
} Image_st;

esp_err_t setUp_camera();
esp_err_t capture_image(Image_st *data);

#endif

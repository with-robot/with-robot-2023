#ifndef IMAGE_S_H
#define IMAGE_S_H

#include <cstdio>
#include <cstdint>
#include <cstddef>
typedef struct Image_s {
  uint8_t *buf;
  size_t size;
} Image_st;

#endif
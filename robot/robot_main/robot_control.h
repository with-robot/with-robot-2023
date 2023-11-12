#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

enum CommandCode
{
  __quality=0x01,
  __flash,
  __flashoff,
  __framesize,
  __speed=0x51,
  __nostop,
  __direction
};

typedef struct
{
    byte buf[256] = {
        0xFF,
        0,
    };
    CommandCode cmd = __direction;
    unsigned int val = 0;
} CmdReq_st;

esp_err_t cmd_handler(CommandCode cmd, unsigned int value);

#endif
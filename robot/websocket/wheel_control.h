// TB6612FNG H-Bridge Connections (both PWM inputs driven by GPIO 12)
#define MTR_PWM 12
#define LEFT_M0 15
#define LEFT_M1 13
#define RIGHT_M0 14
#define RIGHT_M1 2

enum CommandCode
{
  __quality = 0x01,
  __flash,
  __flashoff,
  __framesize,
  __speed = 0x51,
  __nostop = 0x00,
  __direction = 0x81,
  __health_chk
};

enum opstate
{
  FWD,
  REV,
  LFT,
  RGT,
  STP // stop
};

typedef struct
{
  byte buf[256] = {
      0xFF,
      0,
  };
  CommandCode cmd = __direction;
  unsigned int size = 1;
  unsigned char val = 0;
} CmdReq_st;

class WheelControl
{
public:
  WheelControl();
  // 모터 동작
  void operate(int op_cmd);

private:
  std::array<std::array<byte, 4>, 5> op_cntrl;
  std::array<String, 5> op_nm;
};

esp_err_t cmd_handler(CommandCode cmd, unsigned char value, int speed_delta);

void robot_setup();

// Placeholder for functions
void robot_stop();
void robot_fwd();
void robot_back();
void robot_left();
void robot_right();
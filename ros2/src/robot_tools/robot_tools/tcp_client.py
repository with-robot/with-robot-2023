from abc import abstractmethod
import asyncio
import itertools
import time
from typing import Union
from rclpy.node import Node
import struct
import threading
import socket
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sensor_msgs.msg as sensor

# Result
OK = "OK"
FAIL = ""

# command
CMD_CFG = 0x01
NOTI_CAM = 0x81

# error
REQUEST = 0x00
RES_OK = 0x01
UNK_CMD = 0xEE
H_CHECK = 0x02

bridge = CvBridge()

cmd_map = {
    "quality": 0x01,
    "flash": 0x02,
    "flashoff": 0x03,
    "framesize": 0x04,
    "speed": 0x51,
    "nostop": 0x52,
    "direction": 0x53,
    "noti_cam": 0x81,
}


class TCPClient:
    # client_socket = None

    # def __new__(cls, host, port):
    #     if not cls.client_socket:
    #         cls.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #         cls.client_socket.connect((host, port))
    #     return super().__new__(cls)

    def __init__(self, host, port):
        # init socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((host, port))
        # serial recive thread run
        # self.lock = threading.Lock()

    def read(self, length=1):
        # with self.lock:
        buf = bytearray()
        while len(buf) < length:
            data = self.client_socket.recv(length - len(buf))
            if not data:
                raise Exception("수신 실패..")
            buf.extend(data)

        return buf

    def write(self, payload):
        # with self.lock:
        self.client_socket.send(bytearray(payload))

    def disconnect(self):
        if self.client_socket:
            self.client_socket.close()
            # self.lock = None


class RobotProxy:
    host: str = "192.168.4.1"
    port: int = 10000
    # TCP client instance
    robot: any = None
    logger: any = None
    alive: bool = False

    def __new__(cls, logger: any = None):
        if not cls.logger:
            cls.logger = logger
        # 로봇연결
        cls._connect(logger)
        return super().__new__(cls)

    @classmethod
    def _connect(cls, logger):
        logger.info(f"로봇연결을 시도합니다.....{cls.robot}")
        if RobotProxy.alive and cls.robot:
            return

        retry = 1
        while True:
            try:
                cls.robot = TCPClient(cls.host, cls.port)
                cls.alive = True
                break
            except Exception as e:
                time.sleep(2)
                logger.info(f"로봇연결을 다시 시도합니다.[#{retry}]")
                retry += 1

        logger.info(f"로봇이 연결되었습니다....{cls.robot}")

    def __init__(self, logger: any = None):
        # state info
        self.cfg = {"noti_imu": 0}
        self.led = 0
        self.cam = None
        self.logger = logger
        # unique seq counter
        self.seq = itertools.count()
        self._on_sending = False
        self._on_receiving = False

    def disconnect(self):
        self.robot.disconnect()

    def is_alive(self):
        try:
            payload = [0xFF, 0, 0, H_CHECK, 0, 0, 0, 0, 0x01]
            self.robot.write(payload)
            # self.logger.info("health check OK")
            RobotProxy.alive = True
        except Exception:
            RobotProxy.alive = False

        return RobotProxy.alive

    def send_msg(self, cmd: int, message: int) -> str:
        self.logger.info(
            f"send_msg: cmd={cmd}, map_cmd={cmd_map.get(cmd)}, data={message}"
        )

        if not cmd in cmd_map:
            raise Exception("사용할 수 없는 명령코드 입력[{cmd}]")
        try:
            self._send_data(cmd_map.get(cmd), [message])

            rx_buf: bytearray = self._read_data(interesting=CMD_CFG)
            if not rx_buf or rx_buf[3] != 1:
                raise Exception()

        except Exception as e:
            self.logger.error(f"로봇으로부터 메시지 수신 실패")
            return FAIL

        self.logger.info(f"recv_msg:: {','.join([format(x,'02X') for x in rx_buf])}")
        return OK

    def capture_image(self) -> Union[sensor.Image, None]:
        self.logger.info(f"화면이미지를 요청합니다.")

        try:
            self._send_data(cmd_map.get("noti_cam"), [00])

            rx_buf = self._read_data(interesting=NOTI_CAM)
            if not rx_buf or rx_buf[3] != 1:
                raise Exception()

        except Exception as e:
            return FAIL

        self.logger.info(f"받은 데이터를 변환한다.")
        image = cv2.imdecode(
            np.frombuffer(rx_buf[8:-1], dtype=np.uint8), cv2.IMREAD_COLOR
        )

        imgmsg = bridge.cv2_to_imgmsg(image, encoding="bgr8")

        return imgmsg

        # 간격조절
        # time.sleep(3)

        # logging.debug(f"처리 완료....")
        # cv2.imshow("robot vision", image)
        # cv2.waitKey(0)  # 0은 무한 대기, 양의 정수는 해당 시간(밀리초) 동안 대기
        # cv2.destroyAllWindows()

        # task = threading.Thread(name="image_receive_task", target=task)
        # task.start()

    # def set_cfg(self, noti_imu):
    #     self._send_data(CMD_CFG, [1 if noti_imu else 0])

    def _is_NOTI_CAM(self, buf: bytearray) -> bool:
        return buf[1] == NOTI_CAM

    def _read_data(self, interesting) -> bytearray:
        while self._on_receiving:
            time.sleep(0.1)
        self._on_receiving = True

        try:
            rx_buf = bytearray()
            rx_buf.extend(self.robot.read(8))
            self.logger.info(f"_read_data={' '.join(format(x, '02x') for x in rx_buf)}")

            # 시작점 체크
            if rx_buf[0] != 0xFF or rx_buf[1] != interesting:
                return

            rx_len = int.from_bytes(rx_buf[4:8], byteorder="little")
            # if rx_len == 0:
            #     return rx_buf

            rx_buf.extend(self.robot.read(rx_len + 1))

            # self.logger.info(f"_read_data2={' '.join(format(x, '02x') for x in rx_buf)}")

        except Exception as e:
            raise Exception("수신 중 오류가 발생하였습니다")

        finally:
            self._on_receiving = False

        # checksum 체크
        if sum(rx_buf[:-1]) & 0xFF != rx_buf[-1]:
            return

        return rx_buf[:-1]

    def _send_data(self, command, data):
        while self._on_sending:
            time.sleep(0.1)

        self._on_sending = True
        try:
            seq = next(self.seq) % 0xFE + 1
            payload = [0xFF, command, seq, REQUEST]
            payload.extend(len(data).to_bytes(4, byteorder="little"))
            payload.extend(data)
            checksum = sum(payload) & 0xFF
            payload.append(checksum)
            self.logger.info(f"_send_data: {payload}")
            self.robot.write(payload)

        except Exception as e:
            self.logger.error(f"_send_data={command}/{data}")
            raise Exception(f"송신 중 오류[{e}]")

        finally:
            self._on_sending = False

        # self.logger.info(f"_write_data={' '.join(format(x, '02x') for x in payload)}")

from abc import abstractmethod
import itertools
from time import sleep
from rclpy.node import Node
import struct
import threading
import socket
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from .interface import ImageSubscribeInf

# command
CMD_CFG = 0x01
NOTI_CAM = 0x81

# error
REQUEST = 0x00
RES_OK = 0x01
UNK_CMD = 0xEE

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
    def __init__(self, host, port):
        # init socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((host, port))
        # serial recive thread run

    def read(self, length=1):
        buf = bytearray()
        while len(buf) < length:
            data = self.client_socket.recv(length - len(buf))
            buf.extend(data)
        assert len(buf) == length, f"{len(buf)} != {length}"
        return buf

    def write(self, cmd):
        return self.client_socket.send(bytearray(cmd))


class RobotProxy:
    host: str = "192.168.4.1"
    port: int = 10000
    # TCP client instance
    robot: any = None

    def __init__(self, logger: any):
        # state info
        self.cfg = {"noti_imu": 0}
        self.led = 0
        self.cam = None
        self.logger = logger
        # unique seq counter
        self.seq = itertools.count()
        self.subscribers = set()

        # 로봇연결
        self._connect()

    def subscribe(self, subscriber: ImageSubscribeInf):
        self.subscribers.add(subscriber)

    def desubscribe(self, subscirber: ImageSubscribeInf):
        if subscirber in self.subscribers:
            self.subscribers.remove(subscirber)

    def send_msg(self, cmd: int, message: int) -> str:
        if not cmd in cmd_map:
            raise Exception("사용할 수 없는 명령코드 입력[{cmd}]")

        self.logger.info(f"cmd={cmd}, map_cmd={cmd_map.get(cmd)}, data={message}")

        response = "OK"
        retry = 0
        buf: bytearray = []
        while retry < 10:
            self._send_data(cmd_map.get(cmd), [message])

            if self._read_data(interesting=CMD_CFG):
                break

            sleep(0.1)
            # if retry > 5:

            #     response = "FAIL"
            #     break
            self.logger.info(f"fail retry #{retry}")
            retry += 1

        self.logger.info(
            f"count:{len(buf)}, send_msg return={','.join([format(x,'02X') for x in buf])}"
        )

        return response

    def _connect(self):
        self.logger.info(f"로봇연결을 시도합니다.....")
        if self.robot:
            raise Exception("로봇이 존재해요")
            return

        retry = 1
        while True:
            try:
                self.robot = TCPClient(self.host, self.port)
                break
            except Exception as e:
                sleep(2)
                self.logger.info(f"로봇연결을 다시 시도합니다.[{retry}]")
                retry += 1

    def req_capture(self):
        def task():
            while True:
                for publisher in self.subscribers:
                    self._send_data(cmd_map.get("noti_cam"), [00])

                    rx_buf = self._read_data(interesting=NOTI_CAM)
                    if rx_buf:
                        image = cv2.imdecode(
                            np.frombuffer(rx_buf[8:], dtype=np.uint8), cv2.IMREAD_COLOR
                        )

                        stream_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")

                        publisher.update(stream_image)

                # 간격조절
                sleep(5)

                # logging.debug(f"처리 완료....")
                # cv2.imshow("robot vision", image)
                # cv2.waitKey(0)  # 0은 무한 대기, 양의 정수는 해당 시간(밀리초) 동안 대기
                # cv2.destroyAllWindows()

        task = threading.Thread(name="image_receive_task", target=task)
        task.start()

    def set_cfg(self, noti_imu):
        self._send_data(CMD_CFG, [1 if noti_imu else 0])

    def _is_NOTI_CAM(self, buf: bytearray) -> bool:
        return buf[1] == NOTI_CAM

    def _read_data(self, interesting) -> bytearray:
        rx_buf = bytearray()
        rx_buf.extend(self.robot.read(8))
        self.logger.info(f"_read_data={' '.join(format(x, '02x') for x in rx_buf)}")

        # 시작점 체크
        if rx_buf[0] != 0xFF or rx_buf[1] != interesting:
            return

        rx_len = int.from_bytes(rx_buf[4:8], byteorder="little")
        rx_buf.extend(self.robot.read(rx_len + 1))

        # self.logger.info(f"_read_data2={' '.join(format(x, '02x') for x in rx_buf)}")

        # checksum 체크
        if sum(rx_buf[:-1]) & 0xFF != rx_buf[-1]:
            return

        return rx_buf[:-1]

    def _send_data(self, command, data):
        seq = next(self.seq) % 0xFE + 1
        cmd = [0xFF, command, seq, REQUEST]
        cmd.extend(len(data).to_bytes(4, byteorder="little"))
        cmd.extend(data)
        checksum = sum(cmd) & 0xFF
        cmd.append(checksum)

        self.robot.write(cmd)

        self.logger.info(f"_write_data={' '.join(format(x, '02x') for x in cmd)}")

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

        # 센싱처리 개시
        self._image_retrieving()

    def subscribe(self, subscriber: ImageSubscribeInf):
        self.subscribers.add(subscriber)

    def desubscribe(self, subscirber: ImageSubscribeInf):
        if subscirber in self.subscribers:
            self.subscribers.remove(subscirber)

    def send_msg(self, cmd: str, data: any = None) -> str:
        self.logger.info(f"cmd={cmd}")
        self.set_cfg(cmd)
        buf: bytearray = self._read_data()
        for byte in buf:
            self.logger.info(f"result={byte:02X}")
        return buf

    def _connect(self):
        try:
            self.robot = TCPClient(self.host, self.port)

        except Exception as e:
            raise Exception("로봇연결에 실패했습니다", str(e))

    def _image_retrieving(self):
        def task():
            while True:
                for publisher in self.subscribers:
                    rx_buf = self._read_data()
                    if not rx_buf:
                        sleep(1000)
                        break

                    image = cv2.imdecode(
                        np.frombuffer(rx_buf, dtype=np.uint8), cv2.IMREAD_COLOR
                    )

                    stream_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")

                    publisher.update(stream_image)

                # logging.debug(f"처리 완료....")
                # cv2.imshow("robot vision", image)
                # cv2.waitKey(0)  # 0은 무한 대기, 양의 정수는 해당 시간(밀리초) 동안 대기
                # cv2.destroyAllWindows()

        task = threading.Thread(name="image_receive_task", target=task)
        task.start()

    def set_cfg(self, noti_imu):
        self._send_data(CMD_CFG, [1 if noti_imu else 0])

    def _read_data(self) -> bytearray:
        rx_buf = bytearray()
        rx_buf.extend(self.robot.read(8))
        # 시작점 체크
        if rx_buf[0] != 0xFF:
            return

        rx_len = int.from_bytes(rx_buf[4:8], byteorder="little")
        rx_buf.extend(self.robot.read(rx_len + 1))

        # checksum 체크
        if sum(rx_buf[:-1]) & 0xFF != rx_buf[-1]:
            return

        if rx_buf[1] != NOTI_CAM:
            return

        return rx_buf[8:-1]

    def _send_data(self, command, data):
        seq = next(self.seq) % 0xFE + 1
        cmd = [0xFF, command, seq, REQUEST]
        cmd.extend(len(data).to_bytes(4, byteorder="little"))
        cmd.extend(data)
        checksum = sum(cmd) & 0xFF
        cmd.append(checksum)

        self.robot.write(cmd)

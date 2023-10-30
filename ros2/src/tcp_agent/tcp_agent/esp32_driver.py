from abc import abstractmethod
import itertools
import logging
import struct
import threading
import socket
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# command
CMD_CFG = 0x01
NOTI_CAM = 0x81

# error
REQUEST = 0x00
RES_OK = 0x01
UNK_CMD = 0xEE

bridge = CvBridge()


class TCPServer:
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


class IOHandler(TCPServer):
    def __init__(self, host="192.168.4.1", port=10000):
        super().__init__(host, port)
        # state info
        self.cfg = {"noti_imu": 0}
        self.led = 0
        self.cam = None
        # unique seq counter
        self.seq = itertools.count()

    def callback(self, cb):
        self.callback = cb

    def connect(self):
        task = threading.Thread(name="serial_recv_task", target=self.task)
        task.start()

    def task(self):
        while True:
            rx_buf = self.__read_data()
            if not rx_buf:
                continue

            image = cv2.imdecode(
                np.frombuffer(rx_buf, dtype=np.uint8), cv2.IMREAD_COLOR
            )

            stream_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")
            if hasattr(self, "callback"):
                self.callback(stream_image)

            logging.debug(f"수시처리 완료....")
            # cv2.imshow("robot vision", image)
            # cv2.waitKey(0)  # 0은 무한 대기, 양의 정수는 해당 시간(밀리초) 동안 대기
            # cv2.destroyAllWindows()

    def set_cfg(self, noti_imu):
        self.__send_data(CMD_CFG, [1 if noti_imu else 0])

    def __read_data(self) -> bytearray:
        rx_buf = bytearray()
        rx_buf.extend(self.read(8))
        # 시작점 체크
        if rx_buf[0] != 0xFF:
            return

        rx_len = int.from_bytes(rx_buf[4:8], byteorder="little")
        rx_buf.extend(self.read(rx_len + 1))

        # checksum 체크
        if sum(rx_buf[:-1]) & 0xFF != rx_buf[-1]:
            return

        if rx_buf[1] != NOTI_CAM:
            return

        logging.debug(f"__read_data: {rx_len}")
        return rx_buf[8:-1]

    def __send_data(self, command, data):
        seq = next(self.seq) % 0xFE + 1
        cmd = [0xFF, command, seq, REQUEST]
        cmd.extend(len(data).to_bytes(4, byteorder="little"))
        cmd.extend(data)
        checksum = sum(cmd) & 0xFF
        cmd.append(checksum)
        self.write(cmd)

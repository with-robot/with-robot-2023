# from abc import abstractmethod
import asyncio
import itertools
from typing import Union

# from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError

# import numpy as np
import sensor_msgs.msg as sensor, image_geometry
from time import sleep
import websockets

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


class WebSocketClient:
    def __init__(self, host, port):
        # init socket
        con_ = self.create_connection(uri=f"ws://{host}/ws")
        if not con_:
            raise Exception("connect fail")
        self.client_socket = con_

    def create_connection(self, uri) -> Union[object, None]:
        # init socket
        _client_socket = None
        while True:
            try:
                _client_socket = websockets.connect(uri=uri, open_timeout=20)
                if _client_socket:
                    break

            except TimeoutError as e:
                """"""
                break
        return _client_socket

    # websocket에 메시지를 보낸다.
    async def send_data(self, data) -> None:
        send_cnt = 0
        while send_cnt < 3:
            try:
                await self.client_socket.send(data)
                break
            except Exception as e:
                send_cnt += 1
                await asyncio.sleep(1)

        if send_cnt >= 3:
            raise TimeoutError("3회 시도 초과했습니다.")

        await asyncio.sleep(0)

    # 사진 이미지를 수신받는다.
    async def read_data(self) -> object:
        async with self.client_socket as websocket:
            message = await websocket.recv()

        return message

    def disconnect(self):
        if self.client_socket:
            # self.client_socket.disconnect()
            """"""


# url = "ws://192.168.10.102/ws"
class RobotProxy:
    host: str = "192.168.10.102"
    port: int = 80
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

        cls.robot = WebSocketClient(cls.host, cls.port)
        cls.alive = True

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
            self.robot.write("toggle")
            self.logger.info("health check OK")
            RobotProxy.alive = True
        except Exception:
            RobotProxy.alive = False

        return RobotProxy.alive

    async def send_msg(self, cmd: int, message: int) -> str:
        self.logger.info(
            f"send_msg: cmd={cmd}, map_cmd={cmd_map.get(cmd)}, data={message}"
        )

        if not cmd in cmd_map:
            raise Exception("사용할 수 없는 명령코드 입력[{cmd}]")
        try:
            await self._send_data(cmd_map.get(cmd), [message])

        except Exception as e:
            self.logger.error(f"로봇으로부터 메시지 수신 실패")
            return FAIL

        # self.logger.info(f"recv_msg:: {','.join([format(x,'02X') for x in rx_buf])}")
        return OK

    async def capture_image(self) -> Union[sensor.Image, None]:
        import numpy as np

        self.logger.info(f"화면이미지를 요청합니다.")

        try:
            rx_buf = await self.robot.read_data()
            # rx_buf가 bytes 형식인 경우
            image_array = np.frombuffer(rx_buf, dtype=np.uint8)

        except Exception as e:
            self.logger.info(f"FAIL")
            return FAIL

        self.logger.info(f"받은 데이터를 변환한다.")
        image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        # cv2.imwrite("test.jpg", image)
        # cv2.imshow("image", image)
        # cv2.waitKey(1)
        imgmsg = bridge.cv2_to_imgmsg(image, encoding="bgr8")

        return imgmsg

    def _is_NOTI_CAM(self, buf: bytearray) -> bool:
        return buf[1] == NOTI_CAM

    async def _send_data(self, command, data):
        self.logger.info(f"cmd={command},data={data}")
        await self.robot.send_data(data)

        # self.logger.info(f"_write_data={' '.join(format(x, '02x') for x in payload)}")

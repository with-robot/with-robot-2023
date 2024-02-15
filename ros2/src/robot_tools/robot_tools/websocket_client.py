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
import nest_asyncio

nest_asyncio.apply()
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
    "health_chk": 0x82,
}


class WebSocketClient:
    def __init__(self, host, port):
        # init socket
        self.uri = f"ws://{host}:{port}/ws"
        con_ = self.create_connection(uri=self.uri)
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

    # websocket에 메시지를 보낸다. 그 성공여부를 보낸다.
    # WebSocketClientProtocol
    async def send_data(self, data) -> str:
        try:
            async with self.client_socket as websocket:
                await websocket.send(data)
        except Exception as e:
            """"""
            return FAIL
        return OK

    # 사진 이미지를 수신받는다.
    async def read_data(self) -> object:
        count: int = 0
        message: str = None
        while count < 3:
            try:
                async with self.client_socket as websocket:
                    message = await websocket.recv()
                break
            except Exception as e:
                try:
                    self.client_socket = self.create_connection(uri=self.uri)
                    count += 1
                except Exception as e:
                    """"""
                sleep(1)

        return message

    def disconnect(self):
        if self.client_socket:
            self.client_socket.close()


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

        cls.robot: WebSocketClient = WebSocketClient(cls.host, cls.port)
        cls.alive = True

        logger.info(f"로봇이 연결되었습니다....{cls.robot}")

    def __init__(self, logger: any = None):
        # state info
        self.cam = None
        self.logger = logger

    def disconnect(self):
        self.robot.disconnect()

    def is_alive(self):
        result = asyncio.run(self.send_msg("health_chk", "chk"))
        RobotProxy.alive = result == OK

        return RobotProxy.alive

    async def send_msg(self, cmd: str, message: str) -> str:
        self.logger.info(
            f"send_msg: cmd={cmd}, map_cmd={cmd_map.get(cmd)}, data={message}"
        )

        if not cmd in cmd_map:
            raise Exception("사용할 수 없는 명령코드 입력[{cmd}]")
        try:
            send_msg = f"{cmd},{message}"
            await self.robot.send_data(send_msg)

        except Exception as e:
            self.logger.error(f"로봇으로부터 메시지 수신 실패")
            return FAIL

        # self.logger.info(f"recv_msg:: {','.join([format(x,'02X') for x in rx_buf])}")
        return OK

    async def capture_image(self) -> Union[sensor.Image, None]:
        import numpy as np

        # self.logger.info(f"화면이미지를 요청합니다.")

        try:
            rx_buf = await self.robot.read_data()
            # rx_buf가 bytes 형식인 경우
            image_array = np.frombuffer(rx_buf, dtype=np.uint8)

        except Exception as e:
            self.logger.info(f"{str(e)}")
            return FAIL

        # self.logger.info(f"받은 데이터를 변환한다.")
        image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        imgmsg = bridge.cv2_to_imgmsg(image, encoding="bgr8")

        return imgmsg

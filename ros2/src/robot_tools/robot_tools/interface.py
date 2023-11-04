from typing import Protocol
from sensor_msgs.msg import Image


class ImageSubscribeInf(Protocol):
    def update(self, data: Image) -> None:
        """사진을 수신한다."""

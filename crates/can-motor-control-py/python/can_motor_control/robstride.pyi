from enum import IntEnum

class MotorType(IntEnum):
    RS00 = 0
    RS06 = 6

class RobstrideCodec:
    def __init__(self, host_id: int = 0xFD) -> None: ...

from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    SEND_THREE_FLOATS = 2
    ECHO = 3
    DANCE = 4
    SET_VEL = 5
    GET_TIME_MILLIS = 6
    GET_TIME_MILLIS_LOOP = 7
    STORE_TIME_DATA = 8
    SEND_TIME_DATA = 9
    GET_TEMP_READINGS = 10
    GET_IMU_DATA = 11
    GET_RANGE = 12
    SET_PWM = 13
    GO_WALL = 14
    GET_WALL_DATA = 15
    SET_PID_MAX = 16
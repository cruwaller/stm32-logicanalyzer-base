import time
import serial
import struct
from threading import Thread

# F722:
# port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066CFF515455777867162951-if02"
# L432:
# port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066AFF485251667187111950-if02"
# L476RG:
port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066DFF555185754867171621-if02"

baud = 1000000
baud = 115200

ser = serial.Serial(port=port, baudrate=baud, timeout=2.)
# ser.timeout = 2.
ser.isOpen()
ser.flushInput()
ser.flushOutput()
while ser.inWaiting():
    ser.read(1)

HEADER_BYTE = b'^' # 0x5E, 94
TERMINATOR_BYTE1 = b'\r'
TERMINATOR_BYTE2 = b'\n'
TERMINATORS = TERMINATOR_BYTE1 + TERMINATOR_BYTE2

COMMAND_RUN = ord('r')
COMMAND_HALT = ord('h')
COMMAND_GET  = ord('g')
COMMAND_ACK = ord('a')
COMMAND_NACK = ord('n')
COMMAND_DATA = ord('d')

DEBUG = False

if DEBUG:
    def read_all():
        buf = bytearray()
        while 1:
            d = ser.read(1)
            buf += d
            if d == b'\n':
                print(buf)
                buf = bytearray()

    th = Thread(target=read_all)
    th.daemon = True
    th.start()


def read_resp():
    res = ser.read_until(TERMINATORS)
    #print(res)
    return list(res)


def send_cmd(cmd):
    _cmd = HEADER_BYTE + bytearray([3, cmd]) + TERMINATORS
    ser.write(_cmd)
    ser.flush()


def start():
    print("> start")
    for _ in range(3):
        send_cmd(COMMAND_RUN)
        if DEBUG:
            return
        resp = read_resp()
        if not resp:
            raise SystemError("No response to start command!")
        print("  < ", resp)
        if resp[2] == COMMAND_ACK:
            return
    print(" !!! nack to start")
    raise SystemExit()


def stop():
    print("> stop")
    send_cmd(COMMAND_HALT)
    if DEBUG:
        time.sleep(15)
        return
    fetch = False
    for _ in range(10):
        resp = read_resp()
        if not resp:
            print("!")
            continue
        if resp[2] == COMMAND_DATA:
            print("--- Data!")
            events = []
            count = struct.unpack("<I", bytes(resp[4:8]))[0]
            print("  count:", count)
            del resp[0:8]
            while count:
                _us = struct.unpack("<I", bytes(resp[0:4]))[0]
                _pins = struct.unpack("<I", bytes(resp[4:8]))[0]
                del resp[0:8]
                # events.append([_us] + [bool(_pins & _m) ^ invert for _m in masks])
                events.append((_us, bin(_pins)))
                count -= 1
            print("   >>>", events)
        elif resp[2] == COMMAND_ACK:
            print("--- ack!")
            if not fetch:
                print(" ...get...")
                send_cmd(COMMAND_GET)
                fetch = True
                continue
            print("--- done!")
            break
        #else:
        #    print("  < ", resp)


# send bad command to issue reboot
send_cmd(0)
time.sleep(.5)

start()
print("... wait 3sec ...")
time.sleep(3.)
stop()

ser.close()

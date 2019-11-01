from __future__ import print_function

import glob
import sys
import time

import serial

IO_SEVEN_SEG = 0x1000000

VERBOSE = True

# hexes = []
# with open(sys.argv[1]) as f:
#     for line in f:
#         if line.startswith(':'):
#             hexes.append(line.strip())


if sys.platform == 'darwin':
    # mac_irl
    serial_ifs = glob.glob('/dev/cu.usbserial*')
else:
    serial_ifs = glob.glob('/dev/ttyUSB0')

if not serial_ifs:
    print('No serial interfaces found!')
    exit(1)

print('Using serial:', serial_ifs[0])


def vprint(*args, **kwargs):
    if VERBOSE:
        print(*args, **kwargs)


def write_ack(ser, address, data, verify=False):
    payload = 'A' + hex(address)[2:] + 'W' + hex(data)[2:] + '\n'

    ser.write(payload.encode('ascii'))
    vprint(payload)
    time.sleep(0.01)

    while True:
        response = ser.readline().decode('ascii')
        vprint(response)
        if 'K' in response:
            break
        if response == '':
            raise ValueError('didnt ACK')

    # time.sleep(0.1)

    if verify:
        read_val = read_ack(ser, address)
        # print(read_val, data)
        assert read_val == data


def parsehex(num):
    return int('0x' + num, 16)


def read_ack(ser, address):
    payload = 'A' + hex(address)[2:] + 'R' + '\n'

    ser.write(payload.encode('ascii'))
    vprint(payload)
    time.sleep(0.01)

    while True:
        response = ser.readline().decode('ascii')
        vprint(response)
        if 'R' in response:
            break
        if response == '':
            raise ValueError('didnt get ACK')

    ack_address, _, mem = response.strip().partition('R')
    assert len(ack_address) == 9  # R + addr
    assert len(mem) == 8

    assert address == parsehex(ack_address[1:])

    # time.sleep(0.1)

    return parsehex(mem)


def dump_file(ser, filename, base=0, fixes=None, retry=5):
    prog = open(filename, 'rb').read()
    fixes = fixes or {}

    addr = 0
    for i in range(0, len(prog), 4):
        print('Writing File {}/{}'.format(i // 4, len(prog) // 4), end='\r')
        num = prog[i] << 24 | prog[i + 1] << 16 | prog[i + 2] << 8 | prog[i + 3]
        vprint(hex(num))
        if num in fixes:
            vprint('fixing:', hex(num), '->', hex(fixes[num]))
            num = fixes[num]

        for _ in range(retry):
            try:
                write_ack(ser, base + addr * 4, num, verify=True)
                break
            except AssertionError:
                print('Write error, retrying')
        else:
            raise AssertionError('gave up')

        print(hex(read_ack(ser, base + addr * 4)))
        addr += 1

    for i in range(32):
        print('Writing Empty buf {}/{}         '.format(i, 32), end='\r')
        for _ in range(retry):
            try:
                write_ack(ser, base + addr * 4, 0, verify=True)
                break
            except AssertionError:
                print('Write error, retrying')
        else:
            raise AssertionError('gave up')
        addr += 1


def dump_raw(ser, arr, base=0):
    for i in range(0, len(arr)):
        write_ack(ser, base + i * 4, arr[i], verify=True)


with serial.Serial(serial_ifs[0], 115200, timeout=0.1) as ser:
    print('Serial Name:', ser.name)

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    while True:
        d = ser.read(size=1000000)
        if not d:
            break
        print(d)

    # arr = [
    #     0x0D100000,
    #     0x0D200001,
    #     0x0D30000A,
    #     0x0E500100,
    #     0x65400034,
    #     0x08112000,
    #     0x08420000,
    #     0x08210000,
    #     0x08140000,
    #     0xA9250000,
    #     0x0D33FFFF,
    #     0x65F00010,
    #     0x08402000,
    #     0x65F00034,
    # ]

    dump_file(ser, '/Users/will/Work/transfer-learning/llvm-tl45/llvm/bbb/a.out')

    # print('\n')
    #
    # write_ack(ser, 0, 0xdeadbeef)
    #
    # print(hex(read_ack(ser, 0)))
    #
    # write_ack(ser, 512, 0xb0ba)
    #
    # print(hex(read_ack(ser, 0)))


    # for i in range(4, 800, 4):
    #     write_ack(ser, i, 0xb0ba)
    #     print(i, hex(read_ack(ser, 0)))

    # for i in range(1000):
    #     write_ack(ser, 0x1000000, i)
    #     time.sleep(0.5)

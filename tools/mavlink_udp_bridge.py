#!/usr/bin/env python3
"""Bidirectional raw MAVLink bridge between a serial device and UDP."""

import argparse
import select
import socket
import time

import serial


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--target", required=True)
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--listen-port", type=int, default=14550)
    return parser.parse_args()


def bridge(args):
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.bind(("0.0.0.0", args.listen_port))
    target = (args.target, args.port)

    while True:
        try:
            with serial.Serial(args.serial, args.baud, timeout=0) as uart:
                saw_serial = False
                saw_udp = False
                print(
                    f"Bridging {args.serial} at {args.baud} baud "
                    f"to udp://{args.target}:{args.port}",
                    flush=True,
                )
                while True:
                    readable, _, _ = select.select([uart, udp], [], [], 1.0)
                    if uart in readable:
                        payload = uart.read(uart.in_waiting or 1)
                        if payload:
                            udp.sendto(payload, target)
                            if not saw_serial:
                                print("MAVLink data received from Pixhawk", flush=True)
                                saw_serial = True
                    if udp in readable:
                        payload, _ = udp.recvfrom(65535)
                        if payload:
                            uart.write(payload)
                            if not saw_udp:
                                print("UDP response received from QGroundControl", flush=True)
                                saw_udp = True
        except (OSError, serial.SerialException) as error:
            print(f"Bridge unavailable: {error}; retrying in 2 seconds", flush=True)
            time.sleep(2)


if __name__ == "__main__":
    bridge(parse_args())

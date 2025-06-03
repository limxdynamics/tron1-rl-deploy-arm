import can
import struct
import os
import time
import sys
import rich.progress
from typing import Optional

RECV_TIMEOUT = 0.01
MAX_TRIAL = 8
TX_FRAME_INTERVAL = 0.0001
TX_TIMEOUT = 1
MAX_BURN_TIME = 60


def red(text):
    return f"\033[0;31m{text}\033[0m"


def send_can_data(
    bus, data, tx_id, rx_id=None, retry=3, timeout=0.1, recv=True
) -> Optional[str]:
    if rx_id is None:
        rx_id = tx_id
    msg = can.Message(arbitration_id=tx_id, data=data, is_extended_id=False)
    for t in range(retry):
        bus.send(msg)
        if not recv:
            return None
        start = time.time()
        while time.time() - start < timeout:
            response_msg = bus.recv(RECV_TIMEOUT)
            if response_msg is not None and response_msg.arbitration_id == rx_id:
                return response_msg
    # print("Send failed.", file=sys.stderr)
    return None


def send_end_request_data(bus, can_id):
    end_request_data = struct.pack("B", 0x00)
    send_can_data(bus, can_id, end_request_data)


def send_version_request_data(bus, can_id):
    version_request_data = struct.pack("B", 0x02)
    send_can_data(bus, can_id, version_request_data)


def main(can_channel: str, device_id: int, firmware_path: str, device_type: str):

    if not os.path.exists(firmware_path):
        print(red(f"Firmware file {firmware_path} does not exist"), file=sys.stderr)
        return
    else:
        with open(firmware_path, "rb") as file:
            bin_data = file.read()

    if device_id not in range(9):
        print(
            red("Invalid device ID. Please input a number between 0 and 8"),
            file=sys.stderr,
        )
        return

    # Initialize the CAN bus, set the interface and baud rate according to the actual situation
    init_start_time = time.time()
    try:
        bus = can.interface.Bus(
            channel=can_channel, bustype="socketcan", bitrate=1000000
        )
    except OSError:
        print(red(f"Cannot find CAN interface {can_channel}."), file=sys.stderr)
        return

    # Detect device type
    if device_type == "":
        force = False
        recv_msg = send_can_data(
            bus, struct.pack("B", 0x07), device_id, 0x100 | device_id
        )
        if recv_msg is None:
            print(
                red("No device detected. Please retry and input device name."),
                file=sys.stderr,
            )
            return
        resp_idx1, resp_idx2, resp_status = struct.unpack(">BB4s", recv_msg.data)
        device_type = resp_status.decode("utf-8")

        if device_type == "arm-":
            device_type = "end-board"
        elif device_type not in ["arm-", "vesc"]:
            print(red(f"Invalid device type: {device_type}"), file=sys.stderr)
            return
    else:
        force = True
        if device_type not in ["end-board", "vesc"]:
            print(red(f"Invalid given device name: {device_type}"))
            return

    version = None
    ver_trial = 0
    while ver_trial < MAX_TRIAL:
        resp_msg = send_can_data(
            bus, struct.pack("B", 0x02), device_id, 0x100 | device_id
        )
        if resp_msg is None:
            print(red("Version request failed. Please retry."), file=sys.stderr)
            return
        resp_idx1, resp_idx2, resp_status = struct.unpack(">BB4s", resp_msg.data[:6])

        if resp_idx1 == 0x02 and resp_idx2 == 0x01:
            print(
                f"{device_type} detected (ver. {resp_status[1]}.{resp_status[2]}.{resp_status[3]})"
            )
            version = f"{resp_status[1]}.{resp_status[2]}.{resp_status[3]}"
            break
        ver_trial += 1
    if ver_trial == MAX_TRIAL:
        print(red("Version request failed. Please retry."), file=sys.stderr)
        return

    # Request data transmission
    req_trial = 0
    while req_trial < MAX_TRIAL:
        resp_msg = send_can_data(
            bus, struct.pack("B", 0x00), 0x600 | device_id, 0x780 | device_id, timeout=2
        )
        if resp_msg is not None and struct.unpack("B", resp_msg.data)[0] == 0x01:
            break
        req_trial += 1
    if req_trial == MAX_TRIAL:
        print(red("Data transmission request failed. Please retry."), file=sys.stderr)
        return

    can_id = 0x680 | device_id
    index1 = 0
    index2 = 0

    if device_type == "end-board":
        time.sleep(3)

    tx_retry = 0

    with rich.progress.Progress() as progress:
        task = progress.add_task("[cyan]Transmitting data...", total=len(bin_data) / 6)

        while index1 * 256 + index2 < len(bin_data) / 6:

            can_data = (
                struct.pack(">BB", index1 % 256, index2 % 256)
                + bin_data[
                    index1 * 256 * 6 + index2 * 6 : index1 * 256 * 6 + index2 * 6 + 6
                ]
            )

            if version >= "2.5.0" and device_type != "end-board":
                send_can_data(bus, can_data, can_id, recv=False)
                index2 += 1
                time.sleep(TX_FRAME_INTERVAL)
                if index2 == 256:
                    received = False
                    start_time = time.time()
                    while time.time() - start_time < TX_TIMEOUT:
                        resp_msg = bus.recv(RECV_TIMEOUT)
                        if (
                            resp_msg is not None
                            and resp_msg.arbitration_id == 0x780 | device_id
                            and len(resp_msg.data) == 8
                        ):
                            resp_idx1, resp_idx2, resp_status = struct.unpack(
                                ">BB6s", resp_msg.data
                            )
                            if resp_status[0] == 0x01:
                                received = True
                                break
                    index2 = 0
                    if received:
                        tx_retry = 0
                        index1 += 1
                        progress.update(task, advance=256)
                    else:
                        tx_retry += 1
                        if tx_retry <= MAX_TRIAL:
                            # print(red(
                            #     f"Packet {index1} failed. Retrying..."),
                            #         file=sys.stderr)
                            continue
                        else:
                            print(
                                red(
                                    f"Packet {index1} failed too many times. Aborting."
                                ),
                                file=sys.stderr,
                            )
                            return

            else:
                resp_msg = send_can_data(bus, can_data, can_id, 0x780 | device_id)
                received = False
                if resp_msg is not None:
                    resp_idx1, resp_idx2, resp_status = struct.unpack(
                        ">BB6s", resp_msg.data
                    )
                    if resp_status[0] == 0x01:
                        received = True

                if received:
                    tx_retry = 0
                    index2 += 1
                    if index2 == 256:
                        index2 = 0
                        index1 += 1
                    progress.update(task, advance=1)
                else:
                    tx_retry += 1
                    if tx_retry < MAX_TRIAL:
                        # print(red(
                        #     f"Packet {index1}-{index2} failed. Retrying..."
                        # ),
                        #         file=sys.stderr)
                        pass
                    else:
                        print(
                            red(
                                f"Packet {index1}-{index2} failed too many times. Aborting."
                            ),
                            file=sys.stderr,
                        )
                        return

    resp_msg = send_can_data(
        bus, struct.pack("B", 0x00), 0x700 | device_id, 0x780 | device_id, timeout=3
    )
    if resp_msg is not None and struct.unpack("B", resp_msg.data[:1])[0] == 0x02:
        print("End request successful. Data transmit finished.", file=sys.stderr)
        print(
            red(
                "Please wait for the program auto close and don't shutdown the power supply now."
            ),
            file=sys.stderr,
        )
    else:
        print(red("End request failed. Aborting."), file=sys.stderr)
        return

    burned_flag = False
    start_burn_time = time.time()
    with rich.progress.Progress() as burn_progress:
        burn_task = burn_progress.add_task("[green]Burning firmware...", total=100)
        progress_cnt = 0
        while time.time() - start_burn_time < MAX_BURN_TIME:
            time.sleep(0.1)
            progress_cnt = min(progress_cnt + 0.75, 99)
            burn_progress.update(burn_task, completed=progress_cnt)

            if time.time() - start_burn_time > 6:
                resp_msg = send_can_data(
                    bus, struct.pack("B", 0x02), device_id, 0x100 | device_id
                )
                if resp_msg is not None:
                    resp_idx1, resp_idx2, resp_status = struct.unpack(
                        ">BB4s", resp_msg.data[:6]
                    )
                    if resp_idx1 == 0x02 and resp_idx2 == 0x01:
                        burn_progress.update(burn_task, completed=100)
                        burned_flag = True
                        break

    if not burned_flag:
        print(red("Burn failed. Please retry."), file=sys.stderr)
        return

    print(f"Burn finished: ver: {resp_status[1]}.{resp_status[2]}.{resp_status[3]}")

    end_time = time.time()
    print(f"Time elapsed: {end_time - init_start_time} seconds")
    bus.shutdown()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="IAP burn tool")
    parser.add_argument("firmware_path", type=str, help="Firmware path")
    parser.add_argument("-i", "--can-id", type=int, help="", default=-1)
    parser.add_argument(
        "-m", "--can-interface", type=str, help="CAN interface", default="can0"
    )
    parser.add_argument("--slow", action="store_true", help="Slow burn mode")
    parser.add_argument(
        "-n",
        "--device-name",
        type=str,
        help="Device name: arm-interface-board-end or vesc-motor-control",
        default="",
    )
    args = parser.parse_args()

    if args.can_id == -1:
        args.can_id = int(input("Please input device can id: "))

    main(args.can_interface, args.can_id, args.firmware_path, args.device_name)

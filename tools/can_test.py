"""F407 Body ECU / Bootloader CAN test menu.

Usage (PowerShell, from the repository root):
    python .\tools\can_test.py

CANgaroo must be disconnected first because COM17 can normally be opened by
only one Windows process at a time.
"""

from __future__ import annotations

import argparse
import sys
import time
import zlib
from collections.abc import Callable
from pathlib import Path

import can


DEFAULT_PORT = "COM17"
DEFAULT_BITRATE = 500_000
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FIRMWARE_PATH = (
    PROJECT_ROOT
    / "F407_Body_ECU"
    / "build"
    / "Debug"
    / "F407_Body_ECU.bin"
)

APPLICATION_START_ADDRESS = 0x08010000
APPLICATION_END_ADDRESS = 0x08060000

REQUEST_ID = 0x7E0
RESPONSE_ID = 0x7E8
HEARTBEAT_ID = 0x123

RESPONSE_TIMEOUT_SECONDS = 2.0
REQUEST_DOWNLOAD_TIMEOUT_SECONDS = 15.0
HOST_MIN_FRAME_GAP_SECONDS = 0.001


def format_bytes(data: bytes | bytearray) -> str:
    return " ".join(f"{value:02X}" for value in data)


def show_firmware_info(firmware_path: Path) -> None:
    try:
        firmware_data = firmware_path.read_bytes()
    except OSError as error:
        print(f"读取固件失败：{error}", file=sys.stderr)
        return

    firmware_size = len(firmware_data)
    firmware_end_address = APPLICATION_START_ADDRESS + firmware_size
    firmware_crc32 = zlib.crc32(firmware_data) & 0xFFFFFFFF

    print("\n========== 应用固件信息 ==========")
    print(f"文件：{firmware_path}")
    print(f"大小：{firmware_size} 字节（0x{firmware_size:X}）")
    print(f"起始地址：0x{APPLICATION_START_ADDRESS:08X}")
    print(f"结束地址（不含）：0x{firmware_end_address:08X}")
    print(f"CRC-32：0x{firmware_crc32:08X}")

    if firmware_size == 0:
        print("错误：固件文件为空。")
    elif firmware_end_address > APPLICATION_END_ADDRESS:
        print("错误：固件超出应用程序Flash区域。")
    else:
        print("检查结果：固件位于应用程序Flash区域内。")


def send_can_frame(
    bus: can.BusABC,
    data: list[int],
    show_frame: bool = True,
) -> None:
    padded_data = data + [0x00] * (8 - len(data))
    frame = can.Message(
        arbitration_id=REQUEST_ID,
        data=padded_data,
        is_extended_id=False,
    )
    bus.send(frame, timeout=1.0)
    if show_frame:
        print(f"TX  0x{REQUEST_ID:03X}  {format_bytes(frame.data)}")


def wait_for_frame(
    bus: can.BusABC,
    predicate: Callable[[can.Message], bool],
    timeout_seconds: float,
    show_frame: bool = True,
    ignore_heartbeat: bool = True,
) -> can.Message | None:
    deadline = time.monotonic() + timeout_seconds

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return None

        frame = bus.recv(timeout=remaining)
        if frame is None:
            return None

        # 心跳帧与本次UDS问答无关，直接忽略，避免刷屏。
        if ignore_heartbeat and frame.arbitration_id == HEARTBEAT_ID:
            continue

        if show_frame:
            print(
                f"RX  0x{frame.arbitration_id:03X}  "
                f"{format_bytes(frame.data)}"
            )

        if predicate(frame):
            return frame


def wait_for_flow_control(
    bus: can.BusABC,
    show_frames: bool = True,
) -> tuple[int, int]:
    frame = wait_for_frame(
        bus,
        lambda item: (
            item.arbitration_id == RESPONSE_ID
            and len(item.data) >= 3
            and (item.data[0] >> 4) == 0x03
        ),
        RESPONSE_TIMEOUT_SECONDS,
        show_frames,
    )

    if frame is None:
        raise TimeoutError("等待ISO-TP流控帧超时")

    flow_status = frame.data[0] & 0x0F
    if flow_status != 0x00:
        raise RuntimeError(f"ECU没有允许继续发送，FlowStatus={flow_status}")

    return frame.data[1], frame.data[2]


def stmin_to_seconds(stmin: int) -> float:
    if 0x00 <= stmin <= 0x7F:
        return stmin / 1000.0
    if 0xF1 <= stmin <= 0xF9:
        return (stmin - 0xF0) / 10_000.0
    return 0.0


def send_isotp_request(
    bus: can.BusABC,
    payload: bytes,
    show_frames: bool = True,
) -> None:
    payload_length = len(payload)

    if payload_length == 0:
        raise ValueError("UDS请求不能为空")

    if payload_length <= 7:
        send_can_frame(bus, [payload_length, *payload], show_frames)
        return

    if payload_length > 0x0FFF:
        raise ValueError("当前测试工具只支持长度不超过4095字节的ISO-TP消息")

    first_frame = [
        0x10 | ((payload_length >> 8) & 0x0F),
        payload_length & 0xFF,
        *payload[:6],
    ]
    send_can_frame(bus, first_frame, show_frames)

    block_size, stmin = wait_for_flow_control(bus, show_frames)
    separation_seconds = max(
        stmin_to_seconds(stmin),
        HOST_MIN_FRAME_GAP_SECONDS,
    )

    offset = 6
    sequence_number = 1
    frames_in_block = 0

    while offset < payload_length:
        chunk = payload[offset : offset + 7]
        consecutive_frame = [0x20 | sequence_number, *chunk]
        send_can_frame(bus, consecutive_frame, show_frames)

        offset += len(chunk)
        sequence_number = (sequence_number + 1) & 0x0F
        frames_in_block += 1

        if separation_seconds > 0.0 and offset < payload_length:
            time.sleep(separation_seconds)

        if (
            block_size != 0
            and frames_in_block >= block_size
            and offset < payload_length
        ):
            block_size, stmin = wait_for_flow_control(bus, show_frames)
            separation_seconds = max(
                stmin_to_seconds(stmin),
                HOST_MIN_FRAME_GAP_SECONDS,
            )
            frames_in_block = 0


def wait_for_uds_response(
    bus: can.BusABC,
    timeout_seconds: float = RESPONSE_TIMEOUT_SECONDS,
    show_frames: bool = True,
    show_payload: bool = True,
    report_timeout: bool = True,
) -> bytes | None:
    frame = wait_for_frame(
        bus,
        lambda item: (
            item.arbitration_id == RESPONSE_ID
            and len(item.data) >= 1
            and (item.data[0] >> 4) != 0x03
        ),
        timeout_seconds,
        show_frames,
    )

    if frame is None:
        if report_timeout:
            print("等待0x7E8响应超时。")
        return None

    frame_type = frame.data[0] >> 4
    if frame_type != 0x00:
        print("目前测试工具只解析ECU发出的ISO-TP单帧响应。")
        return bytes(frame.data)

    payload_length = frame.data[0] & 0x0F
    if payload_length > len(frame.data) - 1:
        print("响应中的ISO-TP长度不合法。")
        return None

    payload = bytes(frame.data[1 : 1 + payload_length])
    if show_payload:
        print(f"UDS响应：{format_bytes(payload)}")

    if show_payload and len(payload) >= 3 and payload[0] == 0x7F:
        print(
            f"否定响应：服务ID=0x{payload[1]:02X}，"
            f"NRC=0x{payload[2]:02X}"
        )

    return payload


def send_named_request(
    bus: can.BusABC,
    name: str,
    payload: bytes,
) -> bytes | None:
    print(f"\n发送：{name}")
    print(f"UDS请求：{format_bytes(payload)}")
    send_isotp_request(bus, payload)

    # 0x34会触发应用区Flash擦除，执行时间明显长于普通UDS服务。
    if payload[0] == 0x34:
        return wait_for_uds_response(bus, REQUEST_DOWNLOAD_TIMEOUT_SECONDS)

    return wait_for_uds_response(bus)


def verify_application_started(bus: can.BusABC) -> bool:
    print("正在等待应用程序心跳0x123……")
    heartbeat = wait_for_frame(
        bus,
        lambda item: (
            item.arbitration_id == HEARTBEAT_ID
            and len(item.data) == 8
            and item.data[4] == 0xA5
            and item.data[5] == 0x5A
        ),
        timeout_seconds=3.0,
        show_frame=False,
        ignore_heartbeat=False,
    )

    if heartbeat is None:
        print("未检测到应用程序心跳，尚不能确认新应用已经启动。")
        return False

    heartbeat_sequence = int.from_bytes(
        heartbeat.data[0:4],
        byteorder="little",
    )
    lamp_state = heartbeat.data[6]
    print(
        "检测到应用程序心跳："
        f"ID=0x{HEARTBEAT_ID:03X}，"
        f"序号={heartbeat_sequence}，车灯状态={lamp_state}"
    )
    print("验证结果：Bootloader已经跳转并运行新应用程序。")
    return True


COMMANDS: dict[str, tuple[str, bytes]] = {
    "1": (
        "0x10 DiagnosticSessionControl（0x03）",
        bytes([0x10, 0x03]),
    ),
    "2": (
        "0x10 DiagnosticSessionControl（0x02）",
        bytes([0x10, 0x02]),
    ),
    "3": (
        "0x3E TesterPresent（0x00）",
        bytes([0x3E, 0x00]),
    ),
    "4": (
        "0x34 RequestDownload",
        bytes(
            [
                0x34,
                0x00,
                0x44,
                0x08,
                0x01,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x04,
            ]
        ),
    ),
    "5": (
        "0x36 TransferData",
        bytes([0x36, 0x01, 0x11, 0x22, 0x33, 0x44]),
    ),
    "6": (
        "0x37 RequestTransferExit（正确CRC）",
        bytes([0x37, 0x77, 0xF2, 0x9D, 0xD1]),
    ),
    "7": (
        "0x37 RequestTransferExit（错误CRC）",
        bytes([0x37, 0x77, 0xF2, 0x9D, 0xD0]),
    ),
    "8": (
        "0x11 ECUReset（0x01）",
        bytes([0x11, 0x01]),
    ),
}


def print_menu() -> None:
    print("\n========== F407 UDS测试菜单 ==========")
    for key, (name, _) in COMMANDS.items():
        print(f"{key}. {name}")
    print("9. 0x34 → 0x36 → 0x37 自动测试")
    print("f. 固件信息")
    print("u. BIN固件升级")
    print("v. 验证应用启动")
    print("q. 退出")


def run_four_byte_download_test(bus: can.BusABC) -> None:
    answer = input("该操作会擦除应用程序区域，确认继续？[y/N]：").strip().lower()
    if answer != "y":
        print("已取消。")
        return

    for command_key in ("4", "5", "6"):
        name, payload = COMMANDS[command_key]
        response = send_named_request(bus, name, payload)
        if response is None or (response and response[0] == 0x7F):
            print("自动测试中止。")
            return

    print("4字节下载测试完成。")


def run_firmware_download(
    bus: can.BusABC,
    firmware_path: Path,
) -> None:
    try:
        firmware_data = firmware_path.read_bytes()
    except OSError as error:
        print(f"读取固件失败：{error}", file=sys.stderr)
        return

    firmware_size = len(firmware_data)
    firmware_end_address = APPLICATION_START_ADDRESS + firmware_size
    firmware_crc32 = zlib.crc32(firmware_data) & 0xFFFFFFFF

    if firmware_size == 0:
        print("升级中止：固件文件为空。")
        return

    if firmware_end_address > APPLICATION_END_ADDRESS:
        print("升级中止：固件超出应用程序Flash区域。")
        return

    print("\n========== BIN固件升级 ==========")
    print(f"文件：{firmware_path}")
    print(f"大小：{firmware_size} 字节")
    print(f"地址：0x{APPLICATION_START_ADDRESS:08X}")
    print(f"CRC-32：0x{firmware_crc32:08X}")

    answer = input("该操作会擦除并重写应用程序区域，确认继续？[y/N]：").strip().lower()
    if answer != "y":
        print("已取消。")
        return

    print("\n准备阶段：确认ECU已经进入Bootloader")
    send_isotp_request(bus, bytes([0x10, 0x03]))
    session_response = wait_for_uds_response(
        bus,
        show_payload=False,
        report_timeout=False,
    )

    if session_response == bytes([0x50, 0x03]):
        print("ECU当前运行应用程序，正在请求进入编程会话……")
        send_isotp_request(bus, bytes([0x10, 0x02]))
        programming_response = wait_for_uds_response(bus)

        if programming_response != bytes([0x50, 0x02]):
            print("升级中止：应用程序没有接受编程会话请求。")
            return

        print("应用程序正在复位进入Bootloader……")
        time.sleep(1.0)
    elif session_response is None:
        print("未收到应用程序会话响应，继续检查Bootloader下载服务。")
    else:
        print(
            "未进入扩展会话，继续检查Bootloader下载服务。"
        )

    request_download = (
        bytes([0x34, 0x00, 0x44])
        + APPLICATION_START_ADDRESS.to_bytes(4, byteorder="big")
        + firmware_size.to_bytes(4, byteorder="big")
    )

    print("\n阶段1/4：0x34 RequestDownload")
    send_isotp_request(bus, request_download)
    response = wait_for_uds_response(
        bus,
        REQUEST_DOWNLOAD_TIMEOUT_SECONDS,
    )

    if response is None or response[0] != 0x74:
        print("升级中止：0x34没有得到肯定响应。")
        return

    if len(response) < 3:
        print("升级中止：0x74响应长度不足。")
        return

    length_format_identifier = response[1]
    block_length_byte_count = length_format_identifier >> 4

    if (
        block_length_byte_count == 0
        or (length_format_identifier & 0x0F) != 0
        or len(response) < 2 + block_length_byte_count
    ):
        print("升级中止：0x74中的长度格式不合法。")
        return

    max_block_length = int.from_bytes(
        response[2 : 2 + block_length_byte_count],
        byteorder="big",
    )

    if max_block_length <= 2 or max_block_length > 256:
        print(f"升级中止：ECU返回的最大块长度不受支持：{max_block_length}")
        return

    firmware_bytes_per_block = max_block_length - 2
    total_blocks = (
        firmware_size + firmware_bytes_per_block - 1
    ) // firmware_bytes_per_block

    print(
        f"\n阶段2/4：0x36 TransferData，"
        f"每块最多{firmware_bytes_per_block}字节，共{total_blocks}块"
    )

    transferred_size = 0
    block_sequence_counter = 1

    for block_index in range(total_blocks):
        block_end = min(
            transferred_size + firmware_bytes_per_block,
            firmware_size,
        )
        firmware_block = firmware_data[transferred_size:block_end]
        transfer_data = bytes(
            [0x36, block_sequence_counter]
        ) + firmware_block

        send_isotp_request(bus, transfer_data, show_frames=False)
        response = wait_for_uds_response(
            bus,
            show_frames=False,
            show_payload=False,
        )

        expected_response = bytes([0x76, block_sequence_counter])
        if response != expected_response:
            actual_response = (
                "无响应" if response is None else format_bytes(response)
            )
            print(
                f"\n升级中止：第{block_index + 1}块响应错误，"
                f"收到：{actual_response}"
            )
            return

        transferred_size = block_end
        progress_percent = transferred_size * 100.0 / firmware_size
        print(
            f"\r已传输：{transferred_size}/{firmware_size} 字节 "
            f"({progress_percent:5.1f}%)",
            end="",
            flush=True,
        )

        block_sequence_counter = (block_sequence_counter + 1) & 0xFF

    print()

    request_transfer_exit = bytes([0x37]) + firmware_crc32.to_bytes(
        4,
        byteorder="big",
    )

    print("\n阶段3/4：0x37 RequestTransferExit")
    send_isotp_request(bus, request_transfer_exit)
    response = wait_for_uds_response(bus)

    if response != bytes([0x77]):
        print("升级中止：固件CRC校验未通过。")
        return

    print("\n阶段4/4：0x11 ECUReset")
    send_isotp_request(bus, bytes([0x11, 0x01]))
    response = wait_for_uds_response(bus)

    if response != bytes([0x51, 0x01]):
        print("升级中止：ECU没有正确响应复位请求。")
        return

    print("\n固件升级成功，ECU正在复位并启动新应用程序。")
    verify_application_started(bus)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="F407 UDS命名CAN测试工具")
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"CANable串口，默认{DEFAULT_PORT}",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=DEFAULT_BITRATE,
        help=f"CAN总线波特率，默认{DEFAULT_BITRATE}",
    )
    parser.add_argument(
        "--firmware",
        type=Path,
        default=DEFAULT_FIRMWARE_PATH,
        help=f"应用程序BIN文件，默认{DEFAULT_FIRMWARE_PATH}",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()

    print(f"正在打开CANable：{args.port}，CAN波特率：{args.bitrate}")
    print("请确保CANgaroo已经停止并断开COM口。")

    try:
        with can.Bus(
            interface="slcan",
            channel=args.port,
            bitrate=args.bitrate,
            sleep_after_open=2.0,
        ) as bus:
            print(f"连接成功：{bus.channel_info}")

            while True:
                print_menu()
                choice = input("请选择：").strip().lower()

                if choice == "q":
                    return 0

                if choice == "9":
                    run_four_byte_download_test(bus)
                    continue

                if choice == "f":
                    show_firmware_info(args.firmware)
                    continue

                if choice == "u":
                    run_firmware_download(bus, args.firmware)
                    continue

                if choice == "v":
                    verify_application_started(bus)
                    continue

                command = COMMANDS.get(choice)
                if command is None:
                    print("无效选项。")
                    continue

                name, payload = command
                send_named_request(bus, name, payload)

    except KeyboardInterrupt:
        print("\n用户中止。")
        return 0
    except (can.CanError, OSError, TimeoutError, RuntimeError) as error:
        print(f"CAN通信失败：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

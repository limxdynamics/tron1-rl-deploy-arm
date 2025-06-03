import can
import time
import threading

# Constants
SEND_IDS = range(9)  # CAN IDs from 0 to 8
RECEIVE_IDS = [0x100, 0x101, 0x102, 0x103, 4, 5, 6, 0x107, 0x108]
NORMAL_DATA = b"\x00"  # Data for other IDs
BITRATE = 1000000  # Bitrate 1M
INTERFACE = "socketcan"  # Interface
CHANNEL = "can0"  # Channel (typically 'can0' for SocketCAN)
SEND_DELAY = 0.01  # Delay for sending messages (100 Hz)
MAX_SEND_COUNT = 1000  # Maximum number of times to send messages

# Configure the CAN network
bus = can.interface.Bus(channel=CHANNEL, bustype=INTERFACE, bitrate=BITRATE)


def send_messages():
    send_count = 0
    while send_count < MAX_SEND_COUNT:
        for can_id in SEND_IDS:
            data = (
                NORMAL_DATA
                if can_id not in (4, 5, 6)
                else b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD"
            )
            message = can.Message(
                arbitration_id=can_id, data=data, is_extended_id=False
            )
            bus.send(message)
            # print(f"Sent message to CAN ID {can_id}")
        send_count += 1
        time.sleep(SEND_DELAY)


def receive_messages(received_counts):
    start_time = time.time()
    while time.time() - start_time < 15:  # Check for messages for 15 seconds
        message = bus.recv(1.0)  # Timeout after 1 second if no message
        if message and message.arbitration_id in RECEIVE_IDS:
            received_counts[message.arbitration_id] += 1
            # print(f"Received message from CAN ID {hex(message.arbitration_id)}")


def main():
    received_counts = {
        id: 0 for id in RECEIVE_IDS
    }  # Dictionary to track received counts
    send_thread = threading.Thread(target=send_messages, daemon=True)
    send_thread.start()
    receive_messages(received_counts)
    print("Total messages received from each CAN ID:")
    for i in range(len(RECEIVE_IDS)):
        print(f"CAN ID {i}: {received_counts[RECEIVE_IDS[i]]/MAX_SEND_COUNT*100:.2f}%")


if __name__ == "__main__":
    main()

#!/usr/bin/python3
"""Local TCP relay used only when macOS blocks an app's local-network path.

Aura.app connects to loopback; this small unsigned helper owns the LAN socket
to Aura Main Board. The wire protocol is copied byte-for-byte in both
directions. It never interprets, creates, or changes a command.
"""

import select
import socket
import sys
import time

BOARD_HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.69"
BOARD_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 4242
LOCAL_PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 4243


def connect_board():
    while True:
        try:
            link = socket.create_connection((BOARD_HOST, BOARD_PORT), timeout=3)
            link.setblocking(False)
            return link
        except OSError:
            time.sleep(1)


def serve(client):
    client.setblocking(False)
    board = connect_board()
    try:
        while True:
            readable, _, _ = select.select((client, board), (), (), 1)
            for source in readable:
                try:
                    data = source.recv(4096)
                except BlockingIOError:
                    continue
                if not data:
                    return
                destination = board if source is client else client
                destination.sendall(data)
    finally:
        board.close()
        client.close()


def main():
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", LOCAL_PORT))
    listener.listen(1)
    while True:
        client, _ = listener.accept()
        serve(client)


if __name__ == "__main__":
    main()

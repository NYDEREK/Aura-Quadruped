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

import os

# argv: <host> <board port> <local port> [fallback address]
# <host> is normally the mDNS name aura-main-board.local, so a new router or
# DHCP lease never strands the app on an old hard-coded address. The last
# address that answered is cached and tried next; argv[4] is a last resort.
BOARD_HOST = sys.argv[1] if len(sys.argv) > 1 else "aura-main-board.local"
BOARD_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 4242
LOCAL_PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 4243
FALLBACK_HOST = sys.argv[4] if len(sys.argv) > 4 else None
CACHE = os.path.expanduser("~/Library/Application Support/Aura/board_address")


def cached_address():
    try:
        with open(CACHE) as handle:
            return handle.read().strip() or None
    except OSError:
        return None


def remember(address):
    try:
        os.makedirs(os.path.dirname(CACHE), exist_ok=True)
        with open(CACHE, "w") as handle:
            handle.write(address)
    except OSError:
        pass


def connect_board():
    while True:
        candidates = [BOARD_HOST, cached_address(), FALLBACK_HOST]
        for host in dict.fromkeys(h for h in candidates if h):
            try:
                link = socket.create_connection((host, BOARD_PORT), timeout=3)
                remember(link.getpeername()[0])
                link.setblocking(False)
                return link
            except OSError:
                continue
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

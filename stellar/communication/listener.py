import json
import socket
from threading import Thread
from typing import Any, Callable

from communication.common import get_socket_and_address


class MessageListener:
    handler: Callable[[dict], Any]
    listener_socket: socket
    is_running: bool = True

    def __init__(self, sender_id: str, callback: Callable[[dict], Any]):
        self.handler = callback

        self.listener_socket, address = get_socket_and_address(sender_id)
        self.listener_socket.bind(address)
        self.listener_socket.listen()

    @staticmethod
    def receive_all(connection: socket) -> str:
        all_data = []

        while True:
            received = connection.recv(8192)
            if received:
                all_data.append(received.decode('utf-8'))
            else:
                break

        return ''.join(all_data)

    def run(self):
        while self.is_running:
            connection, _ = self.listener_socket.accept()
            received_data = self.receive_all(connection)
            self.handler(json.loads(received_data))
            connection.close()

    def stop(self):
        self.is_running = False


def listen_to(sender_id: str, listener: Callable[[dict], Any]) -> MessageListener:
    """
    Registers a listener for messages sent by sender_id.
    """

    message_listener = MessageListener(sender_id, listener)
    Thread(target=message_listener.run).start()
    return message_listener

import json

from communication.common import get_socket_and_address


def send_message(sender_id: str, message: dict) -> bool:
    """
    Sends a message to whoever is listening.
    Returns False if nobody is listening, True otherwise.
    """

    socket, address = get_socket_and_address(sender_id)
    try:
        with socket as sender_socket:
            sender_socket.connect(address)
            sender_socket.sendall(json.dumps(message).encode('utf-8'))

    except:
        return False  # Nobody's listening right now.

    return True

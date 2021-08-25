import socket

hash_cache: dict = {}


def get_socket_and_address(target: str):
    # AF_UNIX is a lightweight method for interprocess communication,
    # but it is only available on UNIX systems.
    # For testing on Windows, we can use INET as alternative.
    address_family = getattr(socket, 'AF_UNIX', socket.AF_INET)
    address = get_address(address_family, target)

    return socket.socket(address_family, socket.SOCK_STREAM), address


def get_address(address_family, target: str):
    if address_family == socket.AF_INET:
        return get_windows_address(target)
    else:
        return get_unix_address(target)


def get_unix_address(target: str):
    return f'tmp/stellar/{target}'


def get_windows_address(target: str):
    port = 1024 + (get_hash(target) % 48128)
    return ('localhost', port)


def get_hash(string: str):
    # Implementing custom hashing algorithm, because python's adds a random salt that breaks communication on Windows
    global hash_cache

    if string in hash_cache:
        return hash_cache[string]

    hash = 0
    p = 1
    for char in string:
        hash += ord(char) * p
        p *= 13

    hash_cache[string] = hash
    return hash

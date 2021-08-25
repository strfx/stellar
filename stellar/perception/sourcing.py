"""
Handle inbound communication from the tinyK22.

This module takes in a binary stream and returns concrete values, as recorded
by the array of sensors connected to the tinyK22.

The tinyK22 sends bundled sensor data over an UART serial interface. These
messages are demodulated here into distinct values. A UART message consists
of a start bit, 8 bits of data and a stop bit. Multiple messages can be send
indipendently until the stop bit is set.

Multiple steps are performed read and parse UART messages:
    1. Read raw binary data from the device
    2. Reassemble original payload from fragmented messages
    3. Reconstruct original payload by parsing the binary blobs.

"""
import io
import queue
import struct
from typing import List


def decode_blob(blob: bytes, fmt="=ffffii"):
    """
    Decode sensor data array from tinyK22.

    NOTE: No error is raised if the byte length matches, e.g. "=ffffii"
    matches "=ddd" well, so no error is raised. Perform sanity checks.
    """
    try:
        return struct.unpack(fmt, blob)
    except struct.error:
        import binascii
        error_blob = binascii.hexlify(bytearray(blob))
        raise ValueError(f"Unable to decode data, got: {error_blob}")


def combine_messages(blobs: List[bytes]) -> bytes:
    """
    Combines multiple UART messages into a single data buffer.
    Assumes structure described in the module docstring. Start- and
    """
    buf = []
    for blob in blobs:
        stop_bit = blob[-1]
        buf.append(blob[1:9])

        if stop_bit == 0x01:
            break

    return b''.join(buf)


class Sensors:
    """
    Continuously read sensor data from the tinyK22.
    """

    def __init__(self, device: io.BytesIO, out_queue: queue.Queue, fmt="=xffffiix"):
        self.device = device
        self.out_queue = out_queue
        self.fmt = fmt

    def read(self):
        """
        Read and decode sensor signals.
        """
        blob = self.device.read()
        if blob:
            values = decode_blob(blob, self.fmt)
            self.out_queue.put(values)

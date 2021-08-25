"""
Test the communication between stellar and the tinyk22.
"""
import io
import queue
import struct

import pytest
import numpy as np

from stellar.perception.sourcing import decode_blob, combine_messages, Sensors


def test_decode_blob_should_decode_sensor_data_into_correct_types():
    """
    Ensure decode_blob returns correct values after unpacking.
    """
    values = (0.3, 0.2, 0.1)
    blob = struct.pack("fff", *values)
    decoded = decode_blob(blob, fmt="fff")
    assert decoded == pytest.approx(values, 0.001)


def test_decode_blob_should_raise_error_on_invald_data():
    """
    Ensure decode_blob raises a ValueError if incoming binary data does not
    match a given format.
    """
    values = (0.3, 0.2, 0.1)
    blob = struct.pack("ddd", *values)
    with pytest.raises(ValueError):
        decode_blob(blob, fmt="fff")


def test_should_reassemble_fragmented_uart_messages():
    """
    Should correctly combine multiple UART packets into a single data buffer.
    """
    messages = [
        struct.pack("=bffb", 1, 0.1, 0.2, 0),
        struct.pack("=bffb", 1, 0.1, 0.2, 0),
        struct.pack("=bffb", 0, 0.3, 0.4, 1),
    ]
    expected = struct.pack("=ffffff", 0.1, 0.2, 0.1, 0.2, 0.3, 0.4)
    assert combine_messages(messages) == expected


def test_should_reassemble_fragmented_uart_messages_with_multiple_types():
    """
    Should correctly combine multiple UART packets with multiple data types
    into a single data buffer.
    """
    values = (0.1, 0.1, 0.1, 0.1, 2, 3)
    expected = struct.pack("=ffffii", *values)

    messages = [
        struct.pack("=bffb", 1, 0.1, 0.1, 0),
        struct.pack("=bffb", 0, 0.1, 0.1, 0),
        struct.pack("=biib", 0,   2,   3, 1),
    ]

    databuffer = combine_messages(messages)

    assert databuffer == expected
    assert decode_blob(databuffer) == pytest.approx(values, 0.001)


class TestSensorIntegration(object):
    """
    Ensure sensor signals can be retrieved from other components.
    """

    def test_should_write_sensor_data_into_queue(self):
        """
        Ensure Sensors writes signals in a queue with the correct format.
        """
        outqueue = queue.Queue()
        values = (0.1, 0.1, 0.1, 0.1, 2, 3)
        message = struct.pack("=bffffiib", 0, *values, 1)

        device = io.BytesIO(message)
        s = Sensors(device, outqueue)
        s.read()

        decoded_values = outqueue.get()

        assert values == pytest.approx(decoded_values, 0.1)

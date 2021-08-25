"""
Tests for pylon detection.
"""
import pytest

from stellar.perception.pylon_detection import PylonDetector


def test_not_crashing():
    """
    Ensure pylon detection doesn't crash.
    """

    test_image_name = "0001"
    test_image = PylonDetector.load_image(
        f"tests/pylon_test_images/{test_image_name}.jpg")

    pylons_found = PylonDetector.find_pylons(test_image)

    image_out = PylonDetector.mark_pylons(test_image, pylons_found)
    PylonDetector.write_image_debug(image_out, "final")

    assert True


def test_0389_has_3_pylons():
    """
    Ensure PylonDetector detects the correct amount of pylons.
    """

    test_image_name = "0389"
    test_image = PylonDetector.load_image(
        f"tests/pylon_test_images/{test_image_name}.jpg")

    pylons_found = PylonDetector.find_pylons(test_image)

    image_out = PylonDetector.mark_pylons(test_image, pylons_found)
    PylonDetector.write_image_debug(image_out, "final")

    assert len(pylons_found) == 3


def test_distance_is_80():
    """
    Ensure PylonDetector detects the correct distance.
    """

    test_image_name = "distance-test"
    test_image = PylonDetector.load_image(
        f"tests/pylon_test_images/{test_image_name}.jpg")

    pylons_found = PylonDetector.find_pylons(test_image)

    image_out = PylonDetector.mark_pylons(test_image, pylons_found)
    PylonDetector.write_image_debug(image_out, "final")

    assert pylons_found[0].estimated_distance_cm == pytest.approx(80, 0.1)


@pytest.mark.skip(reason="Primarily used to create pylon recognition proof-of-concept video")
def test_picam_simulator():
    """
    Ensures PicamSimulator works
    """

    from stellar.perception.picam_simulator import PicamSimulator
    picam_sim = PicamSimulator("stellar/perception/cv_video_final.mp4")

    is_test_running = True
    frame_count = 0
    while is_test_running:
        has_frame, current_frame = picam_sim.get_next_frame()

        if has_frame:
            frame_count += 1

            pylons_found = PylonDetector.find_pylons(current_frame)
            image_out = PylonDetector.mark_pylons(current_frame, pylons_found)
            PylonDetector.write_image_debug(image_out, "final")

        is_test_running = has_frame

    assert frame_count == 742

import unittest
import os

import pylibcamera.wrapper


class TestLibCameraWrapper(unittest.TestCase):
    def test_camera_manager_wrapper(self):
        camera_manager = pylibcamera.wrapper.PyCameraManager()
        try:
            assert camera_manager.get_n_cameras() >= 0
        finally:
            camera_manager.close()

    def test_everything(self):
        if not os.path.exists("/dev/media0"):
            self.skipTest("Requires a media device")
        camera = pylibcamera.wrapper.LibCameraWrapper(0)

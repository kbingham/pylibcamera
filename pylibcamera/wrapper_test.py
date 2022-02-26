import unittest

import pylibcamera.wrapper




class TestLibCameraWrapper(unittest.TestCase):
    def test_camera_manager_wrapper(self):
        camera_manager = pylibcamera.wrapper.PyCameraManager()
        assert camera_manager.get_n_cameras() == 0

    def test_everything(self):
        self.skipTest("Requires a media device")
        camera = pylibcamera.wrapper.LibCameraWrapper(0)

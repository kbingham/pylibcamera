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

    def test_camera_manager_wrapper_multiple(self):
        for i in range(3):
            camera_manager = pylibcamera.wrapper.PyCameraManager()
            camera_manager.close()

    def test_everything(self):
        camera_manager = pylibcamera.wrapper.PyCameraManager()
        camera = camera_manager.get_camera(0)
        camera.close()
        camera_manager.close()
        print("FP?")
        camera.configure()


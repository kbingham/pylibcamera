import unittest

import tom.camera.libcamera_wrapper


class TestLibCameraWrapper(unittest.TestCase):
    def test_init_delete(self):
        w = tom.camera.libcamera_wrapper.LibCameraWrapper(0, debug=True)
        del w

import unittest

import pylibcamera.wrapper


class TestLibCameraWrapper(unittest.TestCase):
    def test_init_delete(self):
        w = pylibcamera.wrapper.LibCameraWrapper(0, debug=True)
        del w

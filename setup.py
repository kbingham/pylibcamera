from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize

ext = Extension(
    'pylibcamera.wrapper', 
    sources=["pylibcamera/wrapper.pyx"],
    include_dirs = ["/usr/local/include/libcamera"],
    library_dirs = ["/usr/local/lib/x86_64-linux-gnu"],
    libraries=["camera"],
    extra_compile_args= ["-std=c++17"],
    language="c++")

setup(name="pylibcamera", ext_modules = cythonize([ext]))

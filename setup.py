from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize

import subprocess

# https://stackoverflow.com/questions/60174152/how-do-i-add-pkg-config-the-setup-py-of-a-cython-wrapper
def pkgconfig(package):
    # initialise kw to support pkg-config empty args
    kw = { 'include_dirs': [], 'library_dirs': [], 'libraries': [], }
    flag_map = {'-I': 'include_dirs', '-L': 'library_dirs', '-l': 'libraries'}
    output = subprocess.getoutput(
        'pkg-config --cflags --libs {}'.format(package))
    for token in output.strip().split():
        kw.setdefault(flag_map.get(token[:2]), []).append(token[2:])
    return kw

libcamera_args = pkgconfig('libcamera')

ext = Extension(
    'pylibcamera.wrapper', 
    sources=["pylibcamera/wrapper.pyx"],
    include_dirs=libcamera_args["include_dirs"],
    library_dirs = libcamera_args['library_dirs'],
    libraries = libcamera_args['libraries'],
    extra_compile_args= ["-std=c++17"],
    language="c++")

setup(name="pylibcamera", ext_modules = cythonize([ext], gdb_debug=True) )

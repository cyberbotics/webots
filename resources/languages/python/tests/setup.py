# python3 setup.py build_ext --inplace

from distutils.core import setup, Extension
setup(ext_modules=[Extension('arraycuckoo', ['arraycuckoo.c'])])

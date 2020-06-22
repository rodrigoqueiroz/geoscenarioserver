#! /usr/bin/env python
from setuptools import setup, Extension
from io import open
import subprocess
from os import path
import os
import sys
from setuptools.command.build_ext import build_ext

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)
class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: " +
                ", ".join(e.name for e in self.extensions))
        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(
            os.path.dirname(self.get_ext_fullpath(ext.name)))
        # python version
        self.python_version = sys.version_info.major
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]
        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        if self.python_version == 3:
            cmake_args += ['-DUSE_PYTHON3=ON']
        else:
            cmake_args += ['-DUSE_PYTHON3=OFF']
        build_args += ['--', '-j4']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get('CXXFLAGS', ''),
            self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args,
                              cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args,
                              cwd=self.build_temp)

setup(name='lanelet2',
      version='0.0.1',
      description='A standalone python module for the lanelet2 library.',
      url='https://github.com/yuzhangbit/lanelet2_standalone',
      author='yuzhangbit',
      author_email='yu.zhang.bit@gmail.com',
      classifiers=[  # Optional
        'Intended Audience :: Developers',
        'Topic :: Software Development :: autonomous driving',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 2.7',
      ],
      license='BSD 3-Clause',
      packages=['lanelet2'],
      package_dir={'lanelet2':'pylanelet2/lanelet2'},
      ext_modules=[CMakeExtension('lanelet2')],
      cmdclass=dict(build_ext=CMakeBuild),
      zip_safe=False,)

import os
import subprocess
import sys
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name):
        super().__init__(name, sources=[])


class CMakeBuild(build_ext):
    def run(self):
        for ext in self.extensions:
            self._build(ext)

    def _build(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        src_dir = os.path.abspath(os.path.dirname(__file__))

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            "-DBUILD_PYTHON=ON",
            "-DBUILD_EXAMPLES=OFF",
            f"-DPython_EXECUTABLE={sys.executable}",
        ]
        build_dir = os.path.join(self.build_temp, "cmake_build")
        os.makedirs(build_dir, exist_ok=True)

        subprocess.check_call(["cmake", "-S", src_dir, "-B", build_dir] + cmake_args,
                              cwd=src_dir)
        subprocess.check_call(["cmake", "--build", build_dir,
                               "--target", "florid_usb_py", "-j"],
                              cwd=src_dir)


setup(
    name="florid-usb",
    version="0.1.0",
    description="Ragtime Florid USB SDK — MIT direct control over USB CDC",
    ext_modules=[CMakeExtension("florid_usb")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
    python_requires=">=3.8",
)

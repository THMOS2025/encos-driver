import os
import sys
import subprocess
from pathlib import Path

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = ""):
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(Path(sourcedir).resolve())


class CMakeBuild(build_ext):
    def build_extension(self, ext: Extension):
        if not isinstance(ext, CMakeExtension):
            super().build_extension(ext)
            return

        ext_fullpath = Path(self.get_ext_fullpath(ext.name)).resolve()
        extdir = ext_fullpath.parent
        cfg = "Debug" if self.debug else "Release"

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPython3_EXECUTABLE={sys.executable}",
            "-DBUILD_PYTHON_BINDINGS=ON",
            f"-DCMAKE_BUILD_TYPE={cfg}",
        ]

        build_args = ["--config", cfg, "--target", "encos_python"]

        build_temp = Path(self.build_temp) / ext.name
        build_temp.mkdir(parents=True, exist_ok=True)

        subprocess.check_call(["cmake", ext.sourcedir, *cmake_args], cwd=build_temp)
        subprocess.check_call(["cmake", "--build", ".", *build_args], cwd=build_temp)


setup(
    ext_modules=[CMakeExtension("encos_python", sourcedir=".")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
)

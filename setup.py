from setuptools import setup

setup(
    # Most metadata is now in pyproject.toml
    # We just need to tell it where the built .so files are
    packages=[""],
    package_data={"": ["encos_python*.so"]},
)

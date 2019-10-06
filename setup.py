import sys
sys.path.pop(0)
from setuptools import setup
from codecs import open
from os import path

cwd = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(cwd, "README.md"), encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="python_mpu9250",
    py_modules=["mpu9250", "mpu6500", "ak8963"],
    version="0.1.0",
    description="CircuitPython I2C driver for MPU9250 9-axis motion tracking device",
    long_description=long_description,
    long_description_content_type="text/markdown",
    keywords="accelerometer, gyro, magnetometer, python, i2c",
    url="https://github.com/wallarug/circuitpython_mpu9250.git",
    author="Mika Tuupola",
    author_email="tuupola@appelsiini.net",
    maintainer="Cian Byrne",
    maintainer_email="cian@roboticsmasters.co",
    license="MIT",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Programming Language :: CircuitPython :: Implementation :: CPython",
        "License :: OSI Approved :: MIT License",
    ],
)

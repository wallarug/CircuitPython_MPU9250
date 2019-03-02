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
    description="Python I2C driver for MPU9250 9-axis motion tracking device",
    long_description=long_description,
    long_description_content_type="text/markdown",
    keywords="accelerometer, gyro, magnetometer, python, i2c",
    url="https://github.com/eike-welk/python_mpu9250.git",
    author="Mika Tuupola",
    author_email="tuupola@appelsiini.net",
    maintainer="Eike Welk",
    maintainer_email="eike.welk@gmx.net",
    license="MIT",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Programming Language :: Python :: Implementation :: CPython",
        "License :: OSI Approved :: MIT License",
    ],
)

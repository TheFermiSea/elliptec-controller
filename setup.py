#!/usr/bin/env python3
"""
Setup script for the elliptec-controller package.
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="elliptec-controller",
    version="0.1.0",
    author="Brian Squires",
    author_email="squires.b@gmail.com",
    description="A Python controller for Thorlabs Elliptec rotators",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/TheFermiSea/elliptec-controller",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering :: Physics",
        "Development Status :: 4 - Beta",
    ],
    python_requires=">=3.6",
    install_requires=[
        "pyserial>=3.5",
    ],
    entry_points={
        'console_scripts': [
            'elliptec-controller=elliptec_controller.cli:main',
        ],
    },
    keywords="thorlabs, elliptec, optical, rotator, controller, hardware",
)
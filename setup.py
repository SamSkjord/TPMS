#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

# Read the contents of README.md
with open("README.md", encoding="utf-8") as f:
    long_description = f.read()

# Read the requirements
with open("requirements.txt", encoding="utf-8") as f:
    requirements = f.read().splitlines()

setup(
    name="tpms",
    version="2.0.1",
    description="A Python library for interfacing with TPMS (Tire Pressure Monitoring System) devices",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Sam Conway",
    author_email="pypi@invertica.co.uk",
    url="https://github.com/samskjord/tpms",  # Update with your repository URL
    packages=find_packages(),
    install_requires=requirements,
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Software Development :: Libraries",
        "Topic :: System :: Hardware",
        "Topic :: System :: Hardware :: Hardware Drivers",
    ],
    python_requires=">=3.6",
    keywords="tpms, tire pressure, monitoring, sensors, automotive",
    project_urls={
        "Bug Reports": "https://github.com/samskjord/tpms/issues",
        "Source": "https://github.com/samskjord/tpms",
    },
    entry_points={
        "console_scripts": [
            "tpms-monitor=tpms_lib.example:main",
        ],
    },
)

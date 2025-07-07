#!/usr/bin/env python3

import pybind11
from pybind11.setup_helpers import Pybind11Extension, build_ext
from pybind11 import get_cmake_dir
import pybind11.setup_helpers
from setuptools import setup, Extension
import os
import sys
from pathlib import Path

# Get the directory containing this setup.py file
project_root = Path(__file__).parent.absolute()

# Define source files with absolute paths
source_files = [
    "src/python_bindings.cpp",
    "src/py_pibt.cpp",
    "src/pibt.cpp", 
    "src/solver.cpp",
    "src/problem.cpp",
    "src/graph.cpp",
    "src/node.cpp",
    "src/pos.cpp",
    "src/plan.cpp",
    "src/paths.cpp",
]

# Convert to absolute paths
source_files = [str(project_root / src) for src in source_files]

# Define the extension module
ext_modules = [
    Pybind11Extension(
        "pypibt",
        source_files,
        include_dirs=[
            # Path to pybind11 headers
            pybind11.get_include(),
            # Local include directory (absolute path)
            str(project_root / "include"),
        ],
        language='c++',
        cxx_std=17,  # Use C++17 standard
    ),
]

# Additional compiler flags
if sys.platform == "win32":
    # Windows-specific flags
    for ext in ext_modules:
        ext.cxx_flags = ["/std:c++17", "/O2"]
else:
    # Unix-like systems (Linux, macOS)
    for ext in ext_modules:
        ext.cxx_flags = ["-std=c++17", "-O3", "-g"]

setup(
    name="pypibt",
    version="1.0.0",
    author="PIBT Python Bindings",
    description="Python bindings for PIBT multi-agent pathfinding solver",
    long_description="""
    PyPIBT provides Python bindings for the PIBT (Priority Inheritance with Backtracking) 
    multi-agent pathfinding solver. This allows you to:
    
    - Initialize PIBT with grid maps, agent start/goal positions, and optional edge weights
    - Perform step-by-step pathfinding for real-time applications
    - Support both MAPF and LMAPF (Lifelong MAPF) scenarios
    - Handle dynamic goal changes during execution
    
    The library is particularly useful for robotics applications, game AI, and warehouse 
    automation where multiple agents need to navigate efficiently without collisions.
    """,
    long_description_content_type="text/plain",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.6",
    install_requires=[
        "pybind11>=2.6.0",
    ],
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7", 
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: C++",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    keywords="pathfinding, multi-agent, MAPF, PIBT",
) 
#!/usr/bin/env python

import sys
from os import path, walk
from setuptools import setup, find_packages

def print_error(*args, **kwargs):
    """ Print in stderr. """
    print(*args, file=sys.stderr, **kwargs)


def find_resources(package_name):
    """ Find the relative path of files under the resource folder. """
    resources = []
    package_dir = path.join("python", package_name)
    resources_dir = package_dir 

    for (root, _, files) in walk(resources_dir):
        for afile in files:
            if (
                afile != package_name
                and not afile.endswith(".DS_Store")
                and not afile.endswith(".py")
            ):
                rel_dir = path.relpath(root, package_dir)
                src = path.join(rel_dir, afile)
                resources.append(src)
    return resources


# Package name.
package_name = "mim_robots"

# Long description from the readme.
with open(
    path.join(path.dirname(path.realpath(__file__)), "README.md"), "r"
) as fh:
    long_description = fh.read()

# Find the resource files.
resources = find_resources(package_name)

# Final setup.
setup(
    name=package_name,
    version="1.0.0",
    package_dir={package_name: path.join("python", package_name)},
    packages=[package_name] + [package_name + '.' + pkg for pkg in find_packages(path.join("python", package_name))], 
    package_data={package_name: resources},
    install_requires=[
        "setuptools",
        "mujoco",
        "pinocchio",
        "pybullet",
        "robot_descriptions",
        "importlib_resources"
    ],
    zip_safe=True,
    maintainer="hz",
    maintainer_email="hzhu@nyu.edu",
    long_description=long_description,
    long_description_content_type="text/markdown",
    description="Robot configuration files and wrappers around Mujoco",
    license="BSD-3-clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD-3-clause",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8",
)
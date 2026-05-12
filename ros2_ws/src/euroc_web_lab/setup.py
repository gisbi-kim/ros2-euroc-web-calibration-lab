from glob import glob
from setuptools import find_packages, setup

package_name = "euroc_web_lab"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/static", glob(f"{package_name}/static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Advanced Mobility Instructor",
    maintainer_email="instructor@example.com",
    description="ROS2 Jazzy EuRoC camera streaming, capture, and undistortion web lab.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_bridge_node = euroc_web_lab.web_bridge_node:main",
        ],
    },
)

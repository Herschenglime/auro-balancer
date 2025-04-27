from setuptools import find_packages, setup
import os
from glob import glob

package_name = "auro_balancer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robo",
    maintainer_email="herschenglime@gmail.com",
    description="Self-balancing ball seesaw for EEL5934 final.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dist_publisher = auro_balancer.dist_publisher:main",
            "imu_publisher = auro_balancer.imu_publisher:main",
            "servo_controller = auro_balancer.servo_controller:main",
            "kalman_pid = auro_balancer.kalman_pid:main",
        ],
    },
)

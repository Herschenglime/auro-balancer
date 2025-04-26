from setuptools import find_packages, setup

package_name = "auro_balancer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robo",
    maintainer_email="herschenglime@gmail.com",
    description="Self-balancing ball seesaw for EEL5934 final.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["controller = auro_balancer.controller:main"],
    },
)

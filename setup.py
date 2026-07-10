from setuptools import find_packages, setup

package_name = "acceleration_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/acceleration_planner_launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Dani",
    maintainer_email="dani@example.com",
    description="Acceleration planner Lart",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cone_publisher = acceleration_planner.cone_publisher_node:main",
            "planner_node = acceleration_planner.planner_node:main",
        ],
    },
)

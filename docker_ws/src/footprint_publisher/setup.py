from setuptools import find_packages, setup

package_name = "footprint_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Switches Nav2 costmap footprint based on /enabled Bool topic",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "footprint_publisher = footprint_publisher.footprint_publisher_node:main",
        ],
    },
)
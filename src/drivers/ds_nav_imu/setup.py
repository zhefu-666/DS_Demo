from setuptools import find_packages, setup

package_name = "ds_nav_imu"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="liu",
    maintainer_email="2963167340@qq.com",
    description="HiPNUC serial IMU driver for ds_nav_create.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "hipnuc_imu_node = ds_nav_imu.hipnuc_imu_node:main",
        ],
    },
)

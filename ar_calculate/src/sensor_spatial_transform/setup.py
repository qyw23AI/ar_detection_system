from setuptools import find_packages, setup


package_name = "sensor_spatial_transform"


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sensor_spatial_transform.launch.py"]),
        ("share/" + package_name + "/config", ["config/sensor_spatial_transform.params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Layer3 sensor spatial transform for dynamic gimbal compensation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gimbal_tf_handler = sensor_spatial_transform.gimbal_tf_handler:main",
            "ar_pose_adapter = sensor_spatial_transform.ar_pose_adapter:main",
        ],
    },
)
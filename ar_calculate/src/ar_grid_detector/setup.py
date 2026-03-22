from setuptools import setup, find_packages

package_name = 'ar_grid_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ar_grid.launch.py']),
        ('share/' + package_name + '/config', ['config/ar_grid.params.yaml', 'config/ar_grid.layer3.params.yaml', 'config/grid_example.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='AR Grid Detector with fisheye support and flexible grid mapping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ar_grid_node = ar_grid_detector_py.ar_grid_node:main',
        ],
    },
)

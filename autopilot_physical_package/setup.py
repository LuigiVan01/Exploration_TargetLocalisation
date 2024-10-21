from setuptools import find_packages, setup

package_name = 'autopilot_physical_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rico',
    maintainer_email='ricardowest1243@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['aruco_detect_robot=autopilot_physical_package.aruco_node_robot:main'
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autopilot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        *[
    		(os.path.join('share', package_name, model_dir), [os.path.join(model_dir, f) for f in files])
    		for model_dir, _, files in os.walk('models')
	]
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rico',
    maintainer_email='ricardowest1243@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['autopilot=autopilot_package.autopilot:main',
        	'aruco_detect=autopilot_package.aruco_node:main'
        ],
    },
)

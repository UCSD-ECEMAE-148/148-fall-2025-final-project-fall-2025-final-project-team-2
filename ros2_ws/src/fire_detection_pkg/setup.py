from setuptools import setup
from glob import glob
import os

package_name = 'fire_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighth@ucsd.edu',
    description='Fire detection and summit cost logic',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # this is the important line:
            'fire_detection_node = fire_detection_pkg.fire_detection_node:main',
            'oak_viewer = fire_detection_pkg.oak_viewer:main',
        ],
    },
)

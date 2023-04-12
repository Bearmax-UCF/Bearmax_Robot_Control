import os
from setuptools import setup
from glob import glob

package_name = 'bearmax_wsp'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Node to collect sensor data from WSP and publish on a ROS 2 topic for use by Bearmax stress detection algorithms',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wsp_connector = bearmax_wsp.wsp_connector:main'
        ],
    },
)

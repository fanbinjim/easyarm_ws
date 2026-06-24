from glob import glob

from setuptools import find_packages, setup

package_name = 'easyarm_web_bridge'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('README.md')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linx',
    maintainer_email='fanbinjim@qq.com',
    description='ROS 2 HTTP and WebSocket backend bridge for EasyArm motion debugging.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'easyarm_web_bridge = easyarm_web_bridge.server:main',
        ],
    },
)

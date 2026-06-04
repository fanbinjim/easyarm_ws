from setuptools import find_packages, setup

package_name = 'easyarm_calib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linx',
    maintainer_email='fanbinjim@qq.com',
    description='Calibration tools for EasyArm cameras and robot zero offsets.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_preview = easyarm_calib.camera_preview:main',
            'calibrate_camera = easyarm_calib.calibrate_camera:main',
            'collect_joint_zero_vision = easyarm_calib.collect_joint_zero_vision:main',
            'optimize_joint_zero_vision = easyarm_calib.optimize_joint_zero_vision:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'easyarm_utils'

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
    maintainer='linx',
    maintainer_email='fanbinjim@qq.com',
    description='Utility scripts for EasyArm recorded data.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = easyarm_utils.keyboard_teleop:main',
            'plot_ee_trajectory = easyarm_utils.plot_ee_trajectory:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'easyarm_task'

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
    description='Task-level applications for EasyArm.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'easyarm_ball_balance = easyarm_task.ball_balance:main',
            (
                'easyarm_ball_balance_color_benchmark = '
                'easyarm_task.ball_balance_color_benchmark:main'
            ),
        ],
    },
)

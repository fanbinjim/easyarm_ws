from setuptools import find_packages, setup

package_name = 'easyarm_app'

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
    description='Application-level CLI tools for EasyArm motion commands.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'easyarm = easyarm_app.cli:console_main',
            'easyarm_shell = easyarm_app.cli:shell_main',
        ],
    },
)

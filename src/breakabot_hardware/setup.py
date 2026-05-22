from setuptools import find_packages, setup

package_name = 'breakabot_hardware'

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
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'imu_node = breakabot_hardware.imu_node:main',
        'relay_board_node = breakabot_hardware.relay_board_node:main',
        'roboteq_node = breakabot_hardware.roboteq_node:main',
    ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'nun_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labtech',
    maintainer_email='labtech@todo.todo',
    description='Commande série Arduino depuis ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_listener = nun_bot.arduino_subscriber:main',
            'keyboard_talker = nun_bot.keyboard_command_publisher:main',
            'ultrasonic_talker = nun_bot.ultrasonic_sensor_publisher:main',
        ],
    },
)

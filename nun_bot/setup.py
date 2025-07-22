from setuptools import find_packages, setup

package_name = 'nun_bot_fixed'

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
    maintainer='labtech',
    maintainer_email='labtech@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_talker = nun_bot.ultrasonic_sensor_publisher:main',
            'keyboard_talker = nun_bot.keyboard_command_publisher:main'
            'arduino_listener = nun_bot.arduino_subscriber:main'
        ],
    },
)

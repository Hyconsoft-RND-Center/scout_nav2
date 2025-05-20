from setuptools import setup

package_name = 'sonar_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roas',
    maintainer_email='kjkim@roas.co.kr',
    description='Sonar sensor data handling',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'sonar_sensor = sonar_ros.sonar_sensor:main',
            'relay_node = sonar_ros.relay_node:main',
        ],
    },
)

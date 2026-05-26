from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'models', 'turtlebot3_waffle'),
            glob(os.path.join('models', 'turtlebot3_waffle', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danya',
    maintainer_email='danya@danya02.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'square_mover = main.square_mover:main',
            'obstacle_avoider = main.obstacle_avoider:main',
        ],
    },
)

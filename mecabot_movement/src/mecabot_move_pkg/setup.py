import os
from glob import glob
from setuptools import setup

package_name = 'mecabot_move_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package to move the Mecabot forward and backward.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot = mecabot_move_pkg.move_robot:main',
            'move_in_square = mecabot_move_pkg.move_in_square:main',
	    'move_square_differential = mecabot_move_pkg.move_square_differential:main',
        ],
    },
)

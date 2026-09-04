from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'erc_perception'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mira',
    maintainer_email='merayss26@hotmail.com',
    description=(
        'Camera and computer-vision package '
        'for the ERC 2026 challenge.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            (
                'camera_subscriber = '
                'erc_perception.camera_subscriber:main'
            ),
        ],
    },
)

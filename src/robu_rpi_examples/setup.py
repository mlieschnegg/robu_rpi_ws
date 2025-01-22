from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robu_rpi_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robu',
    maintainer_email='li@htl-kaindorf.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'rgbstrip_sub=robu_rpi_examples.rgbstrip_sub:main',
             'rgbstrip_pub=robu_rpi_examples.rgbstrip_pub:main',
             'ledstrip_sub=robu_rpi_examples.ledstrip_sub:main'
        ],
    },
)

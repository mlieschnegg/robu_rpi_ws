from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robuboard'

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
            "poweroff_robuboard = robuboard.gpioctrl:main_poweroff_robuboard",
            "reset_teensy = robuboard.gpioctrl:main_reset_teensy",
            "poweron_teensy = robuboard.gpioctrl:main_reset_teensy",
            "poweroff_teensy = robuboard.gpioctrl:main_poweroff_teensy",
            "powerswitch = robuboard.gpioctrl:main_powerswitch",
            "set_status_led = robuboard.gpioctrl:main_set_status_led",
            "start_bootloader_teensy = robuboard.gpioctrl:main_start_bootloader_teensy",
            "upload_firmware_teensy = robuboard.gpioctrl:main_upload_firmware_teensy",
            "build_firmware_teensy = robuboard.gpioctrl:main_build_firmware_teensy",
            "start_firmware_teensy = robuboard.gpioctrl:main_start_firmware_teensy",
            "build_upload_firmware_teensy = robuboard.gpioctrl:main_build_upload_firmware_teensy",
        ],
    },
)

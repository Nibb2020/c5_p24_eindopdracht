# =========================================================
# Imports
# =========================================================

import os  # Padfuncties
from glob import glob  # Zoekt bestanden met patroon

from setuptools import find_packages, setup  # Setupfuncties voor Python package

# =========================================================
# Package Config
# =========================================================

package_name = 'vision'  # Naam van het ROS2 package

setup(
    name=package_name,  # Package naam
    version='0.0.0',  # Package versie
    packages=find_packages(exclude=['test']),  # Zoek Python packages
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),  # Registreer package
        (
            'share/' + package_name,
            ['package.xml'],
        ),  # Installeer package.xml
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),  # Installeer launchbestanden
    ],
    install_requires=['setuptools'],  # Setup dependency
    zip_safe=True,  # Sta zip-install toe
    maintainer='student',  # Maintainer naam
    maintainer_email='hesselkeijzer847@gmail.com',  # Maintainer mail
    description='Vision package for OAK-D object detection.',  # Beschrijving
    license='Apache-2.0',  # Licentie
    extras_require={
        'test': [
            'pytest',
        ],
    },  # Test dependency
    entry_points={
        'console_scripts': [
            'vision_node = vision.vision_node:main',
        ],
    },  # Maakt ros2 run vision vision_node mogelijk
)
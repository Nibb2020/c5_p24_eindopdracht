from setuptools import find_packages, setup  # Importeer setuptools functies

package_name = 'vision'  # Naam van het ROS2 Python package

setup(
    name = package_name,  # Package naam
    version = '0.0.0',  # Package versie
    packages = find_packages(exclude=['test']),  # Zoek Python packages behalve test
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Registreer package in ament index
        ('share/' + package_name, ['package.xml']),  # Installeer package.xml
    ],
    install_requires = ['setuptools'],  # Vereiste Python package dependency
    zip_safe = True,  # Package mag als zip geïnstalleerd worden
    maintainer = 'student',  # Maintainer naam
    maintainer_email = 'hesselkeijzer847@gmail.com',  # Maintainer e-mail
    description = 'Vision package for OAK-D object detection and robot object pose output',  # Package beschrijving
    license = 'Apache-2.0',  # Licentie
    extras_require = {
        'test': [
            'pytest',  # Test dependency
        ],
    },
    entry_points = {
        'console_scripts': [
            'vision_node = vision.vision_node:main',  # Maakt ros2 run vision vision_node mogelijk
        ],
    },
)
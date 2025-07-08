from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gap_follow_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # índice del paquete
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        # package.xml
        ("share/" + package_name, ["package.xml"]),
        # todos los launch del sub-directorio
        ("share/" + package_name + "/launch",
         glob(os.path.join("launch", "*.py"))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jupazamo',
    maintainer_email='jupazamo@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'gap_follower = gap_follow_pkg.gap_follower:main',
        "lap_timer    = gap_follow_pkg.lap_timer:main",
        ],
    },
)
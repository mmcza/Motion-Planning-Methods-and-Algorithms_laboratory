from setuptools import setup
import os
from glob import glob

package_name = 'mapr_6_student'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bartlomiej Kulecki',
    maintainer_email='bartlomiej.kulecki@put.poznan.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt = mapr_6_student.rrt:main',
            'rrt_vertices = mapr_6_student.rrt_vertices:main',
            'points = mapr_6_student.points:main',
        ],
    },
)

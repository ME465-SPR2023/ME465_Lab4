from setuptools import setup
from glob import glob

package_name = 'ME465_Lab4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.py')),
        ('share/' + package_name, glob("rviz/*.rviz")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='ME465 Lab 4',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab4_node = ME465_Lab4.lab4_node:main'
        ],
    },
)

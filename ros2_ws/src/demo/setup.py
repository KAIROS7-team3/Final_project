import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['ui_static/*']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KAIROS7-team3',
    maintainer_email='team@example.com',
    description='Demo launch package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'demo_trigger = demo.demo_trigger:main',
            'demo_ui = demo.ui_server:main',
        ],
    },
)

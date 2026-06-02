import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'meshes', 'rh_p12_rn_a'),
         glob('urdf/meshes/rh_p12_rn_a/*.stl')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimsungyeoun',
    maintainer_email='jasper104615@gmail.com',
    description='Motion control package: DSR arm, RL policy, RH-P12-RN gripper, RViz merger',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gripper_node = motion.gripper_node:main',
            'rviz_joint_state_merger = motion.rviz_joint_state_merger_node:main',
        ],
    },
)

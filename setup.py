from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mvp_c2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/proto', [package_name + '/proto/mvp_cmd_dccl.proto']),
        ('lib/' + package_name, [package_name + '/dccl_checksum.py']),
        ('lib/' + package_name, [package_name + '/proto/mvp_cmd_dccl_pb2.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'std_msgs','nav_msgs', 'dccl'],
    zip_safe=True,
    # py_modules=['dccl_checksum'],
    maintainer='mingxi',
    maintainer_email='mzhou@uri.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'my_node = mvp_c2.my_node:main'
            'mvp_c2_reporter = mvp_c2.mvp_c2_reporter:main',
            'mvp_c2_commander = mvp_c2.mvp_c2_commander:main'
        ],
    },
)

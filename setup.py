from setuptools import setup

package_name = 'mvp_c2'

setup(
    name=package_name,
    version='0.1.0',
    packages=['mvp_c2'],  # Package directory for your Python modules
    install_requires=['setuptools', 'rclpy', 'dccl'],  # Required dependencies
    zip_safe=True,
    maintainer='Mingxi Zhou',  # Replace with your name
    maintainer_email='mzhou@uri.edu',  # Replace with your email
    description='MVP C2 ROS 2 Package',
    entry_points={
        'console_scripts': [
            'mvp_c2_reporter = mvp_c2.mvp_c2_reporter:main',  # Entry point for the node
        ],
    },
    package_data={
        'mvp_c2': [
            'proto/*.proto',  # Ensure proto files are included
        ],
    },
)

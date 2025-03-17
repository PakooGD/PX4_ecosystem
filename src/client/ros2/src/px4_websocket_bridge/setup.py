from setuptools import find_packages, setup

package_name = 'px4_websocket_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexei',
    maintainer_email='johndoe@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'px4_websocket_node = px4_websocket_bridge.px4_websocket_node:main',
            'log_sender_node = px4_websocket_bridge.log_sender_node:main',  
        ],
    },
)
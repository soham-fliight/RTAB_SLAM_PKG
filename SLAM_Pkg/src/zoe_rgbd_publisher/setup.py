from setuptools import setup
package_name = 'zoe_rgbd_publisher'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Publish RGB + ZoeDepth as RGB-D for RTAB-Map',
    entry_points={'console_scripts': [
        'zoe_rgbd_node = zoe_rgbd_publisher.zoe_rgbd_node:main',
    ]},
)

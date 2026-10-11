from glob import glob

from setuptools import setup

package_name = 'carla_camera_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ap',
    maintainer_email='amulprakash24@gmail.com',
    description='CARLA camera -> ROS 2 -> dataset pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_publisher = carla_camera_pipeline.camera_publisher:main',
            'image_saver = carla_camera_pipeline.image_saver:main',
        ],
    },
)
